package it.unibo.collektive.qp.controlFunctions

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBLinExpr
import com.gurobi.gurobi.GRBModel
import it.unibo.collektive.qp.dsl.GRBVector
import it.unibo.collektive.qp.dsl.addCBF
import it.unibo.collektive.qp.dsl.toQuadExpr
import it.unibo.collektive.qp.utils.Obstacle
import it.unibo.collektive.qp.utils.Robot
import it.unibo.collektive.qp.utils.minus
import it.unibo.collektive.qp.utils.squaredNorm
import it.unibo.collektive.qp.utils.toDoubleArray
import it.unibo.collektive.qp.utils.zeroVec
import kotlin.math.max
import kotlin.math.pow

/**
 * Adds a linear CBF that keeps the robot outside the obstacle safety radius `(r_o + d_o)`.
 */
fun GRBModel.addObstacleAvoidanceCBF(
    currentPosition: DoubleArray,
    obstacle: Obstacle,
    u: GRBVector,
    gamma: Double = 0.5, // \gamma in {0.5 .. 5} = soft || in {5, 20} = hard || > infeasible QP
) {
    // 2(p - p-g)^T u = 2(p_x - p_o,x) u_x + 2(p_y - p_o,y) u_y
    val obstaclePosition: DoubleArray = obstacle.toDoubleArray()
    val distance = currentPosition - obstaclePosition // ||p - p_o||^2
    val safeDistance = obstacle.radius + obstacle.margin
    // - \gamma [ ||p - p_o||^2 - (r_o + d_o)^2 ] ==== - \gamma ((p_x - p_o,x)^2 + (p_y - p_o,y) ^2 - (r_o + d_o)^2)
    val h = distance.squaredNorm() - safeDistance.pow(2)
    addCBF(
        p1 = currentPosition,
        p2 = obstaclePosition,
        u1 = u,
        u2 = zeroVec(u.dimensions),
        gamma = gamma,
        h = h, // ( (p_x - p_o,x)^2 + (p_y - p_o,y) ^2 - (r_o^2 + d_o^2)
        name = "obstacleAvoidance",
        coefU1 = 2.0,
        coefU2 = 0.0,
    )
}

/**
 * Adds a pairwise CBF preventing robot-robot collisions by bounding the relative velocity.
 */
fun GRBModel.addCollisionAvoidanceCBF(ui: GRBVector, uj: GRBVector, robot: Robot, other: Robot) {
    // COLLISION AVOIDANCE 2(p1 - p2)^T (u1 - u2) + \gamma [ ||p1-p2||^2 - dmin^2 ] >= 0
    // 2(p1 - p2)^T (u1 - u2) >= - \gamma [ ||p1-p2||^2 - dmin^2 ]
    val gamma = 0.5
    val distance = (robot.position - other.position).toDoubleArray()
    val maxDist = max(robot.safeMargin, other.safeMargin)
    val collision = GRBLinExpr()
    val collRight = -gamma * (distance.squaredNorm() - maxDist.pow(2))
    for (index in 0 until distance.size) {
        collision.addTerm(2.0 * distance[index], ui[index])
        collision.addTerm(-2.0 * distance[index], uj[index])
    }
    try {
        addConstr(collision, GRB.GREATER_EQUAL, collRight, "collision_avoidance_")
    } catch (e: Exception) {
        println("Error for collision avoidance CBF: ${e.message}")
    }
}

/**
 * Adds a CBF that enforces a maximum communication distance between two robots.
 */
fun GRBModel.addCommunicationRangeCBF(ui: GRBVector, uj: GRBVector, robot: Robot, other: Robot, range: Double) {
    // COMM DISTANCE -2(p1 - p2)^T (u1 -u2) + \gamma [ R^2 - ||p1 - p2||^2 ] >= 0
    // COMM DISTANCE -2(p1 - p2)^T (u1 -u2) >= - \gamma [ R^2 - ||p1 - p2||^2 ]
    val gamma = 0.5
    val distance = (robot.position - other.position).toDoubleArray()
    val communication = GRBLinExpr()
    val commRight = -gamma * (range.pow(2) - distance.squaredNorm())
    for (index in 0 until distance.size) {
        communication.addTerm(-2.0 * distance[index], ui[index])
        communication.addTerm(2.0 * distance[index], uj[index])
    }
    try {
        addConstr(communication, GRB.GREATER_EQUAL, commRight, "communication_range")
    } catch (e: Exception) {
        println("Error for collision avoidance CBF: ${e.message}")
    }
}

/**
 * Variant communication-range CBF that works against a list of candidate neighbors.
 */
fun GRBModel.maybeWrongAddCommunicationRangeCBF(
    maxConnectionDistance: Double,
    robotsToBeConnected: List<Robot>,
    position: DoubleArray,
    u: GRBVector,
    robot: Robot,
) {
    val maxDistSq = maxConnectionDistance.pow(2) // R^2
    val gamma = 0.5
    robotsToBeConnected.forEach { connect ->
        val positionOther: DoubleArray = connect.toDoubleArray()
        val velocityOther: DoubleArray = connect.control.toDoubleArray()
        // 2(p1-p2)^T u2 - \gamma [ R^2 - (p1-p2)^T(p1-p2)  ]
        // (p1-p2)^T(p1-p2) = (p1x - p2x) p1x + (p1y - p2y) p1y
        // (p1x - p2x) p1x = dxr * uxa
        // (p1y - p2y) p1y = dyr * uya
        val h = maxDistSq - (position - positionOther).squaredNorm()
        addCBF(
            p1 = positionOther,
            p2 = position,
            u1 = u,
            u2 = velocityOther,
            gamma = gamma,
            h = h,
            name = "communicationRange",
            coefU1 = -2.0,
            coefU2 = -2.0,
        )
    }
}

/**
 * Variant robot-avoidance CBF that enforces a minimum distance from a list of neighbors.
 */
fun GRBModel.maybeWrongAddRobotAvoidanceCBF(
    robotsToAvoid: List<Robot>,
    robot: Robot,
    position: DoubleArray,
    u: GRBVector,
) = robotsToAvoid.forEach { avoid ->
    val positionOther: DoubleArray = avoid.toDoubleArray()
    val velocityOther: DoubleArray = avoid.control.toDoubleArray()
    val minDistSq = max(robot.safeMargin, avoid.safeMargin).pow(2)
    val gamma = 0.5 // \gamma in {0.5 .. 5} = soft || in {5, 20} = hard || > infeasible QP
    // (p1 - p2)^T (p1 - p2) = (p1x - p2x)^2 + (p1y - p2y)^2
    val h = (position - positionOther).squaredNorm() - minDistSq
    // left side
    // 2(p1-p2)^T u1 // my velocity
    // right side
    // -2(p1-p2)^T u2 - \gamma [ (p1-p2)^T(p1-p2) - dmin^2 ]
    addCBF(
        p1 = position,
        p2 = positionOther,
        u1 = u,
        u2 = velocityOther,
        gamma = gamma,
        h = h, // -2(p1-p2)^T u2 - \gamma [ (p1-p2)^T(p1-p2) - dmin^2 ]
        name = "robotAvoidance",
        coefU1 = 2.0,
        coefU2 = 2.0,
    )
}

/**
 * Enforces a max-speed constraint `||u||^2 <= maxSpeed^2` as a quadratic constraint.
 */
fun GRBModel.maxSpeedCBF(u: GRBVector, robot: Robot) {
    addQConstr(
        u.toQuadExpr(),
        GRB.LESS_EQUAL,
        robot.maxSpeed.pow(2),
        "u_norm",
    )
}
