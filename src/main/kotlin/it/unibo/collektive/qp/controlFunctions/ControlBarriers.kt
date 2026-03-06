package it.unibo.collektive.qp.controlFunctions

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBException
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
    } catch (e: GRBException) {
        println("Error for collision avoidance CBF: ${e.message}")
    }
}

/**
 * Adds a CBF that enforces a maximum communication distance between two robots.
 */
fun GRBModel.addCommunicationRangeCBF(ui: GRBVector, uj: GRBVector, robot: Robot, other: Robot, range: Double) {
    // COMM DISTANCE -2(p1 - p2)^T (u1 -u2) + \gamma [ R^2 - ||p1 - p2||^2 ] >= 0
    // COMM DISTANCE -2(p1 - p2)^T (u1 -u2) >= - \gamma [ R^2 - ||p1 - p2||^2 ]
    val gamma = 2
    val distance = (robot.position - other.position).toDoubleArray()
    val communication = GRBLinExpr()
    val commRight = -gamma * (range.pow(2) - distance.squaredNorm())
    for (index in 0 until distance.size) {
        communication.addTerm(-2.0 * distance[index], ui[index])
        communication.addTerm(2.0 * distance[index], uj[index])
    }
    try {
        addConstr(communication, GRB.GREATER_EQUAL, commRight, "communication_range")
    } catch (e: GRBException) {
        println("Error for collision avoidance CBF: ${e.message}")
    }
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
