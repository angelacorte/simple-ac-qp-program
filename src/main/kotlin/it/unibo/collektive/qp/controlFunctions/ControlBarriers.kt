package it.unibo.collektive.qp.controlFunctions

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBModel
import it.unibo.collektive.qp.dsl.GRBVector
import it.unibo.collektive.qp.dsl.addCBF
import it.unibo.collektive.qp.dsl.minus
import it.unibo.collektive.qp.dsl.toQuadExpr
import it.unibo.collektive.qp.dsl.squaredNorm
import it.unibo.collektive.qp.dsl.zeroVec
import it.unibo.collektive.qp.utils.Obstacle
import it.unibo.collektive.qp.utils.Robot
import it.unibo.collektive.qp.utils.toDoubleArray
import kotlin.math.max
import kotlin.math.pow

/**
 * (OBSTACLE AVOIDANCE) linear CBF 2(p - p_o)^T u >= - \gamma [ ||p - p_o||^2 - (r_o + d_o)^2 ]
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
 * (COMMUNICATION DISTANCE) CBF -2(p1 - p2)^T (u1 -u2) + \gamma [ R^2 - ||p1 - p2||^2 ] >= 0
 * move to the right
 * -2(p1 - p2)^T (u1 -u2) >= - \gamma [ R^2 - ||p1 - p2||^2 ]
 * -2(p1 - p2)^T u1 + 2(p1 - p2)^T u2 >= - \gamma [ R^2 - ||p1 - p2||^2 ]
 * -2(p1 - p2)^T u1 >= -2(p1 - p2)^T u2 - \gamma [ R^2 - ||p1 - p2||^2 ]
 * ||p1 - p2||^2 = (p1 - p2)^T (p1 -p2)
 */
fun GRBModel.addCommunicationRangeCBF(
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
        val velocityOther: DoubleArray = connect.velocity.toDoubleArray()
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
            name = "communicationRange_${robot.id}_with_${connect.id}",
            coefU1 = -2.0,
            coefU2 = -2.0,
        )
    }
}

/**
 * (ROBOT AVOIDANCE) linear CBF 2(p1 - p2)^T (u1 - u2) + \gamma [ ||p1-p2||^2 - dmin^2 ] >= 0
 * 2(p1 - p2)^T (u1 - u2) + \gamma [ (p1-p2)^T(p1-p2) - dmin^2 ] >= 0
 * move u2 to the right
 * 2(p1 - p2)^T u1 -2 (p1 - p2)^T u2 >= - \gamma [ (p1-p2)^T(p1-p2) - dmin^2 ]
 * 2(p1 - p2)^T u1 >= 2(p1 - p2)^T u2 - \gamma [ (p1-p2)^T(p1-p2) - dmin^2 ]
 */
fun GRBModel.addRobotAvoidanceCBF(
    robotsToAvoid: List<Robot>,
    robot: Robot,
    position: DoubleArray,
    u: GRBVector,
) {
    robotsToAvoid.forEach { avoid ->
        val positionOther: DoubleArray = avoid.toDoubleArray()
        val velocityOther: DoubleArray = avoid.velocity.toDoubleArray()
        val minDistSq = max(robot.margin, avoid.margin).pow(2)
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
            name = "robotAvoidance_${robot.id}_vs_${avoid.id}",
            coefU1 = 2.0,
            coefU2 = 2.0,
        )
    }
}

/**
 * norm constraint on the control input ux^2 + uy^2 <= maxSpeed^2
 */
fun GRBModel.maxSpeedCBF(
    u: GRBVector,
    robot: Robot,
) {
    addQConstr(
        u.toQuadExpr(),
        GRB.LESS_EQUAL,
        robot.maxSpeed.pow(2),
        "u_norm"
    )
}
