package it.unibo.collektive.qp.dsl

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBEnv
import com.gurobi.gurobi.GRBModel
import it.unibo.collektive.qp.utils.Obstacle
import it.unibo.collektive.qp.utils.Robot
import it.unibo.collektive.qp.utils.SpeedControl2D
import it.unibo.collektive.qp.utils.Target
import it.unibo.collektive.qp.controlFunctions.addCommunicationRangeCBF
import it.unibo.collektive.qp.controlFunctions.addObstacleAvoidanceCBF
import it.unibo.collektive.qp.controlFunctions.addRobotAvoidanceCBF
import it.unibo.collektive.qp.controlFunctions.goToTargetCLF
import it.unibo.collektive.qp.controlFunctions.maxSpeedCBF
import it.unibo.collektive.qp.utils.setLicense
import it.unibo.collektive.qp.utils.toDoubleArray

/**
       min ||u - u^nom||^2 + \delta
s.t.   2(p - p_o)^T u + \gamma [ ||p - p_o||^2 - (r_o + d_o)^2 ] >= 0 (OBSTACLE AVOIDANCE)
       2(p1 - p2)^T (u1 - u2) + \gamma [ (p1-p2)^T(p1-p2) - dmin^2 ] >= 0 (ROBOT AVOIDANCE)
       -2(p1 - p2)^T (u1 -u2) + \gamma [ R^2 - ||p1 - p2||^2 ] >= 0 (COMMUNICATION DISTANCE)
       ||u_k|| <= u_max
       2(p - p_g)^T u <= -c || p - p_g ||^2 + \delta

Find the optimal control to go towards the defined target,
without taking in account any obstacle.
 */
fun robotToTargetWithAvoidanceAndDistance(
    robot: Robot,
    target: Target,
    obstacle: Obstacle,
    robotsToAvoid: List<Robot> = emptyList(),
    robotsToBeConnected: List<Robot> = emptyList(),
    maxConnectionRange: Double = Double.MIN_VALUE,
): SpeedControl2D {
    setLicense() // Tell Gurobi exactly where the license is
    val env = GRBEnv(true) // create environment in manual mode (because of license file specification)
    env.start()
    val model = GRBModel(env) // create an optimization model inside the environment
    // decision variables
    // control input (velocity or displacement) bounds represent admissible control directions
    val u: GRBVector = model.addVecVar(dimension = robot.position.dimension, lowerBound = -robot.maxSpeed, upperBound = robot.maxSpeed, name = "u")
    val delta = model.addVar(0.0, GRB.INFINITY, 0.0, GRB.CONTINUOUS, "delta") // slack variable
    val position: ScalarVector = robot.toDoubleArray()
    // (OBSTACLE AVOIDANCE) linear CBF 2(p - p_o)^T u >= - \gamma [ ||p - p_o||^2 - (r_o + d_o)^2 ]
    model.addObstacleAvoidanceCBF(position, obstacle, u)
    //  (ROBOT AVOIDANCE) linear CBF 2(p1 - p2)^T (u1 - u2) + \gamma [ (p1-p2)^T(p1-p2) - dmin^2 ] >= 0
    // move u2 to the right
    // 2(p1 - p2)^T u1 >= -2(p1 - p2)^T u2 - \gamma [ (p1-p2)^T(p1-p2) - dmin^2 ]
    if (robotsToAvoid.isNotEmpty()) {
        model.addRobotAvoidanceCBF(robotsToAvoid, robot, position, u)
    }
    // (COMMUNICATION DISTANCE) CBF -2(p1 - p2)^T (u1 -u2) + \gamma [ R^2 - ||p1 - p2||^2 ] >= 0
    if (robotsToBeConnected.isNotEmpty() && maxConnectionRange != Double.MIN_VALUE) {
        check(maxConnectionRange > 0) {
            "Please specify a valid connection distance, currently is $maxConnectionRange, which lead to infeasible problem"
        }
        model.addCommunicationRangeCBF(maxConnectionRange, robotsToBeConnected, position, u, robot)
    }
    // norm constraint on the control input ux^2 + uy^2 <= maxSpeed^2
    model.maxSpeedCBF(u, robot)
    // GO-TO-TARGET CLF 2(p - p_g)^T u <= -c || p - p_g ||^2 + \delta
    model.goToTargetCLF(target, position, u, delta)
    // objective: min ||u - u_nom||^2 + phi * delta^2
    model.minimizeNominal(target, robot, u, delta)
    // solve
    model.optimize()
    val uOptX = u[0].get(GRB.DoubleAttr.X)
    val uOptY = u[1].get(GRB.DoubleAttr.X)
    println("Optimal control for ${robot.id}: u = ($uOptX, $uOptY)")
    model.dispose()
    env.dispose()
    return SpeedControl2D(uOptX, uOptY)
}
