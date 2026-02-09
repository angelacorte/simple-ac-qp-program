package it.unibo.collektive.qp.dsl

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBEnv
import com.gurobi.gurobi.GRBModel
import it.unibo.collektive.qp.controlFunctions.addCollisionAvoidanceCBF
import it.unibo.collektive.qp.controlFunctions.addCommunicationRangeCBF
import it.unibo.collektive.qp.controlFunctions.maybeWrongAddCommunicationRangeCBF
import it.unibo.collektive.qp.controlFunctions.addObstacleAvoidanceCBF
import it.unibo.collektive.qp.controlFunctions.maybeWrongAddRobotAvoidanceCBF
import it.unibo.collektive.qp.controlFunctions.goToTargetCLF
import it.unibo.collektive.qp.controlFunctions.maxSpeedCBF
import it.unibo.collektive.qp.controlFunctions.minimizeADMMCommonQP
import it.unibo.collektive.qp.controlFunctions.minimizeADMMLocalQP
import it.unibo.collektive.qp.controlFunctions.minimizeNominal
import it.unibo.collektive.qp.utils.Obstacle
import it.unibo.collektive.qp.utils.Robot
import it.unibo.collektive.qp.utils.SpeedControl2D
import it.unibo.collektive.qp.utils.Target
import it.unibo.collektive.qp.utils.setLicense
import it.unibo.collektive.qp.utils.toDoubleArray

fun <ID: Comparable<ID>> avoidObstacleGoToTarget(robot: Robot<ID>, target: Target, obstacle: Obstacle, parameters: Parameters<ID>): Pair<SpeedControl2D, Double> {
    setLicense() // Tell Gurobi exactly where the license is
    val env = GRBEnv(true).also { it.start() } // create environment in manual mode (because of license file specification)
    val model = GRBModel(env).also { it.setupLogger() } // create an optimization model inside the environment
    val u: GRBVector = model.addVecVar(dimension = robot.position.dimension, lowerBound = -robot.maxSpeed, upperBound = robot.maxSpeed, name = "u")
    val delta = model.addVar(0.0, GRB.INFINITY, 0.0, GRB.CONTINUOUS, "delta") // slack variable
    val position: DoubleArray = robot.toDoubleArray()
    // (OBSTACLE AVOIDANCE) linear CBF 2(p - p_o)^T u >= - \gamma [ ||p - p_o||^2 - (r_o + d_o)^2 ]
    model.addObstacleAvoidanceCBF(position, obstacle, u)
    // norm constraint on the control input ux^2 + uy^2 <= maxSpeed^2
    model.maxSpeedCBF(u, robot)
    // GO-TO-TARGET CLF 2(p - p_g)^T u <= -c || p - p_g ||^2 + \delta
    model.goToTargetCLF(target, position, u, delta)
    // || u - u_nom||^2 + rho_s * delta^2 + rho_a / 2 * SUM ||i - z_ij,i + y_ij,i||^2
    val result = model.minimizeADMMLocalQP(u, delta, robot, target, parameters.edges)
    model.dispose()
    env.dispose()
    return result
}

fun <ID: Comparable<ID>> robotAvoidanceAndCommunicationRangeCBF(robot: Robot<ID>, other: Robot<ID>, range: Double, edge: Coupled<ID>): SuggestedControl<ID> {
    setLicense() // Tell Gurobi exactly where the license is
    val env = GRBEnv(true).also { it.start() } // create environment in manual mode (because of license file specification)
    val model = GRBModel(env).also { it.setupLogger() } // create an optimization model inside the environment
    val zi: GRBVector = model.addVecVar(dimension = robot.position.dimension, lowerBound = -robot.maxSpeed, upperBound = robot.maxSpeed, name = "z_ij^i")
    val zj: GRBVector = model.addVecVar(dimension = other.position.dimension, lowerBound = -other.maxSpeed, upperBound = other.maxSpeed, name = "z_ij^j")
    // COLLISION AVOIDANCE 2(p1 - p2)^T (u1 - u2) + \gamma [ ||p1-p2||^2 - dmin^2 ] >= 0
    model.addCollisionAvoidanceCBF(zi, zj, robot, other)
    // COMM DISTANCE -2(p1 - p2)^T (u1 -u2) + \gamma [ R^2 - ||p1 - p2||^2 ] >= 0
    model.addCommunicationRangeCBF(zi, zj, robot, other, range)
    // rho / 2 * ( ||z_ij,i - (ui + y_ij,i)||^2 + || z_ij,j - (uj + y_ij,j)||^2 )
    val result = model.minimizeADMMCommonQP(zi, zj, robot, other, edge)
    model.dispose()
    env.dispose()
    return result
}

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
fun <ID: Comparable<ID>> wrongRobotToTargetWithAvoidanceAndDistance(
    robot: Robot<ID>,
    target: Target,
    obstacle: Obstacle,
    robotsToAvoid: List<Robot<ID>> = emptyList(),
    robotsToBeConnected: List<Robot<ID>> = emptyList(),
    maxConnectionRange: Double = Double.MIN_VALUE,
): Pair<SpeedControl2D, Double> {
    setLicense() // Tell Gurobi exactly where the license is
    val env = GRBEnv(true) // create environment in manual mode (because of license file specification)
    env.start()
    val model = GRBModel(env) // create an optimization model inside the environment
    // decision variables
    // control input (velocity or displacement) bounds represent admissible control directions
    val u: GRBVector = model.addVecVar(dimension = robot.position.dimension, lowerBound = -robot.maxSpeed, upperBound = robot.maxSpeed, name = "u")
    val delta = model.addVar(0.0, GRB.INFINITY, 0.0, GRB.CONTINUOUS, "delta") // slack variable
    val position: DoubleArray = robot.toDoubleArray()
    // (OBSTACLE AVOIDANCE) linear CBF 2(p - p_o)^T u >= - \gamma [ ||p - p_o||^2 - (r_o + d_o)^2 ]
    model.addObstacleAvoidanceCBF(position, obstacle, u)
    //  (ROBOT AVOIDANCE) linear CBF 2(p1 - p2)^T (u1 - u2) + \gamma [ (p1-p2)^T(p1-p2) - dmin^2 ] >= 0
    // move u2 to the right
    // 2(p1 - p2)^T u1 >= -2(p1 - p2)^T u2 - \gamma [ (p1-p2)^T(p1-p2) - dmin^2 ]
    if (robotsToAvoid.isNotEmpty()) {
        model.maybeWrongAddRobotAvoidanceCBF(robotsToAvoid, robot, position, u)
    }
    // (COMMUNICATION DISTANCE) CBF -2(p1 - p2)^T (u1 -u2) + \gamma [ R^2 - ||p1 - p2||^2 ] >= 0
    if (robotsToBeConnected.isNotEmpty() && maxConnectionRange != Double.MIN_VALUE) {
        check(maxConnectionRange > 0) {
            "Please specify a valid connection distance, currently is $maxConnectionRange, which lead to infeasible problem"
        }
        model.maybeWrongAddCommunicationRangeCBF(maxConnectionRange, robotsToBeConnected, position, u, robot)
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
    val deltaOpt = u[2].get(GRB.DoubleAttr.X)
    println("Optimal control for ${robot.id}: u = ($uOptX, $uOptY)")
    model.dispose()
    env.dispose()
    return SpeedControl2D(uOptX, uOptY) to deltaOpt
}

fun <ID: Comparable<ID>> localBarrierAndGoToTarget(robot: Robot<ID>, obstacle: Obstacle, target: Target): Pair<SpeedControl2D, Double> {
    setLicense() // Tell Gurobi exactly where the license is
    val env = GRBEnv(true) // create environment in manual mode (because of license file specification)
    env.start()
    val model = GRBModel(env) // create an optimization model inside the environment
    // decision variables
    // control input (velocity or displacement) bounds represent admissible control directions
    val u: GRBVector = model.addVecVar(dimension = robot.position.dimension, lowerBound = -robot.maxSpeed, upperBound = robot.maxSpeed, name = "u")
    val delta = model.addVar(0.0, GRB.INFINITY, 0.0, GRB.CONTINUOUS, "delta") // slack variable
    val position: DoubleArray = robot.toDoubleArray()
    // (OBSTACLE AVOIDANCE) linear CBF 2(p - p_o)^T u >= - \gamma [ ||p - p_o||^2 - (r_o + d_o)^2 ]
    model.addObstacleAvoidanceCBF(position, obstacle, u)
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
    val deltaOpt = u[2].get(GRB.DoubleAttr.X)
    println("Optimal control for ${robot.id}: u = ($uOptX, $uOptY)")
    model.dispose()
    env.dispose()
    return SpeedControl2D(uOptX, uOptY) to deltaOpt
}
