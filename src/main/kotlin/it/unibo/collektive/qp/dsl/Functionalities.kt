package it.unibo.collektive.qp.dsl

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBEnv
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.qp.carol.DualParams
import it.unibo.collektive.qp.carol.IncidentDuals
import it.unibo.collektive.qp.carol.SuggestedControl
import it.unibo.collektive.qp.controlFunctions.addCollisionAvoidanceCBF
import it.unibo.collektive.qp.controlFunctions.addCommunicationRangeCBF
import it.unibo.collektive.qp.controlFunctions.addObstacleAvoidanceCBF
import it.unibo.collektive.qp.controlFunctions.goToTargetCLF
import it.unibo.collektive.qp.controlFunctions.maxSpeedCBF
import it.unibo.collektive.qp.controlFunctions.minimizeADMMCommonQP
import it.unibo.collektive.qp.controlFunctions.minimizeADMMLocalQP
import it.unibo.collektive.qp.utils.Obstacle
import it.unibo.collektive.qp.utils.Robot
import it.unibo.collektive.qp.utils.SpeedControl2D
import it.unibo.collektive.qp.utils.Target
import it.unibo.collektive.qp.utils.toDoubleArray

// Shared setup for local ADMM QPs; guarantees model lifecycle is handled consistently.
private fun <T> withLocalAdmmModel(
    robot: Robot,
    target: Target,
    obstacle: Obstacle?,
    block: (model: GRBModel, u: GRBVector, delta: GRBVar, position: DoubleArray) -> T,
): T {
    setLicense() // Tell Gurobi exactly where the license is
    val env = GRBEnv(true).also { it.start() }
    val model = GRBModel(env).also { it.setupLogger() } // create an optimization model inside the environment
    val u: GRBVector = model.addVecVar(
        dimension = robot.position.dimension,
        lowerBound = -robot.maxSpeed,
        upperBound = robot.maxSpeed,
        name = "u",
    )
    val delta = model.addVar(0.0, GRB.INFINITY, 0.0, GRB.CONTINUOUS, "delta") // slack variable
    val position: DoubleArray = robot.toDoubleArray()
    if (obstacle != null) {
        model.addObstacleAvoidanceCBF(position, obstacle, u)
    }
    model.maxSpeedCBF(u, robot)
    model.goToTargetCLF(target, position, u, delta)
    return try {
        block(model, u, delta, position)
    } finally {
        model.dispose()
        env.dispose()
    }
}

/**
 * Solves the local QP that moves the robot toward the target while
 * avoiding a single obstacle and enforcing ADMM consensus.
 *
 * @return optimal control and slack value for the local agent.
 */
fun avoidObstacleGoToTarget(
    robot: Robot,
    target: Target,
    obstacle: Obstacle? = null,
    average: DoubleArray,
    cardinality: Int,
): Pair<SpeedControl2D, Double> = withLocalAdmmModel(robot, target, obstacle) { model, u, delta, _ ->
    model.minimizeADMMLocalQP(u, delta, robot, target, average, cardinality)
}

/**
 * Solves the local QP that moves the robot toward the target while
 * avoiding a single obstacle and enforcing ADMM consensus.
 *
 * @return optimal control and slack value for the local agent.
 */
fun <ID : Comparable<ID>> avoidObstacleGoToTarget(
    robot: Robot,
    target: Target,
    obstacle: Obstacle?,
    duals: Map<ID, DualParams>,
): Pair<SpeedControl2D, Double> = withLocalAdmmModel(robot, target, obstacle) { model, u, delta, _ ->
    model.minimizeADMMLocalQP(u, delta, robot, target, duals)
}

/**
 * Solves the pairwise QP that enforces robot avoidance (and optionally communication range) for an edge.
 */
fun robotAvoidanceAndCommunicationRangeCBF(
    robot: Robot,
    other: Robot,
    range: Double? = null,
    incidentDuals: IncidentDuals,
): SuggestedControl {
    setLicense() // Tell Gurobi exactly where the license is
    val env = GRBEnv(true).also { it.start() }
    val model = GRBModel(env).also { it.setupLogger() } // create an optimization model inside the environment
    val zi: GRBVector = model.addVecVar(
        dimension = robot.position.dimension,
        lowerBound = -robot.maxSpeed,
        upperBound = robot.maxSpeed,
        name = "z_ij^i",
    )
    val zj: GRBVector = model.addVecVar(
        dimension = other.position.dimension,
        lowerBound = -other.maxSpeed,
        upperBound = other.maxSpeed,
        name = "z_ij^j",
    )
    // COLLISION AVOIDANCE 2(p1 - p2)^T (u1 - u2) + \gamma [ ||p1-p2||^2 - dmin^2 ] >= 0
    model.addCollisionAvoidanceCBF(zi, zj, robot, other)
    // COMM DISTANCE -2(p1 - p2)^T (u1 -u2) + \gamma [ R^2 - ||p1 - p2||^2 ] >= 0
    if (range != null) {
        model.addCommunicationRangeCBF(zi, zj, robot, other, range)
    }
    // rho / 2 * ( ||z_ij,i - (ui + y_ij,i)||^2 + || z_ij,j - (uj + y_ij,j)||^2 )
    val result = model.minimizeADMMCommonQP(zi, zj, robot, other, incidentDuals)
    model.dispose()
    env.dispose()
    return result
}
