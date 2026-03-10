package it.unibo.collektive.qp.dsl

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.qp.carol.DualParams
import it.unibo.collektive.qp.carol.IncidentDuals
import it.unibo.collektive.qp.carol.SuggestedControl
import it.unibo.collektive.solver.gurobi.QpSettings
import it.unibo.collektive.qp.controlFunctions.CbfContext
import it.unibo.collektive.qp.controlFunctions.applyLocalCbfs
import it.unibo.collektive.qp.controlFunctions.applyPairwiseCbfs
import it.unibo.collektive.qp.controlFunctions.goToTargetCLF
import it.unibo.collektive.qp.controlFunctions.maxSpeedCBF
import it.unibo.collektive.qp.controlFunctions.minimizeADMMCommonQP
import it.unibo.collektive.qp.controlFunctions.minimizeADMMLocalQP
import it.unibo.collektive.model.Obstacle
import it.unibo.collektive.model.Robot
import it.unibo.collektive.model.SpeedControl2D
import it.unibo.collektive.model.Target
import it.unibo.collektive.model.toDoubleArray
import it.unibo.collektive.solver.gurobi.ConstraintNames
import it.unibo.collektive.solver.gurobi.GRBVector
import it.unibo.collektive.solver.gurobi.addVecVar
import it.unibo.collektive.solver.gurobi.withModel

// Shared setup for local ADMM QPs; guarantees model lifecycle is handled consistently.
private fun <T> withLocalADMMModel(
    robot: Robot,
    target: Target,
    obstacle: Obstacle?,
    settings: QpSettings = QpSettings(),
    block: (
        model: GRBModel,
        u: GRBVector,
        delta: GRBVar,
        position: DoubleArray,
        target: Target,
        obstacle: Obstacle?,
    ) -> T,
): T = withModel(settings) { model ->
    val u: GRBVector = model.addVecVar(
        dimension = robot.position.dimension,
        lowerBound = -robot.maxSpeed,
        upperBound = robot.maxSpeed,
        name = "u",
    )
    val delta = model.addVar(0.0, GRB.INFINITY, 0.0, GRB.CONTINUOUS, ConstraintNames.slack("local"))
    val position: DoubleArray = robot.toDoubleArray()
    block(model, u, delta, position, target, obstacle)
}

private fun runLocalADMM(
    model: GRBModel,
    u: GRBVector,
    delta: GRBVar,
    robot: Robot,
    target: Target,
    obstacle: Obstacle?,
    settings: QpSettings,
    objective: () -> Pair<SpeedControl2D, Double>,
): Pair<SpeedControl2D, Double> {
    applyLocalCbfs(model, u, CbfContext(self = robot, obstacle = obstacle, settings = settings))
    model.maxSpeedCBF(u, robot)
    model.goToTargetCLF(target, robot.toDoubleArray(), u, delta, settings)
    return objective()
}

// /**
// * Solves the local QP that moves the robot toward the target while
// * avoiding a single obstacle and enforcing ADMM consensus.
// *
// * @return optimal control and slack value for the local agent.
// */
// fun avoidObstacleGoToTarget(
//    robot: Robot,
//    target: Target,
//    obstacle: Obstacle? = null,
//    average: DoubleArray,
//    cardinality: Int,
//    settings: QpSettings = QpSettings(),
// ): Pair<SpeedControl2D, Double> =
//    withLocalADMMModel(robot, target, obstacle, settings) { model, u, delta, _, tgt, obs ->
//        runLocalADMM(model, u, delta, robot, tgt, obs, settings) {
//            model.minimizeADMMLocalQP(u, delta, robot, tgt, average, cardinality, settings)
//        }
//    }

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
    settings: QpSettings = QpSettings(),
): Pair<SpeedControl2D, Double> =
    withLocalADMMModel(robot, target, obstacle, settings) { model, u, delta, _, tgt, obs ->
        runLocalADMM(model, u, delta, robot, tgt, obs, settings) {
            model.minimizeADMMLocalQP(u, delta, robot, tgt, duals, settings)
        }
    }

/**
 * Solves the pairwise QP that enforces robot avoidance (and optionally communication range) for an edge.
 */
fun robotAvoidanceAndCommunicationRangeCBF(
    robot: Robot,
    other: Robot,
    range: Double? = null,
    incidentDuals: IncidentDuals,
    settings: QpSettings = QpSettings(),
): SuggestedControl = withModel(settings) { model ->
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
    applyPairwiseCbfs(
        model,
        zi,
        zj,
        CbfContext(self = robot, other = other, communicationRange = range, settings = settings),
    )
    model.minimizeADMMCommonQP(zi, zj, robot, other, incidentDuals, settings)
}
