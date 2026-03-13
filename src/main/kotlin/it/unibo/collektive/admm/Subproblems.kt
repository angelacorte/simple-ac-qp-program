package it.unibo.collektive.admm

import com.gurobi.gurobi.GRBModel
import it.unibo.collektive.control.ControlFunctionContext
import it.unibo.collektive.control.cbf.CBF
import it.unibo.collektive.control.cbf.CBFRegistry
import it.unibo.collektive.control.clf.CLF
import it.unibo.collektive.control.clf.CLFRegistry.applyLocalCLFs
import it.unibo.collektive.model.Robot
import it.unibo.collektive.model.SpeedControl2D
import it.unibo.collektive.solver.gurobi.GRBVector
import it.unibo.collektive.solver.gurobi.QpSettings
import it.unibo.collektive.solver.gurobi.addVecVar
import it.unibo.collektive.solver.gurobi.withModel

/**
 * Solves the local QP that minimizes deviation from a nominal control while
 * enforcing local safety constraints and ADMM consensus.
 *
 * @return optimal control for the local agent.
 */
fun <ID: Comparable<ID>> solveLocalQP(
    robot: Robot,
    uNominal: DoubleArray,
    duals: Map<ID, DualParams>,
    settings: QpSettings = QpSettings(),
    localCLFs: List<CLF>,
    localCBFs: List<CBF> = emptyList(),
): SpeedControl2D = withModel(settings) { model ->
    val u: GRBVector = model.addVecVar(robot.position.dimension, -robot.maxSpeed, robot.maxSpeed, "u")
    val context = ControlFunctionContext(self = robot, settings = settings)
    applyLocalCLFs(model, u, context, localCLFs)
    CBFRegistry.applyLocalCBFs(model, u, context, localCBFs)
    val allControlFunctions = localCLFs + localCBFs
    model.minimizeADMMLocalQP(u, uNominal, robot, duals, allControlFunctions, context)
}

/**
 * Solves the pairwise QP applying the given [pairwiseCBFs] between [robot] and [other].
 */
fun solvePairwiseQP(
    robot: Robot,
    other: Robot,
    incidentDuals: IncidentDuals,
    settings: QpSettings = QpSettings(),
    pairwiseCBFs: List<CBF> = emptyList(),
): SuggestedControl = withModel(settings) { model ->
    val zi: GRBVector = model.addVecVar(robot.position.dimension, -robot.maxSpeed, robot.maxSpeed, "z_ij^i")
    val zj: GRBVector = model.addVecVar(other.position.dimension, -other.maxSpeed, other.maxSpeed, "z_ij^j")
    val context = ControlFunctionContext(self = robot, otherRobot = other, settings = settings)
    CBFRegistry.applyPairwiseCBFs(model, zi, zj, context, pairwiseCBFs)
    model.minimizeADMMCommonQP(zi, zj, robot, other, incidentDuals, pairwiseCBFs, context)
}
