package it.unibo.collektive.admm

import it.unibo.alchemist.collektive.device.CollektiveDevice
import it.unibo.alchemist.model.positions.Euclidean2DPosition
import it.unibo.collektive.aggregate.api.Aggregate
import it.unibo.collektive.aggregate.api.sharing
import it.unibo.collektive.aggregate.toMap
import it.unibo.collektive.alchemist.device.applyControl
import it.unibo.collektive.alchemist.device.sensors.TimeSensor
import it.unibo.collektive.control.cbf.CBF
import it.unibo.collektive.control.clf.CLF
import it.unibo.collektive.mathutils.minus
import it.unibo.collektive.mathutils.norm
import it.unibo.collektive.mathutils.plus
import it.unibo.collektive.model.Robot
import it.unibo.collektive.model.SpeedControl2D
import it.unibo.collektive.solver.gurobi.QpSettings
import it.unibo.collektive.stdlib.spreading.gossipMax
import it.unibo.collektive.stdlib.time.localDeltaTime
import kotlin.time.DurationUnit

/**
 * Sets up the ADMM control loop for a robot in an aggregate context.
 *
 * @param robot The robot to control.
 * @param uNominal The nominal control input.
 * @param localCLF List of local Control Lyapunov Functions.
 * @param localCBF List of local Control Barrier Functions.
 * @param pairwiseCBF List of pairwise Control Barrier Functions.
 * @param settings The QP solver settings.
 */
context(timeSensor: TimeSensor, device: CollektiveDevice<Euclidean2DPosition>)
fun Aggregate<Int>.setup(
    robot: Robot,
    uNominal: DoubleArray,
    localCLF: List<CLF>,
    localCBF: List<CBF>,
    pairwiseCBF: List<CBF>,
    settings: QpSettings,
) {
    val timeDistribution: Double = device["TimeDistribution"]
    val deltaTime: Double =
        localDeltaTime(timeSensor.getTimeAsInstant()).toDouble(DurationUnit.SECONDS)
            .takeIf { it > 0.0 } ?: (1.0 / timeDistribution)
    val settingsUpdated = settings.copy(deltaTime = deltaTime)
    val maxIter: Int = device["MaxIterations"]
    val res = controlLoop(
        robot = robot,
        uNominal = uNominal,
        maxIter = maxIter,
        settings = settingsUpdated,
        localCLF = localCLF,
        localCBFs = localCBF,
        pairwiseCBFs = pairwiseCBF,
    )
    if (res.first) robot.applyControl(res.second, settings.deltaTime) // stop if residuals < threshold
}

/**
 * Main control loop that runs ADMM consensus iterations until the residuals
 * meet the configured [tolerance] or the [maxIter] limit is reached.
 *
 * @param robot The robot to control.
 * @param uNominal The nominal control input.
 * @param maxIter Maximum number of ADMM iterations.
 * @param settings The QP solver settings.
 * @param localCLF List of local Control Lyapunov Functions.
 * @param localCBFs List of local Control Barrier Functions.
 * @param pairwiseCBFs List of pairwise Control Barrier Functions.
 * @return Pair of (shouldApplyControl, computedControl).
 */
fun Aggregate<Int>.controlLoop(
    robot: Robot,
    uNominal: DoubleArray,
    maxIter: Int,
    settings: QpSettings,
    localCLF: List<CLF>,
    localCBFs: List<CBF> = emptyList(),
    pairwiseCBFs: List<CBF> = emptyList(),
): Pair<Boolean, SpeedControl2D> = evolving(0 to ControlAndDuals(robot.control, emptyMap())) { previousDuals ->
    val output: ControlAndDuals<Int> = coreADMM(
        robot.copy(control = previousDuals.second.control),
        uNominal,
        previousDuals.second.duals,
        settings,
        localCLF,
        localCBFs,
        pairwiseCBFs,
    )
    val previousSuggested: Map<Int, SuggestedControl> = previousDuals.second.duals.toMap().mapValues {
        it.value.suggestedControl
    }
    val (primalResidual, dualResidual) = residualUpdate(settings, output, previousSuggested)
    val nextIter = previousDuals.first + 1
    val (shouldApply, iter) = when {
        (primalResidual <= settings.tolerance.primal && dualResidual <= settings.tolerance.dual) ||
            nextIter >= maxIter -> true to 0
        else -> false to nextIter
    }
    (iter to output).yielding { shouldApply to output.control }
}

/**
 * Executes one ADMM round: local update plus dual refresh for all neighbors.
 *
 * @param robot The robot to control.
 * @param uNominal The nominal control input.
 * @param duals The current dual variables for each neighbor.
 * @param settings The QP solver settings.
 * @param localCLF List of local Control Lyapunov Functions.
 * @param localCBFs List of local Control Barrier Functions.
 * @param pairwiseCBFs List of pairwise Control Barrier Functions.
 * @return The updated control and duals for this round.
 */
fun <ID : Comparable<ID>> Aggregate<ID>.coreADMM(
    robot: Robot,
    uNominal: DoubleArray,
    duals: Map<ID, DualParams>,
    settings: QpSettings,
    localCLF: List<CLF>,
    localCBFs: List<CBF> = emptyList(),
    pairwiseCBFs: List<CBF> = emptyList(),
): ControlAndDuals<ID> {
    val control: SpeedControl2D = solveLocalQP(robot, uNominal, duals, settings, localCLF, localCBFs)
    val robotUpdated = robot.copy(control = control)
    return sharing(robotUpdated) { controls ->
        val commons: Map<ID, DualParams> = controls.neighbors.toMap().mapValues { (id, neighbor) ->
            val incidentDuals = duals[id]?.incidentDuals ?: IncidentDuals()
            val (zi, zj) = solvePairwiseQP(robotUpdated, neighbor, incidentDuals, settings, pairwiseCBFs)
            val newIncidentDuals = IncidentDuals(
                // local dual update
                incidentDuals.yi + control - zi, // y_ij^i,t+1 = y_ij^i,t + (u_i^t+1 - z_ij^i,t+1)
                incidentDuals.yj + neighbor.control - zj, // y_ij^j,t+1 = y_ij^j,t + (u_j^t+1 - z_ij^j,t+1)
            )
            DualParams(SuggestedControl(zi, zj), newIncidentDuals)
        }.filterNot { it.key == localId }
        robotUpdated.yielding { ControlAndDuals(control, commons) }
    }
}

/**
 * Computes the primal and dual residuals for the current ADMM iteration.
 *
 * @param settings The QP solver settings.
 * @param output The current control and duals.
 * @param previousSuggested The previous suggested controls for each neighbor.
 * @return The computed residuals (primal, dual).
 */
private fun Aggregate<Int>.residualUpdate(
    settings: QpSettings,
    output: ControlAndDuals<Int>,
    previousSuggested: Map<Int, SuggestedControl>,
): Residuals {
    val currentSuggested = output.duals.filterNot { it.key == localId }.mapValues { it.value.suggestedControl }
    val primalResidualLocal: Double =
        currentSuggested.maxOfOrNull { (_, value) -> (output.control - value.zi).norm() } ?: 0.0
    val primalResidualGlobal = gossipMax(primalResidualLocal)
    val dualResidualLocal = currentSuggested.maxOfOrNull { (id, value) ->
        val prev = previousSuggested[id] ?: SuggestedControl()
        settings.rhoResidual * (value.zi - prev.zi).norm()
    } ?: 0.0
    val dualResidualGlobal = gossipMax(dualResidualLocal)
    return Residuals(primalResidualGlobal, dualResidualGlobal)
}
