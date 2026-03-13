package it.unibo.collektive.admm

import it.unibo.collektive.aggregate.api.Aggregate
import it.unibo.collektive.aggregate.api.sharing
import it.unibo.collektive.aggregate.toMap
import it.unibo.collektive.control.cbf.CBF
import it.unibo.collektive.control.clf.CLF
import it.unibo.collektive.model.Robot
import it.unibo.collektive.model.SpeedControl2D
import it.unibo.collektive.mathutils.minus
import it.unibo.collektive.mathutils.norm
import it.unibo.collektive.mathutils.plus
import it.unibo.collektive.solver.gurobi.QpSettings
import it.unibo.collektive.stdlib.spreading.gossipMax

/**
 * Main control loop that runs ADMM consensus iterations until the residuals
 * meet the configured [tolerance] or the [maxIter] limit is reached.
 */
fun Aggregate<Int>.controlLoop(
    robot: Robot,
    uNominal: DoubleArray,
    maxIter: Int,
    tolerance: Tolerance,
    deltaTime: Double,
    localCLF: List<CLF>,
    localCBFs: List<CBF> = emptyList(),
    pairwiseCBFs: List<CBF> = emptyList(),
): Pair<Boolean, SpeedControl2D> = evolving(0 to ControlAndDuals(robot.control, emptyMap())) { previousDuals ->
    val settings = QpSettings(deltaTime = deltaTime)
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
    val (rt, st) = residualUpdate(output, previousSuggested)
    val nextIter = previousDuals.first + 1
    val (shouldApply, iter) = when {
        (rt <= tolerance.primal && st <= tolerance.dual) || nextIter >= maxIter -> true to 0
        else -> false to nextIter
    }
    (iter to output).yielding { shouldApply to output.control }
}

/**
 * Executes one ADMM round: local update plus dual refresh for all neighbors.
 */
fun <ID : Comparable<ID>> Aggregate<ID>.coreADMM(
    robot: Robot,
    uNominal: DoubleArray,
    duals: Map<ID, DualParams>,
    settings: QpSettings,
    localCLF: List<CLF>,
    localCBFs: List<CBF> = emptyList(),
    pairwiseCBFs: List<CBF> = emptyList(),
): ControlAndDuals<ID> = sharing(robot) { controls ->
    val control: SpeedControl2D =
        executeLocalADMM(robot, uNominal, duals, settings, localCLF, localCBFs)
    val robotUpdated = robot.copy(control = control)
    val commons: Map<ID, DualParams> = controls.neighbors.toMap().mapValues { (id, neighbor) ->
        val incidentDuals = duals[id]?.incidentDuals ?: IncidentDuals()
        val (zi, zj) = executeCommonADMM(robotUpdated, neighbor, incidentDuals, settings, pairwiseCBFs)
        // local dual update
        val newIncidentDuals = IncidentDuals(
            incidentDuals.yi + control - zi, // y_ij^i,t+1 = y_ij^i,t + (u_i^t+1 - z_ij^i,t+1)
            incidentDuals.yj + neighbor.control - zj, // y_ij^j,t+1 = y_ij^j,t + (u_j^t+1 - z_ij^j,t+1)
        )
        DualParams(SuggestedControl(zi, zj), newIncidentDuals)
    }.filterNot { it.key == localId }
    robotUpdated.yielding { ControlAndDuals(control, commons) }
}

private fun executeCommonADMM(
    robotUpdated: Robot,
    other: Robot,
    incidentDuals: IncidentDuals,
    settings: QpSettings,
    pairwiseCBFs: List<CBF>,
): SuggestedControl = solvePairwiseQP(robotUpdated, other, incidentDuals, settings, pairwiseCBFs)

/**
 * Local QP wrapper computing the optimal control for obstacle avoidance and goal tracking.
 */
fun <ID : Comparable<ID>> executeLocalADMM(
    robot: Robot,
    uNominal: DoubleArray,
    duals: Map<ID, DualParams>,
    settings: QpSettings,
    localCLF: List<CLF>,
    localCBFs: List<CBF> = emptyList(),
): SpeedControl2D =
    solveLocalQP(robot, uNominal, duals, settings, localCLF, localCBFs)

private fun Aggregate<Int>.residualUpdate(
    output: ControlAndDuals<Int>,
    previousSuggested: Map<Int, SuggestedControl>,
): Residuals {
    val currentSuggested = output.duals.filterNot { it.key == localId }.mapValues { it.value.suggestedControl }
    val rijt: Double = currentSuggested.maxOfOrNull { (_, value) -> (output.control - value.zi).norm() } ?: 0.0
    val rt = gossipMax(rijt)
    val rho = 0.5
    val sit = currentSuggested.maxOfOrNull { (id, value) ->
        val prev = previousSuggested[id] ?: SuggestedControl()
        rho * (value.zi - prev.zi).norm()
    } ?: 0.0
    val st = gossipMax(sit)
    return Residuals(rt, st)
}
