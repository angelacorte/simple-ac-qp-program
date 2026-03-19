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
import kotlin.math.max
import kotlin.time.DurationUnit

/**
 * Aggregate entrypoint: runs the distributed ADMM control loop and applies the resulting velocity.
 */
context(timeSensor: TimeSensor, device: CollektiveDevice<Euclidean2DPosition>)
fun Aggregate<Int>.admmEntrypoint(
    robot: Robot,
    frequency: Double? = null,
    maxIterations: Int,
    uNominal: DoubleArray,
    localCLF: List<CLF>,
    localCBF: List<CBF>,
    pairwiseCBF: List<CBF>,
    settings: QpSettings,
) {
    val deltaTime: Double = localDeltaTime(timeSensor.getTimeAsInstant()).toDouble(DurationUnit.SECONDS)
        .takeIf { it > 0.0 } ?: (1.0 / (frequency ?: 1.0))
    val result = controlLoop(
        robot = robot,
        uNominal = uNominal,
        maxIter = maxIterations,
        settings = settings.copy(deltaTime = deltaTime),
        localCLF = localCLF,
        localCBFs = localCBF,
        pairwiseCBFs = pairwiseCBF,
    )
//    if(result.shouldApply) {
    robot.applyControl(result.control, deltaTime)
//    } else {
//        device["Velocity"] = zeroSpeed()
//    }
}

/**
 * Runs ADMM iterations until both primal and dual residuals fall below their tolerances,
 * or until [maxIter] iterations are exhausted.
 */
fun Aggregate<Int>.controlLoop(
    robot: Robot,
    uNominal: DoubleArray,
    maxIter: Int,
    settings: QpSettings,
    localCLF: List<CLF>,
    localCBFs: List<CBF> = emptyList(),
    pairwiseCBFs: List<CBF> = emptyList(),
): OutputControl = evolving(Infos(0, ControlAndDuals(robot.control, emptyMap()))) { previousDuals ->
    val output = coreADMM(
        robot = robot.copy(control = previousDuals.admmOutput.control),
        uNominal = uNominal,
        duals = previousDuals.admmOutput.duals,
        settings = settings,
        localCLF = localCLF,
        localCBFs = localCBFs,
        pairwiseCBFs = pairwiseCBFs,
    )
    val previousSuggested = previousDuals.admmOutput.duals
        .toMap().mapValues { it.value.suggestedControl }
    val (primalResidual, dualResidual) = residualUpdate(settings, output, previousSuggested)
    val nextIter = previousDuals.iteration + 1
    val (shouldApply, iter) = when {
        (primalResidual <= settings.tolerance.primal && dualResidual <= settings.tolerance.dual) ||
            nextIter >= maxIter -> true to 0
        else -> false to nextIter
    }
    val confidence = confidence(primalResidual, dualResidual, settings.tolerance)
    val scaledControl = SpeedControl2D(output.control.x * confidence, output.control.y * confidence)
    Infos(iter, output).yielding { OutputControl(shouldApply, scaledControl) }
//    Infos(iter, output).yielding { OutputControl(shouldApply, output.control) }
}

fun <ID : Comparable<ID>> Aggregate<ID>.coreADMM(
    robot: Robot,
    uNominal: DoubleArray,
    duals: Map<ID, DualParams>,
    settings: QpSettings,
    localCLF: List<CLF>,
    localCBFs: List<CBF> = emptyList(),
    pairwiseCBFs: List<CBF> = emptyList(),
): ControlAndDuals<ID> {
    val control: SpeedControl2D = solveLocalQP(localId, robot, uNominal, duals, settings, localCLF, localCBFs)
    val robotUpdated = robot.copy(control = control)
    return sharing(robotUpdated) { controls ->
        val commons: Map<ID, DualParams> = controls.neighbors.toMap()
            .filterNot { it.key == localId }
            .mapValues { (neighborId, neighbor) ->
                val incidentDuals = duals[neighborId]?.incidentDuals ?: IncidentDuals()
                val (zi, zj) = solvePairwiseQP(
                    localId,
                    neighborId,
                    robotUpdated,
                    neighbor,
                    incidentDuals,
                    settings,
                    pairwiseCBFs,
                )
                val newIncidentDuals = IncidentDuals(
                    incidentDuals.yi + control - zi, // y_ij^i,t+1
                    incidentDuals.yj + neighbor.control - zj, // y_ij^j,t+1
                )
                DualParams(SuggestedControl(zi, zj), newIncidentDuals)
            }
        robotUpdated.yielding { ControlAndDuals(control, commons) }
    }
}

private fun Aggregate<Int>.residualUpdate(
    settings: QpSettings,
    output: ControlAndDuals<Int>,
    previousSuggested: Map<Int, SuggestedControl>,
): Residuals {
    val currentSuggested = output.duals.filterNot { it.key == localId }.mapValues { it.value.suggestedControl }
    val primalResidualLocal = currentSuggested.maxOfOrNull { (_, v) -> (output.control - v.zi).norm() } ?: 0.0
    val primalResidualGlobal = gossipMax(primalResidualLocal)
    val dualResidualLocal = currentSuggested.maxOfOrNull { (id, v) ->
        val prev = previousSuggested[id] ?: SuggestedControl()
        settings.rhoResidual * (v.zi - prev.zi).norm()
    } ?: 0.0
    val dualResidualGlobal = gossipMax(dualResidualLocal)
    return Residuals(primalResidualGlobal, dualResidualGlobal)
}

private fun confidence(primalResidual: Double, dualResidual: Double, tolerance: Tolerance): Double {
    val error = max(primalResidual / tolerance.primal, dualResidual / tolerance.dual)
    return if (error <= 1.0) 1.0 else 1.0 / error
}
