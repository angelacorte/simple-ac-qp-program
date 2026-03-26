package it.unibo.collektive.admm

import it.unibo.alchemist.collektive.device.CollektiveDevice
import it.unibo.alchemist.model.positions.Euclidean2DPosition
import it.unibo.collektive.aggregate.Field
import it.unibo.collektive.aggregate.api.Aggregate
import it.unibo.collektive.aggregate.api.exchange
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
import it.unibo.collektive.model.zeroSpeed
import it.unibo.collektive.solver.Solver
import it.unibo.collektive.solver.gurobi.QpSettings
import it.unibo.collektive.stdlib.collapse.fold
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
    maxIterations: Int,
    uNominal: DoubleArray,
    solver: Solver,
    localCLF: List<CLF>,
    localCBF: List<CBF>,
    pairwiseCBF: List<CBF>,
) {
    val deltaTime: Double = localDeltaTime(timeSensor.getTimeAsInstant()).toDouble(DurationUnit.SECONDS)
        .takeIf { it > 0.0 } ?: (1.0 / (device["TimeDistribution"] as Double? ?: 1.0))
    val result = controlLoop(robot, uNominal, maxIterations, deltaTime, solver, localCLF, localCBF, pairwiseCBF)
    if (result.shouldApply) {
        robot.applyControl(result.control, deltaTime)
    } else {
        device["Velocity"] = zeroSpeed()
    }
}

fun Aggregate<Int>.controlLoop(
    robot: Robot,
    uNominal: DoubleArray,
    maxIter: Int,
    deltaTime: Double,
    solver: Solver,
    localCLF: List<CLF>,
    localCBF: List<CBF>,
    pairwiseCBF: List<CBF>,
): OutputControl = evolving(Infos(0, ControlAndDuals(robot.control, emptyMap()))) { previousDuals ->
    val output = coreADMM(
        robot.copy(control = previousDuals.admmOutput.control),
        uNominal,
        previousDuals.admmOutput.duals,
        deltaTime,
        solver,
        localCLF,
        localCBF,
        pairwiseCBF,
    )
    val previousSuggested = previousDuals.admmOutput.duals.toMap().mapValues { it.value.suggestedControl }
    val (primalResidual, dualResidual) = residualUpdate(solver.settings, output, previousSuggested)
    val nextIter = previousDuals.iteration + 1
    val (shouldApply, iter) = when {
        (primalResidual <= solver.settings.tolerance.primal && dualResidual <= solver.settings.tolerance.dual) ||
            nextIter >= maxIter -> true to 0
        else -> false to nextIter
    }
    val confidence = confidence(primalResidual, dualResidual, solver.settings.tolerance)
    val scaledControl = SpeedControl2D(output.control.x * confidence, output.control.y * confidence)
//    Infos(iter, output).yielding { OutputControl(shouldApply, scaledControl) }
    Infos(iter, output).yielding { OutputControl(shouldApply, output.control) }
}

fun <ID : Comparable<ID>> Aggregate<ID>.coreADMM(
    robot: Robot,
    uNominal: DoubleArray,
    duals: Map<ID, DualParams>,
    deltaTime: Double,
    solver: Solver,
    localCLF: List<CLF>,
    localCBF: List<CBF>,
    pairwiseCBF: List<CBF>,
): ControlAndDuals<ID> {
    if (!solver.isLocalModelAvailable) solver.setupLocalModel(robot, localCLF, localCBF)
    val control: SpeedControl2D = solver.updateAndSolveLocal(robot, uNominal, duals, deltaTime)
    val robotUpdated = robot.copy(control = control)
    return sharing(robotUpdated) { controls ->
        val commons: Map<ID, DualParams> = controls.neighbors.toMap()
            .filterNot { it.key == localId }
            .mapValues { (neighborId, neighbor) ->
                val incidentDuals = duals[neighborId]?.incidentDuals ?: IncidentDuals()
                if (!solver.isPairwiseModelAvailable) solver.setupPairwiseModel(robot, neighbor, pairwiseCBF)
                val (zi, zj) = solver.updateAndSolvePairwise(robotUpdated, neighbor, incidentDuals, deltaTime)
                val newIncidentDuals = IncidentDuals(
                    incidentDuals.yi + control - zi, // y_ij^i,t+1
                    incidentDuals.yj + neighbor.control - zj, // y_ij^j,t+1
                )
                DualParams(SuggestedControl(zi, zj), newIncidentDuals)
            }
        robotUpdated.yielding { ControlAndDuals(control, commons) }
    }
}

fun <ID : Comparable<ID>> Aggregate<ID>.coreADMMWithEdges(
    robot: Robot,
    uNominal: DoubleArray,
    duals: Map<ID, DualParams>,
    deltaTime: Double,
    solver: Solver,
    localCLF: List<CLF>,
    localCBF: List<CBF>,
    pairwiseCBF: List<CBF>,
): ControlAndDuals<ID> {
    if (!solver.isLocalModelAvailable) solver.setupLocalModel(robot, localCLF, localCBF)
    val control: SpeedControl2D = solver.updateAndSolveLocal(robot, uNominal, duals, deltaTime)
    val robotUpdated = robot.copy(control = control)
    val result = exchange(robotUpdated to DualParams()) { controls ->
        controls.map { (neighborID, neighborData) ->
            val neighborInfo = neighborData.first
            val neighborDuals = neighborData.second
            val incidentDuals = duals[neighborID]?.incidentDuals ?: IncidentDuals()
            when {
                isOwner(neighborID) -> {
                    if (!solver.isPairwiseModelAvailable) solver.setupPairwiseModel(robot, neighborInfo, pairwiseCBF)
                    val (zi, zj) = solver.updateAndSolvePairwise(robotUpdated, neighborInfo, incidentDuals, deltaTime)
                    val newIncidentDuals = IncidentDuals(
                        incidentDuals.yi + control - zi, // y_ij^i,t+1
                        incidentDuals.yj + neighborInfo.control - zj, // y_ij^j,t+1
                    )
                    neighborInfo to DualParams(SuggestedControl(zi, zj), newIncidentDuals)
                }
                else -> robotUpdated to neighborDuals
            }
        }
    }
    val other: ControlAndDuals<ID> = result.map { (id, data) ->
        ControlAndDuals(data.first.control, mapOf(id to data.second))
    }.neighbors.fold(ControlAndDuals(result.local.value.first.control)) { acc, next ->
        ControlAndDuals(acc.control, acc.duals + next.value.duals)
    }
    return other
}

fun <ID: Comparable<ID>> Aggregate<ID>.isOwner(otherID: ID): Boolean = localId != otherID && localId >= otherID

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
