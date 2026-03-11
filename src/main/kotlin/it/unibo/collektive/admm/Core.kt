@file:Suppress("UnusedPrivateMember")

package it.unibo.collektive.admm

import it.unibo.collektive.aggregate.api.Aggregate
import it.unibo.collektive.aggregate.api.neighboring
import it.unibo.collektive.aggregate.api.sharing
import it.unibo.collektive.aggregate.toMap
import it.unibo.collektive.aggregate.values
import it.unibo.collektive.model.Obstacle
import it.unibo.collektive.model.Robot
import it.unibo.collektive.model.SpeedControl2D
import it.unibo.collektive.model.Target
import it.unibo.collektive.model.minus
import it.unibo.collektive.model.norm
import it.unibo.collektive.model.plus
import it.unibo.collektive.model.zeroSpeed
import it.unibo.collektive.stdlib.collapse.max
import it.unibo.collektive.stdlib.spreading.gossipMax

/**
 * TODO.
 */
fun Aggregate<Int>.controlLoop(
    robot: Robot,
    target: Target,
    obstacle: Obstacle?,
    communicationDistance: Double?,
    maxIter: Int,
    tolerance: Tolerance,
): Pair<Boolean, SpeedControl2D> = evolving(0 to ControlAndDuals(robot.control, emptyMap())) { previousDuals ->
    val output: ControlAndDuals<Int> = coreADMM(
        robot.copy(control = previousDuals.second.control),
        target,
        obstacle,
        communicationDistance,
        previousDuals.second.duals,
    ) // local update already done
    val previousSuggested: Map<Int, SuggestedControl> = previousDuals.second.duals.toMap().mapValues {
        it.value.suggestedControl
    }
    val (rt, st) = residualUpdateNoNbr(output, previousSuggested)
//    val (primalResidual, dualResidual) = residualUpdate(output, previousSuggested)
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
    target: Target,
    obstacle: Obstacle?,
    communicationDistance: Double?,
    duals: Map<ID, DualParams>,
): ControlAndDuals<ID> = sharing(robot) { controls ->
    val control: SpeedControl2D =
        executeLocalADMM(robot, target, obstacle, duals)
    val robotUpdated = robot.copy(control = control)
    val commons: Map<ID, DualParams> = controls.neighbors.toMap().mapValues { (id, neighbor) ->
        val incidentDuals = duals[id]?.incidentDuals ?: IncidentDuals()
        val (zi, zj) = executeCommonADMM(robotUpdated, neighbor, communicationDistance, incidentDuals)
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
    communicationDistance: Double?,
    incidentDuals: IncidentDuals,
): SuggestedControl = robotAvoidanceAndCommunicationRangeCBF(robotUpdated, other, communicationDistance, incidentDuals)

// /**
// * Local QP wrapper computing the optimal control for obstacle avoidance and goal tracking.
// */
// fun executeLocalADMM(
//    robot: Robot,
//    target: Target,
//    obstacle: Obstacle?,
//    avg: DoubleArray,
//    cardinality: Int,
// ): SpeedControl2D {
//    val (uWanted, deltaNom) = avoidObstacleGoToTarget(robot, target, obstacle, avg, cardinality)
//    return uWanted
// }

/**
 * Local QP wrapper computing the optimal control for obstacle avoidance and goal tracking.
 */
fun <ID : Comparable<ID>> executeLocalADMM(
    robot: Robot,
    target: Target,
    obstacle: Obstacle?,
    duals: Map<ID, DualParams>,
): SpeedControl2D {
    val (uWanted, _) = avoidObstacleGoToTarget(robot, target, obstacle, duals)
    return uWanted
}

private fun Aggregate<Int>.residualUpdateNoNbr(
    output: ControlAndDuals<Int>,
    previousSuggested: Map<Int, SuggestedControl>,
): Residuals {
    val currentSuggested = output.duals.filterNot { it.key == localId }.mapValues { it.value.suggestedControl }
    // r_ij^t = max ||ui - zij,i||
    val rijt: Double = currentSuggested.maxOfOrNull { (_, value) -> (output.control - value.zi).norm() } ?: 0.0
    // r^t = max ri^t
    val rt = gossipMax(rijt)
    val rho = 0.5
    // sit = \rhoa max ||zij^it - zij^it-1||
    val sit = currentSuggested.maxOfOrNull { (id, value) ->
        val prev = previousSuggested[id] ?: SuggestedControl()
        rho * (value.zi - prev.zi).norm()
    } ?: 0.0
    val st = gossipMax(sit)
    return Residuals(rt, st)
}

@Suppress("UnusedPrivateFunction")
private fun Aggregate<Int>.residualUpdate(
    output: ControlAndDuals<Int>,
    previousSuggested: Map<Int, SuggestedControl>,
): Residuals {
    // primal residual TODO("||ui - ziji||")
    val neighborsExit = neighboring(output.duals)
    val primalResidualLocal = neighborsExit.map<Double> { (id, value) ->
        (output.control - (value[id]?.suggestedControl?.zi ?: zeroSpeed())).norm()
    }.neighbors.values.max()
    val primalResidual = gossipMax(primalResidualLocal)
    // dual residual  TODO("||ziji current - ziji previous||")
    val dualResidualLocal = neighborsExit.map<Double> { (id, value) ->
        (value[id]?.suggestedControl?.zi?.minus(previousSuggested[id]?.zi ?: zeroSpeed()))?.norm() ?: 0.0
    }.neighbors.values.max()
    val dualResidual = gossipMax(dualResidualLocal)
    return Residuals(primalResidual, dualResidual)
}
