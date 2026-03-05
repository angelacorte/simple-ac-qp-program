package it.unibo.collektive.qp.dsl

import it.unibo.alchemist.collektive.device.CollektiveDevice
import it.unibo.alchemist.model.positions.Euclidean2DPosition
import it.unibo.collektive.aggregate.api.Aggregate
import it.unibo.collektive.aggregate.api.neighboring
import it.unibo.collektive.aggregate.api.sharing
import it.unibo.collektive.aggregate.toMap
import it.unibo.collektive.aggregate.values
import it.unibo.collektive.alchemist.device.sensors.LocationSensor
import it.unibo.collektive.qp.carol.ControlAndDuals
import it.unibo.collektive.qp.carol.DualParams
import it.unibo.collektive.qp.carol.IncidentDuals
import it.unibo.collektive.qp.carol.SuggestedControl
import it.unibo.collektive.qp.carol.Tolerance
import it.unibo.collektive.qp.utils.Obstacle
import it.unibo.collektive.qp.utils.Robot
import it.unibo.collektive.qp.utils.SpeedControl2D
import it.unibo.collektive.qp.utils.Target
import it.unibo.collektive.qp.utils.avg
import it.unibo.collektive.qp.utils.getObstacle
import it.unibo.collektive.qp.utils.getRobot
import it.unibo.collektive.qp.utils.getTarget
import it.unibo.collektive.qp.utils.initVector2D
import it.unibo.collektive.qp.utils.minus
import it.unibo.collektive.qp.utils.moveNodeToPosition
import it.unibo.collektive.qp.utils.norm
import it.unibo.collektive.qp.utils.plus
import it.unibo.collektive.qp.utils.toDoubleArray
import it.unibo.collektive.qp.utils.zeroSpeed
import it.unibo.collektive.stdlib.collapse.max
import it.unibo.collektive.stdlib.spreading.gossipMax
import org.apache.commons.lang3.compare.ComparableUtils.min

/**
 * Main aggregate entrypoint: runs distributed ADMM to compute a safe control and applies it when converged.
 */
fun Aggregate<Int>.entrypoint(position: LocationSensor, device: CollektiveDevice<Euclidean2DPosition>) =
    context(position, device) {
        val maxIter: Int = device["MaxIterations"]
        val tolerance = Tolerance(device["PrimalTolerance"], device["DualTolerance"])
        val robot = getRobot()
        val target: Target = getTarget(device["TargetID"] as Number)
        val communicationDistance: Double? = device["CommunicationDistance"]
        val obstacle = getObstacle()
        val res = controlLoop(robot, target, obstacle, communicationDistance, maxIter, tolerance)
        // stop if residuals < threshold
        if (res.first) {
            device["Velocity"] = res.second
            robot.applyControl(res.second)
        }
    }

context(device: CollektiveDevice<*>)
fun Aggregate<Int>.controlLoop(
    robot: Robot,
    target: Target,
    obstacle: Obstacle?,
    communicationDistance: Double?,
    maxIter: Int,
    tolerance: Tolerance,
): Pair<Boolean, SpeedControl2D> = evolving(0 to ControlAndDuals(robot.control, emptyMap())) { previousDuals ->
    val output: ControlAndDuals<Int> =
        coreADMM(robot.copy(control = previousDuals.second.control), target, obstacle, communicationDistance, previousDuals.second.duals) // local update already done
    val previousSuggested: Map<Int, SuggestedControl> = previousDuals.second.duals.toMap().mapValues {
        it.value.suggestedControl
    }
    val (rt, st) = residualUpdateNoNbr(output, previousSuggested)
//    val (primalResidual, dualResidual) = residualUpdate(output, previousSuggested)
    device["rt"] = rt
    device["st"] = st
    val nextIter = previousDuals.first + 1
    val (shouldApply, iter) = when {
        (rt <= tolerance.primal && st <= tolerance.dual) || nextIter >= maxIter -> true to 0
        else -> false to nextIter
    }.also { device["iter"] = it.second }
    (iter to output).yielding { shouldApply to output.control}
}

context(device: CollektiveDevice<*>)
private fun Aggregate<Int>.residualUpdate(
    output: ControlAndDuals<Int>,
    previousSuggested: Map<Int, SuggestedControl>,
): Pair<Double, Double> {
    // primal residual TODO("||ui - ziji||")
    val neighborsExit = neighboring(output.duals)
    val primalResidualLocal = neighborsExit.map<Double> { (id, value) ->
        (output.control - (value[id]?.suggestedControl?.zi ?: zeroSpeed())).norm()
    }.neighbors.values.max()
    val primalResidual = gossipMax(primalResidualLocal)
    // dual residual  TODO("||ziji current - ziji previous||")
    val dualResidualLocal = neighborsExit.map<Double> { (id, value) ->
        (value[id]?.suggestedControl?.zi?.minus(
            previousSuggested[id]?.zi ?: zeroSpeed()
        ))?.norm() ?: 0.0
    }.neighbors.values.max()
    val dualResidual = gossipMax(dualResidualLocal)
    return primalResidual to dualResidual
}

context(device: CollektiveDevice<*>)
private fun Aggregate<Int>.residualUpdateNoNbr(
    output: ControlAndDuals<Int>,
    previousSuggested: Map<Int, SuggestedControl>,
): Pair<Double, Double> {
    val currentSuggested = output.duals.filterNot { it.key == localId }.mapValues { it.value.suggestedControl }
    // r_ij^t = max ||ui - zij,i||
    val rijt: Double = currentSuggested.maxOfOrNull { (id, value) ->
        device["outcontrol"] = output.control
        val va = (output.control - value.zi).norm()
        device["zi$id"] = value.zi
        va
    } ?: 0.0
    device["rijt"] = rijt
    // r^t = max ri^t
    val rt = gossipMax(rijt)

    // sit = \rhoa max ||zij^it - zij^it-1||
    val sit = currentSuggested.maxOfOrNull { (id, value) ->
        val prev = previousSuggested[id] ?: SuggestedControl(zeroSpeed(), zeroSpeed())
        (value.zi - prev.zi).norm()
    } ?: 0.0
    device["sit"] = sit
    val st = gossipMax(sit)
    return Pair(rt, st)
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
    val nbrControls = controls.map { (_, value) ->
        value.control
    }.neighbors.values.list
    val avg: SpeedControl2D = if (nbrControls.isEmpty()) zeroSpeed() else nbrControls.avg()
    val control: SpeedControl2D =
        executeLocalADMM(robot, target, obstacle, avg.toDoubleArray(), controls.neighbors.values.size)
    val robotUpdated = robot.copy(control = control)
    val commons: Map<ID, DualParams> = controls.neighbors.toMap().mapValues { (id, neighbor) ->
        val incidentDuals = duals[id]?.incidentDuals ?: IncidentDuals(initVector2D(), initVector2D())
        val (zi, zj) = executeCommonADMM(robotUpdated, neighbor, communicationDistance, incidentDuals)
        // local dual update
        // y_ij^i,t+1 = y_ij^i,t + (u_i^t+1 - z_ij^i,t+1) // y_ij^j,t+1 = y_ij^j,t + (u_j^t+1 - z_ij^j,t+1)
        val newIncidentDuals = IncidentDuals(incidentDuals.yi + control - zi, incidentDuals.yj + neighbor.control - zj)
        DualParams(SuggestedControl(zi, zj), newIncidentDuals)
    }
    robotUpdated.yielding { ControlAndDuals(control, commons) }
}

private fun executeCommonADMM(
    robotUpdated: Robot,
    other: Robot,
    communicationDistance: Double?,
    incidentDuals: IncidentDuals,
): SuggestedControl = robotAvoidanceAndCommunicationRangeCBF(robotUpdated, other, communicationDistance, incidentDuals)

/**
 * Local QP wrapper computing the optimal control for obstacle avoidance and goal tracking.
 */
fun executeLocalADMM(
    robot: Robot,
    target: Target,
    obstacle: Obstacle?,
    avg: DoubleArray,
    cardinality: Int,
): SpeedControl2D {
    val (uWanted, deltaNom) = avoidObstacleGoToTarget(robot, target, obstacle, avg, cardinality)
    return uWanted
}

context(device: CollektiveDevice<Euclidean2DPosition>)
/**
 * Applies the computed control to the robot by moving its node inside the environment.
 */
fun Robot.applyControl(
    control: SpeedControl2D,
) {

    moveNodeToPosition(this.position + control)
}

/**
 * Edge owner policy where the device with the smallest id owns the update.
 */
private fun <ID : Comparable<ID>> Aggregate<ID>.isOwner(id: ID): Boolean = min(localId, id) == localId
