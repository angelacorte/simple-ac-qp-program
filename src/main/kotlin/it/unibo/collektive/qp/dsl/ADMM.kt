package it.unibo.collektive.qp.dsl

import it.unibo.alchemist.collektive.device.CollektiveDevice
import it.unibo.alchemist.model.positions.Euclidean2DPosition
import it.unibo.collektive.aggregate.api.Aggregate
import it.unibo.collektive.aggregate.api.sharing
import it.unibo.collektive.aggregate.toMap
import it.unibo.collektive.aggregate.values
import it.unibo.collektive.alchemist.device.sensors.LocationSensor
import it.unibo.collektive.qp.utils.Obstacle
import it.unibo.collektive.qp.utils.Robot
import it.unibo.collektive.qp.utils.SpeedControl2D
import it.unibo.collektive.qp.utils.Target
import it.unibo.collektive.qp.utils.Vector2D
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
import it.unibo.collektive.stdlib.spreading.gossipMax
import org.apache.commons.lang3.compare.ComparableUtils.min

/**
 * Absolute and relative tolerances used to stop ADMM iterations.
 *
 * @property primal primal residual threshold.
 * @property dual dual residual threshold.
 */
data class Tolerance(val primal: Double, val dual: Double)

/**
 * Dual variables associated to an edge (stored from the local and neighbor perspective).
 *
 * @property yi local dual variable.
 * @property yj neighbor dual variable.
 */
data class IncidentDuals(val yi: Vector2D, val yj: Vector2D) // y_{ij}^{i}, y_{ij}^{j}

/**
 * Suggested consensus controls for the two edge endpoints.
 *
 * @property zi control suggested for the local node.
 * @property zj control suggested for the neighbor node.
 */
data class SuggestedControl(val zi: SpeedControl2D, val zj: SpeedControl2D) // z_{ij} = [z_ij^i z_ij^j]

/**
 * Local dual bundle for a neighbor edge.
 *
 * @property suggestedControl consensus controls for the edge.
 * @property incidentDuals associated dual variables.
 */
data class DualParams(val suggestedControl: SuggestedControl, val incidentDuals: IncidentDuals)

/**
 * Control decision and dual map for the current node.
 *
 * @property control optimal control for the node.
 * @property duals per-neighbor dual parameters.
 */
data class ControlAndDuals<ID : Comparable<ID>>(val control: SpeedControl2D, val duals: Map<ID, DualParams>)

/**
 * Primal/dual residuals for stopping criteria.
 *
 * @property primalResidual maximum primal residual.
 * @property dualResidual maximum dual residual.
 */
data class Residuals(val primalResidual: Double, val dualResidual: Double)

/**
 * Control paired with residuals.
 *
 * @property control optimal control.
 * @property residuals residual values.
 */
data class ControlAndResiduals(val control: SpeedControl2D, val residuals: Residuals)

/**
 * Main aggregate entrypoint: runs distributed ADMM to compute a safe control and applies it when converged.
 */
fun Aggregate<Int>.entrypoint(position: LocationSensor, device: CollektiveDevice<Euclidean2DPosition>) =
    context(position, device) {
        val maxIter = device["MaxIterations"] as Int
        val tolerance = Tolerance(device["PrimalTolerance"], device["DualTolerance"])
        val robot = getRobot()
        val target: Target = getTarget(device["TargetID"] as Number)
        val communicationDistance: Double? = device["CommunicationDistance"]
        val obstacle = getObstacle()

        val res: Pair<Int, ControlAndResiduals> =
            evolving(0 to ControlAndDuals(robot.control, emptyMap<Int, DualParams>())) { previousDuals ->
                val output: ControlAndDuals<Int> =
                    admm(robot, target, obstacle, communicationDistance, previousDuals.second.duals) // local update already done
                val previousSuggested: Map<Int, SuggestedControl> = previousDuals.second.duals.toMap().mapValues {
                    it.value.suggestedControl
                }

                // r_ij^t = max ||ui - zij,i||
                val rijt: Double? = output.duals.maxOfOrNull { (_, value) ->
                    (output.control - value.suggestedControl.zi).norm()
                }
//        val rijt = output.suggested.maxOfOrNull { (output.control - it.controlForLocal).norm() }
                // r^t = max ri^t
                val rt = gossipMax(rijt ?: 0.0)

                // sit = \rhoa max ||zij^it - zij^it-1||
                val sit: Double? = output.duals.maxOfOrNull { (id, value) ->
                    val prev: SuggestedControl = previousSuggested[id] ?: SuggestedControl(zeroSpeed(), zeroSpeed())
                    (value.suggestedControl.zi - prev.zi).norm()
                }
//        val sit = output.suggested.map { (it.controlForOther - previous).norm() }.max()
                val st = gossipMax(sit ?: 0.0)

                // primal residual TODO("||ui - ziji||")
//        val neighborsExit = neighboring(output)
//        val neighborsExit = neighboring(output.duals[localId])
//        val primalResidualLocal = neighborsExit.map<Double> { (id, value) ->  (output.control - value?.suggestedControl.zi).norm() }.neighbors.values.max()
//        val primalResidual = gossipMax(primalResidualLocal)
                // dual residual  TODO("||ziji current - ziji previous||")
//        val dualResidualLocal = neighborsExit.map<Double> { (id, value) -> (value.suggestedControl.zi - previousSuggested[id].zi).norm() }.neighbors.values.max()
//        val dualResidualLocal = neighborsExit.map<Double> { TODO("||ziji current - ziji previous||") }.neighbors.values.max()
//        val dualResidual = gossipMax(dualResidualLocal)

                val nextIter = previousDuals.first + 1
                val iter = if (nextIter == maxIter) 0 else nextIter
                (iter to output).yielding { nextIter to ControlAndResiduals(output.control, Residuals(rt, st)) }
            }
        device["Iteration"] = res.first
        // stop if residuals < threshold
        if ((res.second.residuals.primalResidual <= tolerance.primal &&
                res.second.residuals.dualResidual <= tolerance.dual) || res.first == maxIter) {
            robot.applyControl(res.second.control)
        }
    }

/**
 * Executes one ADMM round: local update plus dual refresh for all neighbors.
 */
fun <ID : Comparable<ID>> Aggregate<ID>.admm(
    robot: Robot,
    target: Target,
    obstacle: Obstacle,
    communicationDistance: Double?,
    duals: Map<ID, DualParams>,
): ControlAndDuals<ID> = sharing(robot) { fieldU ->
    val nbrControls = fieldU.map { (_, value) ->
        value.control
    }.neighbors.values.list
    val avg: SpeedControl2D = if (nbrControls.isEmpty()) zeroSpeed() else nbrControls.avg()
    val control: SpeedControl2D =
        executeLocalADMM(robot, target, obstacle, avg.toDoubleArray(), fieldU.neighbors.values.size)
    val robotUpdated = robot.copy(control = control)
    val commons: Map<ID, DualParams> = fieldU.neighbors.toMap().mapValues { (id, r) ->
        val incidentDuals = duals[id]?.incidentDuals ?: IncidentDuals(initVector2D(), initVector2D())
        val (zi, zj) = robotAvoidanceAndCommunicationRangeCBF(robotUpdated, r, communicationDistance, incidentDuals)
        // local dual update
        // y_ij^i,t+1 = y_ij^i,t + (u_i^t+1 - z_ij^i,t+1) // y_ij^j,t+1 = y_ij^j,t + (u_j^t+1 - z_ij^j,t+1)
        val newIncidentDuals = IncidentDuals(incidentDuals.yi + control - zi, incidentDuals.yj + r.control - zj)
        DualParams(SuggestedControl(zi, zj), newIncidentDuals)
    }
    robotUpdated.yielding { ControlAndDuals(control, commons) }
}

/**
 * Local QP wrapper computing the optimal control for obstacle avoidance and goal tracking.
 */
fun executeLocalADMM(
    robot: Robot,
    target: Target,
    obstacle: Obstacle,
    avg: DoubleArray,
    cardinality: Int,
): SpeedControl2D {
    val (uWanted, deltaNom) = avoidObstacleGoToTarget(robot, target, obstacle, avg, cardinality)
    return uWanted
}

/**
 * Edge owner policy where the device with the smallest id owns the update.
 */
private fun <ID : Comparable<ID>> Aggregate<ID>.isOwner(id: ID): Boolean = min(localId, id) == localId

context(device: CollektiveDevice<Euclidean2DPosition>)
/**
 * Applies the computed control to the robot by moving its node inside the environment.
 */
fun Robot.applyControl(
    control: SpeedControl2D,
) {
    moveNodeToPosition(this.position + control)
}
