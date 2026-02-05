package it.unibo.collektive.qp

import it.unibo.alchemist.collektive.device.CollektiveDevice
import it.unibo.collektive.aggregate.api.Aggregate
import it.unibo.collektive.aggregate.api.neighboring
import it.unibo.collektive.alchemist.device.sensors.EnvironmentVariables
import it.unibo.collektive.alchemist.device.sensors.LocationSensor
import it.unibo.collektive.qp.dsl.localBarrierAndGoToTarget
import it.unibo.collektive.qp.utils.Robot
import it.unibo.collektive.qp.utils.SpeedControl2D
import it.unibo.collektive.qp.utils.Target
import it.unibo.collektive.qp.utils.Vector2D
import it.unibo.collektive.qp.utils.getObstacle
import it.unibo.collektive.qp.utils.getRobot
import it.unibo.collektive.qp.utils.getTarget
import it.unibo.collektive.qp.utils.initVector2D
import it.unibo.collektive.qp.utils.minus
import it.unibo.collektive.qp.utils.plus
import it.unibo.collektive.qp.utils.squaredNorm
import it.unibo.collektive.qp.utils.uNom
import kotlin.math.pow
import org.apache.commons.lang3.compare.ComparableUtils.min

data class Edge<ID: Comparable<ID>>(val local: ID, val other: ID)

data class SuggestedControl<ID: Comparable<ID>>(val controlForLocal:SpeedControl2D, val controlForOther: SpeedControl2D) // z_{ij} = [u_i u_j]

data class Residual<ID: Comparable<ID>>(val valueForLocal: Vector2D, val valueForOther: Vector2D) // y_{ij}^{i}, y_{ij}^{j}

data class Coupled<ID: Comparable<ID>>(val edge: Edge<ID>, val suggestedControl: SuggestedControl<ID>, val residuals: Residual<ID>)

data class Parameters<ID: Comparable<ID>>(val robot: Robot<ID>, val edges: List<Coupled<ID>>)

data class Tolerance(val primal: Double, val dual: Double)

fun Aggregate<Int>.fullADMM(
    position: LocationSensor,
    env: EnvironmentVariables,
    device: CollektiveDevice<*>,
): Unit = context(position, device) {
    val maxIterations = 50
    val target: Target = getTarget(env["TargetID"] as Number)
    val tolerance = Tolerance(1e-3, 1e-3) // used the same tolerance as in ADMM Example3
    val robot = getRobot(localId)
    val neighborsPositions = neighboring(robot.position)
    val localParams = initParameters(robot)
    evolve(localParams) { params ->
        for (iteration in 0 until maxIterations) { // todo this cycle breaks aggregate
            executeCoreADMM(robot, target, params)
            // stop if got to residual threshold, no need to execute all iterations
            if (localResidualThreshold()) break
        }
        updateLocalControl()
        params // should be the updated ones
    }
    applyControl()
}

context(position: LocationSensor, device: CollektiveDevice<*>)
fun Aggregate<Int>.executeCoreADMM(robot: Robot<Int>, target: Target, params: Parameters<Int>) {
    val uFromQP = executeLocalQP(robot, target)
    val uNom = uFromQP.uNom(target)
    val uWanted = updateLocalParams<Int>(params, uFromQP, uNom)
    // sendToNeighborsMyNextU(uWanted)
    val uNeighbors = neighboring(uWanted).neighbors.list// receiveFromNeighborsTheirNextU<Int>()
    // todo collapse the two above into a neighboring
    val residuals = shareResiduals()
    uNeighbors.forEach { n ->
        if(isOwner(n.id)) {
            executeCommonQP()
            // todo and share them
        } else {
            // receive update desired speed from owner
        }
    }
    uNeighbors.forEach { n ->
        updateResiduals()
    }
    val localResidualsUpdated = updateLocalResidual<Double>()
    updateDualResidual(localResidualsUpdated)
}

/**
 * Initialize ADMM parameters for the first iteration of the algorithm, for the given [robot].
 */
fun <ID: Comparable<ID>> Aggregate<ID>.initParameters(robot: Robot<ID>): Parameters<ID> {
    val neighborsVel = neighboring(robot.velocity)
    val coupledEdges: List<Coupled<ID>> = neighborsVel.neighbors.list.map {
        Coupled(
            Edge(localId, it.id),
            SuggestedControl(robot.velocity, it.value),
            Residual(initVector2D(),initVector2D())
        )
    }
//    val controls: List<SuggestedControl<ID>> = neighborsVel.neighbors.list.map { SuggestedControl(Edge(localId, it.id), robot.velocity to it.value) }
//    val residuals: List<Residual<ID>> = neighborsVel.neighbors.ids.list.map { Residual(Edge(localId, it), 0.0, 0.0) }
    return Parameters(robot, coupledEdges)
}

/**
 * Edge owner policy where the device with the smallest [ID] is the owner.
 * Where [id] is the neighbor [ID] to compare.
 */
private fun <ID: Comparable<ID>> Aggregate<ID>.isOwner(id: ID): Boolean = min(localId, id) == localId

context(device: CollektiveDevice<*>)
fun <ID: Comparable<ID>> executeLocalQP(robot: Robot<ID>, target: Target): SpeedControl2D =
    localBarrierAndGoToTarget(robot, getObstacle(), target)

fun <ID: Comparable<ID>> updateLocalParams(params: Parameters<Int>, uFromQP: SpeedControl2D, uNom: SpeedControl2D): Parameters<ID> {
    val rhoSlack = 1.0
    val rhoADMM = 10.0
    val delta = 1.0 // todo get this from QP
    val summatory = params.edges.sumOf { (uFromQP - it.suggestedControl.controlForLocal + it.residuals.valueForLocal).squaredNorm() }
    val argmin = (uFromQP - uNom).squaredNorm() + rhoSlack * delta.pow(2) + (rhoADMM / 2) * summatory
}

fun sendToNeighborsMyNextU(uWanted: SpeedControl2D): SpeedControl2D = TODO()
// todo merge this two into a single nbr
fun <ID> receiveFromNeighborsTheirNextU(): List<Robot<ID>> = TODO()

fun shareResiduals(): SpeedControl2D = TODO("not a speed control, but for sure a neighboring")

fun executeCommonQP(): List<SpeedControl2D> = TODO("list of speed associated with id of neighbor. exchange? multi gradient?")

fun updateResiduals(): Unit = TODO("of all pairs")

fun <Residual> updateLocalResidual(): Residual = TODO()

fun updateDualResidual(res: Double): Unit = TODO("to the neighbors")

fun updateLocalControl(): Unit = TODO("given by the overall iterations of ADMM")

fun localResidualThreshold(): Boolean = TODO()

fun applyControl(): Unit = TODO("apply control to current robot")
