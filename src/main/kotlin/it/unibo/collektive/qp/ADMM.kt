package it.unibo.collektive.qp

import it.unibo.collektive.aggregate.api.Aggregate
import it.unibo.collektive.aggregate.api.neighboring
import it.unibo.collektive.aggregate.ids
import it.unibo.collektive.alchemist.device.sensors.EnvironmentVariables
import it.unibo.collektive.alchemist.device.sensors.LocationSensor
import it.unibo.collektive.qp.utils.Robot
import it.unibo.collektive.qp.utils.SpeedControl2D
import it.unibo.collektive.qp.utils.getRobot
import org.apache.commons.lang3.compare.ComparableUtils.min

data class Edge<ID>(val localID: ID, val other: ID)

data class Residual<ID>(val edge: Edge<ID>, val valueForLocal: Double, val valueForOther: Double) // y_{ij}^{i}, y_{ij}^{j}

data class SuggestedControl<ID>(val edge: Edge<ID>, val control: Pair<SpeedControl2D, SpeedControl2D>) // z_{ij} = [u_i u_j]

data class Parameters<ID>(val robot: Robot<ID>, val suggestedControls: List<SuggestedControl<ID>>, val residuals: List<Residual<ID>>)

data class Tolerance(val primal: Double, val dual: Double)

fun Aggregate<Int>.fullADMM(
    position: LocationSensor,
    env: EnvironmentVariables,
): Unit = context(position, env) {
    val maxIterations = 50
    val tolerance = Tolerance(1e-3, 1e-3) // used the same tolerance as in ADMM Example3
    val robot = getRobot(localId)
    val neighborsPositions = neighboring(robot.position)
    val localParams = initParameters(robot)
    evolve(localParams) { params ->
        for (iteration in 0 until maxIterations) {
            executeCoreADMM()
            // stop if got to residual threshold, no need to execute all iterations
            if (localResidualThreshold()) break
        }
        updateLocalControl()
        params // should be the updated ones
    }
    applyControl()
}

fun <ID: Comparable<ID>> Aggregate<ID>.initParameters(robot: Robot<ID>): Parameters<ID> {
    val neighborsVel = neighboring(robot.velocity)
    val controls: List<SuggestedControl<ID>> = neighborsVel.neighbors.list.map { SuggestedControl(Edge(localId, it.id), robot.velocity to it.value) }
    val residuals: List<Residual<ID>> = neighborsVel.neighbors.ids.list.map { Residual(Edge(localId, it), 0.0, 0.0) }
    return Parameters(robot, controls, residuals)
}

fun Aggregate<Int>.executeCoreADMM() {
    val uWanted = executeLocalQP()
    updateLocalVars<Int>()
    sendToNeighborsMyNextU(uWanted)
    val neighborsInfo = receiveFromNeighborsTheirNextU<Int>()
    // todo collapse the two above into a neighboring
    val residuals = shareResiduals()
    neighborsInfo.forEach { n ->
        if(isOwner(n.id)) {
            executeCommonQP()
            // todo and share them
        } else {
            // receive update desired speed from owner
        }
    }
    neighborsInfo.forEach { n ->
        updateResiduals()
    }
    val localResidualsUpdated = updateLocalResidual<Double>()
    updateDualResidual(localResidualsUpdated)
}

/**
 * Edge owner policy where the device with the smallest [ID] is the owner.
 * Where [id] is the neighbor [ID] to compare,
 */
private fun <ID: Comparable<ID>> Aggregate<ID>.isOwner(id: ID): Boolean = min(localId, id) == localId

fun executeLocalQP(): SpeedControl2D = TODO("execute QP with local safety constraints and CLF")

fun <ID> updateLocalVars(): Robot<ID> = TODO("not sure if it has to return a robot")

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
