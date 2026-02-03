package it.unibo.collektive.qp

import it.unibo.collektive.aggregate.api.Aggregate
import it.unibo.collektive.qp.utils.Robot
import it.unibo.collektive.qp.utils.SpeedControl2D

fun Aggregate<Int>.fullADMM() {
     val maxIterations = 50
    for (iteration in 1..maxIterations) {
        executeADMM()
        // stop if got to residual threshold, no need to execute all iterations
    }
    updateLocalControl()
}

fun Aggregate<Int>.executeADMM() {
    val uWanted = executeLocalQP()
    updateLocalVars()
    sendToNeighborsMyNextU(uWanted)
    val neighborsInfo = receiveFromNeighborsTheirNextU()
    // todo collapse the two above into a neighboring
    val residuals = shareResiduals()
    neighborsInfo.forEach { n ->
        if(checkIfOwner(localId, n.id)) {
            computeCommonQP()
            // todo and share them
        } else {
            // receive update desired speed from owner
        }
    }
    neighborsInfo.forEach { n ->
        updateResiduals()
    }
    val residualsUpdated = localCheckOnResidualValue<Double>()
    sendUpdatedResiduals(residualsUpdated)
}

private fun Aggregate<Int>.checkIfOwner(localId: Int, id: Number): Boolean {
    TODO("Not yet implemented")
}

fun executeLocalQP(): SpeedControl2D = TODO("execute QP with local safety constraints and CLF")

fun updateLocalVars(): Robot = TODO("not sure if it has to return a robot")

fun sendToNeighborsMyNextU(uWanted: SpeedControl2D): SpeedControl2D = TODO()
// todo merge this two into a single nbr
fun receiveFromNeighborsTheirNextU(): List<Robot> = TODO()

fun shareResiduals(): SpeedControl2D = TODO("not a speed control, but for sure a neighboring")

fun computeCommonQP(): List<SpeedControl2D> = TODO("list of speed associated with id of neighbor. exchange? multi gradient?")

fun updateResiduals(): Unit = TODO("of all pairs")

fun <Residual> localCheckOnResidualValue(): Residual = TODO()

fun sendUpdatedResiduals(res: Double): Unit = TODO("to the neighbors")

fun updateLocalControl(): Unit = TODO("given by the overall iterations of ADMM")
