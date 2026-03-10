package it.unibo.collektive.qp.controlFunctions

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBException
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBQuadExpr
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.qp.carol.DualParams
import it.unibo.collektive.qp.carol.IncidentDuals
import it.unibo.collektive.qp.carol.SuggestedControl
import it.unibo.collektive.qp.dsl.GRBVector
import it.unibo.collektive.qp.dsl.addRhoNorm2Sq
import it.unibo.collektive.qp.utils.Robot
import it.unibo.collektive.qp.utils.SpeedControl2D
import it.unibo.collektive.qp.utils.Target
import it.unibo.collektive.qp.utils.minus
import it.unibo.collektive.qp.utils.plus
import it.unibo.collektive.qp.utils.toDoubleArray

// || u - u_nom||^2 + rho_s * delta^2 + rho_a / 2 * avg
/**
 * Local ADMM QP: minimizes deviation from nominal control plus slack and consensus penalties.
 *
 * @return optimal control and slack value; falls back to previous control on failure.
 */
fun GRBModel.minimizeADMMLocalQP(
    u: GRBVector,
    delta: GRBVar,
    robot: Robot,
    target: Target,
    average: DoubleArray,
    cardinality: Int,
): Pair<SpeedControl2D, Double> {
    val rhoSlack = 2.0
    val rhoADMM = 10.0
    val uNominal = (robot.position - target.position).toDoubleArray()
    val obj = GRBQuadExpr()
    // ||u - u_nom||^2
    // minimizeDeviation(u, uNo minal, delta, 1.0)
    obj.addRhoNorm2Sq(u, uNominal)
    // slack + rho_s * delta^2
    obj.addTerm(rhoSlack, delta)
    // rho_a / 2 * |N| ||u - m||^2
    obj.addRhoNorm2Sq(u, average, (rhoADMM / 2) * cardinality) // todo check correctness

    var result: Pair<SpeedControl2D, Double> = robot.control to 0.0
    try {
        setObjective(obj, GRB.MINIMIZE)
        // solve
        optimize()
        val status = get(GRB.IntAttr.Status)
        if (status == GRB.INFEASIBLE) {
            println("==== eh INFEASIBLE")
            computeIIS()
            write("logging/localModel.ilp")
        }
        if (status == GRB.OPTIMAL) {
            val uOptX = u[0].get(GRB.DoubleAttr.X)
            val uOptY = u[1].get(GRB.DoubleAttr.X)
            val deltaOpt = delta.get(GRB.DoubleAttr.X)
            println("Optimal control : u = ($uOptX, $uOptY)")
            result = SpeedControl2D(uOptX, uOptY) to deltaOpt
        } else {
            println("Optimization failed with status $status")
        }
    } catch (ex: GRBException) {
        println(
            "Minimization problem is infeasible, returning previous control: ${robot.control}. " +
                "Got exception: ${ex.message}",
        )
    }
    return result
}

// rho / 2 * ( ||z_ij,i - (ui + y_ij,i)||^2 + || z_ij,j - (uj + y_ij,j)||^2 )

/**
 * Common-edge ADMM QP: computes pairwise suggested controls for a neighbor pair.
 */
fun GRBModel.minimizeADMMCommonQP(
    zi: GRBVector,
    zj: GRBVector,
    robot: Robot,
    other: Robot,
    incidentDuals: IncidentDuals,
): SuggestedControl {
    val rhoADMM = 10.0
    val obj = GRBQuadExpr()
    val ui = robot.control
    val uj = other.control
    val yi = incidentDuals.yi
    val yj = incidentDuals.yj
    // ||zij,i - (ui + yij,i)||^2
    obj.addRhoNorm2Sq(zi, (ui + yi).toDoubleArray(), rhoADMM / 2)
    // + ||zij,j - (uj + yij,j)||^2
    obj.addRhoNorm2Sq(zj, (uj + yj).toDoubleArray(), rhoADMM / 2)

    var result = SuggestedControl(robot.control, other.control)
    try {
        setObjective(obj, GRB.MINIMIZE)
        // solve
        optimize()
        val status = get(GRB.IntAttr.Status)
        if (status == GRB.INFEASIBLE) {
            println("==== eh INFEASIBLE")
            computeIIS()
            write("logging/commonModel.ilp")
        }
        if (status == GRB.OPTIMAL) {
            val zxiOpt = zi[0].get(GRB.DoubleAttr.X)
            val zyiOpt = zi[1].get(GRB.DoubleAttr.X)
            val zxjOpt = zj[0].get(GRB.DoubleAttr.X)
            val zyjOpt = zj[1].get(GRB.DoubleAttr.X)
            println("Optimal control for me: u = ($zxiOpt, $zyiOpt)")
            println("Optimal control for other: u = ($zxjOpt, $zyjOpt)")
            result = SuggestedControl(SpeedControl2D(zxiOpt, zyiOpt), SpeedControl2D(zxjOpt, zyjOpt))
        } else {
            println("Optimization failed with status $status")
        }
    } catch (ex: GRBException) {
        println(
            "Minimization problem is infeasible, returning previous controls: ${robot.control} & ${other.control}. " +
                "Got exception: ${ex.message}",
        )
    }
    return result
}

// || u - u_nom||^2 + rho_s * delta^2 + rho_a / 2 * SUM ||i - z_ij,i + y_ij,i||^2

/**
 * Local ADMM QP: minimizes deviation from nominal control plus slack and consensus penalties.
 *
 * @return optimal control and slack value; falls back to previous control on failure.
 */
fun <ID : Comparable<ID>> GRBModel.minimizeADMMLocalQP(
    u: GRBVector,
    delta: GRBVar,
    robot: Robot,
    target: Target,
    duals: Map<ID, DualParams>,
): Pair<SpeedControl2D, Double> {
    val rhoSlack = 2.0
    val rhoADMM = 10.0
    val uNominal = (robot.position - target.position).toDoubleArray()
    val obj = GRBQuadExpr()
    // ||u - u_nom||^2
    // minimizeDeviation(u, uNo minal, delta, 1.0)
    obj.addRhoNorm2Sq(u, uNominal)
    // slack + rho_s * delta^2
    obj.addTerm(rhoSlack, delta)
    // rho_a / 2 * SUM ||ui - z_ij,i + y_ij,i||^2
    duals.forEach { (id, value) ->
        val suggested = value.suggestedControl.zi.toDoubleArray()
        val residual = value.incidentDuals.yi.toDoubleArray()
        obj.addRhoNorm2Sq(u, suggested - residual, rhoADMM / 2)
    }
    setObjective(obj, GRB.MINIMIZE)
    // solve
    optimize()
    val uOptX = u[0].get(GRB.DoubleAttr.X)
    val uOptY = u[1].get(GRB.DoubleAttr.X)
    val deltaOpt = delta.get(GRB.DoubleAttr.X)

    println("Optimal control: u = ($uOptX, $uOptY)")
    return SpeedControl2D(uOptX, uOptY) to deltaOpt
}
