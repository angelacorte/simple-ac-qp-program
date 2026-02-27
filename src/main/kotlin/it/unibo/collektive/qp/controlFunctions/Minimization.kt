package it.unibo.collektive.qp.controlFunctions

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBQuadExpr
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.qp.dsl.GRBVector
import it.unibo.collektive.qp.dsl.LocalUpdate
import it.unibo.collektive.qp.dsl.SuggestedControl
import it.unibo.collektive.qp.dsl.addRhoNorm2Sq
import it.unibo.collektive.qp.utils.Robot
import it.unibo.collektive.qp.utils.SpeedControl2D
import it.unibo.collektive.qp.utils.Target
import it.unibo.collektive.qp.utils.minus
import it.unibo.collektive.qp.utils.toDoubleArray

// || u - u_nom||^2 + rho_s * delta^2 + rho_a / 2 * SUM ||i - z_ij,i + y_ij,i||^2
fun GRBModel.minimizeADMMLocalQP(u: GRBVector, delta: GRBVar, robot: Robot, target: Target, average: DoubleArray, cardinality: Int): Pair<SpeedControl2D, Double> {
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

    setObjective(obj, GRB.MINIMIZE)
    // solve
    optimize()
    val uOptX = u[0].get(GRB.DoubleAttr.X)
    val uOptY = u[1].get(GRB.DoubleAttr.X)
    val deltaOpt = delta.get(GRB.DoubleAttr.X)

    println("Optimal control : u = ($uOptX, $uOptY)")
    return SpeedControl2D(uOptX, uOptY) to deltaOpt
}

// rho / 2 * ( ||z_ij,i - (ui + y_ij,i)||^2 + || z_ij,j - (uj + y_ij,j)||^2 )
fun GRBModel.minimizeADMMCommonQP(zi: GRBVector, zj: GRBVector, robot: Robot, other: Robot, localUpdate: LocalUpdate): SuggestedControl {
    val rhoADMM = 10.0
    val obj = GRBQuadExpr()
    val ui = robot.control.toDoubleArray()
    val uj = other.control.toDoubleArray()
    val yi = localUpdate.yi.toDoubleArray()
    val yj = localUpdate.yj.toDoubleArray()
    // ||zij,i - (ui + yij,i)||^2
    obj.addRhoNorm2Sq(zi, ui - yi, rhoADMM / 2)
    // + ||zij,j - (uj + yij,j)||^2
    obj.addRhoNorm2Sq(zj, uj - yj, rhoADMM / 2)
    setObjective(obj, GRB.MINIMIZE)
    // solve
    optimize()
    val zxiOpt = zi[0].get(GRB.DoubleAttr.X)
    val zyiOpt = zi[1].get(GRB.DoubleAttr.X)
    val zxjOpt = zj[0].get(GRB.DoubleAttr.X)
    val zyjOpt = zj[1].get(GRB.DoubleAttr.X)
    println("Optimal control for me: u = ($zxiOpt, $zyiOpt)")
    println("Optimal control for other: u = ($zxjOpt, $zyjOpt)")
    return SuggestedControl(SpeedControl2D(zxiOpt, zyiOpt), SpeedControl2D(zxjOpt, zyjOpt))
}

//fun <ID: Comparable<ID>> GRBModel.minimizeADMMLocalQP(u: GRBVector, delta: GRBVar, robot: Robot<ID>, target: Target, edges: List<Coupled>): Pair<SpeedControl2D, Double> {
//    val rhoSlack = 2.0
//    val rhoADMM = 10.0
//    val uNominal = (robot.position - target.position).toDoubleArray()
//    val obj = GRBQuadExpr()
//    // ||u - u_nom||^2
//    // minimizeDeviation(u, uNo minal, delta, 1.0)
//    obj.addRhoNorm2Sq(u, uNominal)
//    // slack + rho_s * delta^2
//    obj.addTerm(rhoSlack, delta)
//    // rho_a / 2 * SUM ||ui - z_ij,i + y_ij,i||^2
//    edges.forEach { e ->
//        val suggested = e.suggestedControl.controlForLocal.toDoubleArray()
//        val residual = e.residuals.valueForLocal.toDoubleArray()
//        obj.addRhoNorm2Sq(u, suggested - residual, rhoADMM / 2)
//    }
//    setObjective(obj, GRB.MINIMIZE)
//    // solve
//    optimize()
//    val uOptX = u[0].get(GRB.DoubleAttr.X)
//    val uOptY = u[1].get(GRB.DoubleAttr.X)
//    val deltaOpt = delta.get(GRB.DoubleAttr.X)
//
//    println("Optimal control: u = ($uOptX, $uOptY)")
//    return SpeedControl2D(uOptX, uOptY) to deltaOpt
//}
