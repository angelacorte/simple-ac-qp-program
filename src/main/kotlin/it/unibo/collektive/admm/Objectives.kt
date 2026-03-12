package it.unibo.collektive.admm

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBException
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBQuadExpr
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.model.Robot
import it.unibo.collektive.model.SpeedControl2D
import it.unibo.collektive.model.Target
import it.unibo.collektive.model.minus
import it.unibo.collektive.model.plus
import it.unibo.collektive.model.toDoubleArray
import it.unibo.collektive.solver.gurobi.GRBVector
import it.unibo.collektive.solver.gurobi.QpSettings
import it.unibo.collektive.solver.gurobi.addRhoNorm2Sq
import java.io.File

/**
 * Common-edge ADMM QP: computes pairwise suggested controls for a neighbor pair.
 */
fun GRBModel.minimizeADMMCommonQP(
    zi: GRBVector,
    zj: GRBVector,
    robot: Robot,
    other: Robot,
    incidentDuals: IncidentDuals,
    settings: QpSettings = QpSettings(),
    commSlack: GRBVar? = null,
): SuggestedControl {
    val obj = GRBQuadExpr()
    val ui = robot.control
    val uj = other.control
    val yi = incidentDuals.yi
    val yj = incidentDuals.yj
    obj.addRhoNorm2Sq(zi, (ui + yi).toDoubleArray(), settings.rhoADMM / 2)
    obj.addRhoNorm2Sq(zj, (uj + yj).toDoubleArray(), settings.rhoADMM / 2)
    commSlack?.let { obj.addCommSlack(it, settings) }
    return solveCommon(obj, zi, zj, robot, other, settings)
}

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
    settings: QpSettings = QpSettings(),
): Pair<SpeedControl2D, Double> {
    val uNominal = (target.position - robot.position).toDoubleArray()
    val obj = GRBQuadExpr()
    obj.addRhoNorm2Sq(u, uNominal)
    obj.addSlack(delta, settings)
    duals.forEach { (_, value) ->
        val suggested = value.suggestedControl.zi.toDoubleArray()
        val residual = value.incidentDuals.yi.toDoubleArray()
        obj.addRhoNorm2Sq(u, suggested - residual, settings.rhoADMM / 2)
    }
    return solveLocal(u, delta, obj, robot, settings)
}

private fun GRBQuadExpr.addSlack(delta: GRBVar, settings: QpSettings) {
    if (settings.slackQuadratic) {
        addTerm(settings.rhoSlack, delta, delta)
    } else {
        addTerm(settings.rhoSlack, delta)
    }
}

private fun GRBQuadExpr.addCommSlack(delta: GRBVar, settings: QpSettings) {
    if (settings.slackQuadratic) {
        addTerm(settings.rhoCommSlack, delta, delta)
    } else {
        addTerm(settings.rhoCommSlack, delta)
    }
}

private fun GRBModel.writeIIS(fileName: String) {
    val folder = File("logging")
    folder.mkdirs()
    computeIIS()
    write(File(folder, fileName).path)
}

private fun GRBModel.solveLocal(
    u: GRBVector,
    delta: GRBVar,
    obj: GRBQuadExpr,
    robot: Robot,
    settings: QpSettings,
): Pair<SpeedControl2D, Double> = try {
    setObjective(obj, GRB.MINIMIZE)
    optimize()
    val status = get(GRB.IntAttr.Status)
    if (status == GRB.INFEASIBLE) {
        writeIIS("localModel.ilp")
    }
    if (get(GRB.IntAttr.SolCount) > 0) {
        val uOptX = u[0].get(GRB.DoubleAttr.X)
        val uOptY = u[1].get(GRB.DoubleAttr.X)
        val deltaOpt = delta.get(GRB.DoubleAttr.X)
        SpeedControl2D(uOptX, uOptY) to deltaOpt
    } else {
        println("Local QP: no solution found (status $status)")
        robot.control to 0.0
    }
} catch (ex: GRBException) {
    println("${ex.message} - Minimization problem is infeasible, returning control: ${robot.control}.",)
    robot.control to 0.0
}

private fun GRBModel.solveCommon(
    obj: GRBQuadExpr,
    zi: GRBVector,
    zj: GRBVector,
    robot: Robot,
    other: Robot,
    settings: QpSettings,
): SuggestedControl = try {
    setObjective(obj, GRB.MINIMIZE)
    optimize()
    val status = get(GRB.IntAttr.Status)
    if (status == GRB.INFEASIBLE) {
        writeIIS("commonModel.ilp")
    }
    if (get(GRB.IntAttr.SolCount) > 0) {
        val zxiOpt = zi[0].get(GRB.DoubleAttr.X)
        val zyiOpt = zi[1].get(GRB.DoubleAttr.X)
        val zxjOpt = zj[0].get(GRB.DoubleAttr.X)
        val zyjOpt = zj[1].get(GRB.DoubleAttr.X)
        SuggestedControl(SpeedControl2D(zxiOpt, zyiOpt), SpeedControl2D(zxjOpt, zyjOpt))
    } else {
        println("Common QP: no solution found (status $status)")
        SuggestedControl(robot.control, other.control)
    }
} catch (ex: GRBException) {
    println("${ex.message} - Minimization problem is infeasible, return controls from local QP: ${robot.control} & ${other.control}.",)
    SuggestedControl(robot.control, other.control)
}
