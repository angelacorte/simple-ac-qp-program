package it.unibo.collektive.solver.gurobi

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBQuadExpr
import it.unibo.collektive.admm.DualParams
import it.unibo.collektive.control.ControlFunctionContext
import it.unibo.collektive.control.cbf.CBF
import it.unibo.collektive.control.clf.CLF
import it.unibo.collektive.mathutils.minus
import it.unibo.collektive.mathutils.toDoubleArray
import it.unibo.collektive.model.Robot
import it.unibo.collektive.model.SpeedControl2D

class LocalQP private constructor(
    val model: GRBModel,
    val u: GRBVector,
    private val constraints: List<Constraint>,
) {

    fun <ID : Comparable<ID>> updateAndSolve(
        robot: Robot,
        uNominal: DoubleArray,
        duals: Map<ID, DualParams>,
        context: ControlFunctionContext,
    ): SpeedControl2D {
        for (i in u.variables.indices) {
            u[i].set(GRB.DoubleAttr.LB, -robot.maxSpeed)
            u[i].set(GRB.DoubleAttr.UB, robot.maxSpeed)
            u[i].set(GRB.DoubleAttr.Start, robot.control.toDoubleArray()[i]) // warm start
        }
        constraints.forEach { constraint -> constraint.update(model, context) }
        model.setObjective(buildObjective(uNominal, duals, context), GRB.MINIMIZE)
        model.update()
        model.optimize()
        return extractSolution(robot)
    }

    private fun buildObjective( // todo this should be taken from outside, can be different by different simulations
        uNominal: DoubleArray,
        duals: Map<*, DualParams>,
        context: ControlFunctionContext,
    ): GRBQuadExpr = GRBQuadExpr().apply {
        addRhoNorm2Sq(u, uNominal)
        constraints.forEach { constr ->
            constr.slack?.let { slack ->
                val weight = constr.slackWeight ?: context.settings.rhoSlack
                addTerm(weight, slack, slack)
            }
        }
        duals.forEach { (_, value) ->
            val suggested = value.suggestedControl.zi.toDoubleArray()
            val residual = value.incidentDuals.yi.toDoubleArray()
            addRhoNorm2Sq(u, suggested - residual, context.settings.rhoADMM / 2.0)
        }
    }

    private fun extractSolution(robot: Robot): SpeedControl2D {
        val status = model.get(GRB.IntAttr.Status)
        if (status == GRB.INFEASIBLE) {
            model.writeIIS("localModel.ilp")
            for (constr in model.constrs) {
                if (constr.get(GRB.IntAttr.IISConstr) == 1) {
                    println("Local IIS constraint: ${constr.get(GRB.StringAttr.ConstrName)}")
                }
            }
        }
        return when {
            model.get(GRB.IntAttr.SolCount) > 0 -> SpeedControl2D(u[0].get(GRB.DoubleAttr.X), u[1].get(GRB.DoubleAttr.X))
            else -> {
                println("Local QP: no solution found (status $status), returning previous control.")
                robot.control
            }
        }
    }

    companion object {
        fun create(model: GRBModel, robot: Robot, localCLFs: List<CLF>, localCBFs: List<CBF>): LocalQP {
            val u = model.addVecVar(robot.position.dimension, -robot.maxSpeed, robot.maxSpeed, "u")
            val installed = mutableListOf<Constraint>()
            localCLFs.forEach { clf -> installed += clf.install(model, u, null) }
            localCBFs.forEach { cbf -> installed += cbf.install(model, u, null) }
            model.update()
            return LocalQP(
                model = model,
                u = u,
                constraints = installed,
            )
        }
    }
}
