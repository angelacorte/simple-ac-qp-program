package it.unibo.collektive.solver.gurobi

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBEnv
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBQuadExpr
import it.unibo.collektive.admm.IncidentDuals
import it.unibo.collektive.admm.SuggestedControl
import it.unibo.collektive.control.ControlFunction
import it.unibo.collektive.control.ControlFunctionContext
import it.unibo.collektive.control.cbf.CBF
import it.unibo.collektive.mathutils.plus
import it.unibo.collektive.mathutils.toDoubleArray
import it.unibo.collektive.model.Robot
import it.unibo.collektive.model.SpeedControl2D

/**
 * A persistent Gurobi model for the **pairwise** ADMM sub-problem between two robots.
 * [create] once per directed edge `(selfId, neighborId)` to build the model:
 *    add `z_i` and `z_j` decision variables, install pairwise CBF constraints, commit.
 *  [solve] every ADMM iteration.  Only variable bounds, constraint parameters,
 *    and the objective are refreshed — no structural changes.
 * Each one owns its [GRBEnv]. Separate envs keep Gurobi's internal runtime fully
 * independent per model, avoiding any serialization that could occur when models share one env.
 * [close] disposes both the model and the env.
 *
 * Does not call [GRBModel.reset] between solves, so Gurobi automatically warm-starts each
 * [GRBModel.optimize] call from the previous solution.
 *
 * @param env       owned Gurobi environment (disposed in [close])
 * @param model     owned persistent Gurobi model (disposed in [close])
 * @param zi        `z_ij^i` decision-variable vector for this agent
 * @param zj        `z_ij^j` decision-variable vector for the neighbor
 * @param installed installed CBF constraint handles
 */
class PairwiseQP private constructor(
    private val env: GRBEnv,
    private val model: GRBModel,
    val zi: GRBVector,
    val zj: GRBVector,
    private val installed: List<Constraint>,
    private val cbfCount: Int,
) : AutoCloseable {

    /**
     * Solves the pairwise QP for the current robot states and dual variables.
     *
     * @param robot        current state of this agent
     * @param other        current state of the neighbour
     * @param incidentDuals current dual variables for this edge
     * @param context       solver context (robot pair + settings)
     * @param currentCBFs   **same-length** CBF list as at [create]
     * @return optimal suggested controls `(z_i, z_j)`, or the robots' current controls on failure
     */
    fun solve(
        robot: Robot,
        other: Robot,
        incidentDuals: IncidentDuals,
        context: ControlFunctionContext,
        currentCBFs: List<CBF>,
    ): SuggestedControl {
        require(currentCBFs.size == cbfCount) {
            "CBF list size changed since install — pairwise template must be rebuilt. " +
                "Expected $cbfCount, got ${currentCBFs.size}."
        }
        for (i in zi.vars.indices) {
            zi[i].set(GRB.DoubleAttr.LB, -robot.maxSpeed)
            zi[i].set(GRB.DoubleAttr.UB, robot.maxSpeed)
        }
        for (i in zj.vars.indices) {
            zj[i].set(GRB.DoubleAttr.LB, -other.maxSpeed)
            zj[i].set(GRB.DoubleAttr.UB, other.maxSpeed)
        }
        val currentCFs: List<ControlFunction> = currentCBFs
        installed.zip(currentCFs).forEach { (ic, cf) -> ic.update(model, cf, context) }
        model.setObjective(buildObjective(robot, other, incidentDuals, context), GRB.MINIMIZE)
        model.update()
        model.optimize()
        return extractSolution(robot, other)
    }

    private fun buildObjective(
        robot: Robot,
        other: Robot,
        incidentDuals: IncidentDuals,
        context: ControlFunctionContext,
    ): GRBQuadExpr = GRBQuadExpr().apply {
        val rho = context.settings.rhoADMM / 2.0
        // (ρ/2)‖z_i − (u_i + y_i)‖²  +  (ρ/2)‖z_j − (u_j + y_j)‖²
        addRhoNorm2Sq(zi, (robot.control + incidentDuals.yi).toDoubleArray(), rho)
        addRhoNorm2Sq(zj, (other.control + incidentDuals.yj).toDoubleArray(), rho)
        installed.forEach { ic ->
            ic.slack?.let { slack ->
                val w = ic.slackWeight ?: context.settings.rhoSlack
                addTerm(w, slack, slack)
            }
        }
    }

    private fun extractSolution(robot: Robot, other: Robot): SuggestedControl {
        val status = model.get(GRB.IntAttr.Status)
        if (status == GRB.INFEASIBLE) {
            model.writeIIS("commonModel.ilp")
            for (c in model.constrs) {
                if (c.get(GRB.IntAttr.IISConstr) == 1) {
                    println("Pairwise IIS constraint: ${c.get(GRB.StringAttr.ConstrName)}")
                }
            }
        }
        return if (model.get(GRB.IntAttr.SolCount) > 0) {
            SuggestedControl(
                SpeedControl2D(zi[0].get(GRB.DoubleAttr.X), zi[1].get(GRB.DoubleAttr.X)),
                SpeedControl2D(zj[0].get(GRB.DoubleAttr.X), zj[1].get(GRB.DoubleAttr.X)),
            )
        } else {
            println("Pairwise QP: no solution found (status $status), returning current controls.")
            SuggestedControl(robot.control, other.control)
        }
    }

    override fun close() {
        model.dispose()
        env.dispose()
    }

    companion object {
        /**
         * Creates and commits the model structure for a directed edge `(self, neighbor)`.
         *
         * This is the **only** place where [GRBModel.addVar] and [GRBModel.addConstr] are called for
         * this sub-problem.
         */
        fun create(robot: Robot, other: Robot, pairwiseCBFs: List<CBF>, settings: QpSettings): PairwiseQP {
            setLicense()
            val env = GRBEnv(true).also {
                it.set(GRB.IntParam.OutputFlag, if (settings.logEnabled) 1 else 0)
                it.start()
            }
            val model = GRBModel(env).also { if (settings.logEnabled) it.setupLogger() }
            val zi = model.addVecVar(robot.position.dimension, -robot.maxSpeed, robot.maxSpeed, "z_ij^i")
            val zj = model.addVecVar(other.position.dimension, -other.maxSpeed, other.maxSpeed, "z_ij^j")
            val installed = mutableListOf<Constraint>()
            pairwiseCBFs.forEach { cbf -> installed += cbf.install(model, zi, zj) }
            model.update()
            return PairwiseQP(
                env = env,
                model = model,
                zi = zi,
                zj = zj,
                installed = installed,
                cbfCount = pairwiseCBFs.size,
            )
        }
    }
}
