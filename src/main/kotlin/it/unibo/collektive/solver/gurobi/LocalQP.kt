package it.unibo.collektive.solver.gurobi

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBEnv
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBQuadExpr
import it.unibo.collektive.admm.DualParams
import it.unibo.collektive.control.ControlFunction
import it.unibo.collektive.control.ControlFunctionContext
import it.unibo.collektive.control.cbf.CBF
import it.unibo.collektive.control.clf.CLF
import it.unibo.collektive.mathutils.minus
import it.unibo.collektive.mathutils.toDoubleArray
import it.unibo.collektive.model.Robot
import it.unibo.collektive.model.SpeedControl2D

/**
 * A persistent Gurobi model for a single agent's **local** ADMM sub-problem.
 *
 * [create] once (per robot, per simulation) to build the model structure:
 *    add the `u` decision variables, install all CLF/CBF constraints via [ControlFunction.install],
 *    and commit with [GRBModel.update].
 * [solve] on every ADMM iteration.  It only:
 *    - updates variable bounds,
 *    - calls [Constraint.update] on each constraint (RHS + coefficients),
 *    - rebuilds the **objective** expression (allocation is cheap; `addVar`/`addConstr` are not),
 *    - calls [GRBModel.update] once to flush pending changes, then [GRBModel.optimize].
 *
 * Each one owns its [GRBEnv]. Separate envs keep Gurobi's internal runtime fully
 * independent per model, avoiding any serialization that could occur when models share one env.
 * [close] disposes both the model and the env.
 *
 * Gurobi automatically uses the previous solve's solution as a warm start when [GRBModel.optimize]
 * is called again on the same model without an intervening [GRBModel.reset].
 * Does not call [GRBModel.reset], so every ADMM iteration after the first benefits from a warm start
 * at no extra cost.
 *
 * @param env        the Gurobi environment (owned — disposed in [close])
 * @param model      the persistent Gurobi model (owned — disposed in [close])
 * @param u          the `u` decision-variable vector
 * @param installed  installed constraint handles paired with the original CF instances
 * @param clfCount   number of CLF constraints (used to validate the `currentCLFs` list size)
 * @param cbfCount   number of CBF constraints (used to validate the `currentCBFs` list size)
 */
class LocalQP(
    private val env: GRBEnv,
    private val model: GRBModel,
    val u: GRBVector,
    private val installed: List<Constraint>,
    private val clfCount: Int,
    private val cbfCount: Int,
) : AutoCloseable {

    /**
     * Solves the local QP with fresh robot state, duals, and CF configurations.
     *
     * @param robot       current robot state
     * @param uNominal    nominal control input for this step
     * @param duals       current per-neighbour dual variables
     * @param context     solver context (robot + settings)
     * @param currentCLFs **same-length** list of CLFs as at [create], but possibly carrying updated goals
     * @param currentCBFs **same-length** list of CBFs as at [create], but with updated configuration if any
     * @return the optimal `SpeedControl2D`, or [robot]'s previous control if the QP has no solution
     */
    fun <ID : Comparable<ID>> solve(
        robot: Robot,
        uNominal: DoubleArray,
        duals: Map<ID, DualParams>,
        context: ControlFunctionContext,
        currentCLFs: List<CLF>,
        currentCBFs: List<CBF>,
    ): SpeedControl2D {
        require(currentCLFs.size == clfCount && currentCBFs.size == cbfCount) {
            "CF list size changed since install — local template must be rebuilt. " +
                "Expected ($clfCount CLF, $cbfCount CBF), got (${currentCLFs.size}, ${currentCBFs.size})."
        }
        for (i in u.vars.indices) {
            u[i].set(GRB.DoubleAttr.LB, -robot.maxSpeed)
            u[i].set(GRB.DoubleAttr.UB,  robot.maxSpeed)
        }
        val currentCFs: List<ControlFunction> = currentCLFs + currentCBFs
        installed.zip(currentCFs).forEach { (ic, cf) -> ic.update(model, cf, context) }
        model.setObjective(buildObjective(uNominal, duals, context), GRB.MINIMIZE)
        model.update()
        model.optimize()
        return extractSolution(robot)
    }

    private fun buildObjective(
        uNominal: DoubleArray,
        duals: Map<*, DualParams>,
        context: ControlFunctionContext,
    ): GRBQuadExpr = GRBQuadExpr().apply {
        addRhoNorm2Sq(u, uNominal)
        installed.forEach { ic ->
            ic.slack?.let { slack ->
                val w = ic.slackWeight ?: context.settings.rhoSlack
                addTerm(w, slack, slack)
            }
        }
        duals.forEach { (_, value) ->
            val suggested = value.suggestedControl.zi.toDoubleArray()
            val residual  = value.incidentDuals.yi.toDoubleArray()
            addRhoNorm2Sq(u, suggested - residual, context.settings.rhoADMM / 2.0)
        }
    }

    private fun extractSolution(robot: Robot): SpeedControl2D {
        val status = model.get(GRB.IntAttr.Status)
        if (status == GRB.INFEASIBLE) {
            model.writeIIS("localModel.ilp")
            for (c in model.constrs) {
                if (c.get(GRB.IntAttr.IISConstr) == 1) {
                    println("Local IIS constraint: ${c.get(GRB.StringAttr.ConstrName)}")
                }
            }
        }
        return if (model.get(GRB.IntAttr.SolCount) > 0) {
            SpeedControl2D(u[0].get(GRB.DoubleAttr.X), u[1].get(GRB.DoubleAttr.X))
        } else {
            println("Local QP: no solution found (status $status), returning previous control.")
            robot.control
        }
    }

    override fun close() {
        model.dispose()
        env.dispose()
    }
    companion object {
        /**
         * Creates and commits the model structure for a robot's local QP.
         *
         * This is the **only** place where [GRBModel.addVar] and [GRBModel.addConstr] are called for
         * this sub-problem.  After [create] returns, the topology is frozen.
         */
        fun create(
            robot: Robot,
            localCLFs: List<CLF>,
            localCBFs: List<CBF>,
            settings: QpSettings,
        ): LocalQP {
            setLicense()
            val env = GRBEnv(true).also {
                it.set(GRB.IntParam.OutputFlag, if (settings.logEnabled) 1 else 0)
                it.start()
            }
            val model = GRBModel(env).also { if (settings.logEnabled) it.setupLogger() }
            val u = model.addVecVar(robot.position.dimension, -robot.maxSpeed, robot.maxSpeed, "u")
            val installed = mutableListOf<Constraint>()
            localCLFs.forEach { clf -> installed += clf.install(model, u, null) }
            localCBFs.forEach { cbf -> installed += cbf.install(model, u, null) }
            model.update()
            return LocalQP(
                env       = env,
                model     = model,
                u         = u,
                installed = installed,
                clfCount  = localCLFs.size,
                cbfCount  = localCBFs.size,
            )
        }
    }
}
