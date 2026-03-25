package it.unibo.collektive.control.clf

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBLinExpr
import com.gurobi.gurobi.GRBModel
import it.unibo.collektive.control.ControlFunctionContext
import it.unibo.collektive.mathutils.minus
import it.unibo.collektive.mathutils.squaredNorm
import it.unibo.collektive.mathutils.toDoubleArray
import it.unibo.collektive.model.Target
import it.unibo.collektive.solver.gurobi.Constraint
import it.unibo.collektive.solver.gurobi.GRBVector
import kotlin.math.pow

/**
 * Discrete-time CLF constraint for goal-reaching under ZOH dynamics.
 *
 * Installed constraint (built once):
 * ```
 * 2Δt·(p_i − p_g)ᵀ·u − slack  ≤  −λ·‖p_i − p_g‖² − Δt²·u_max²
 * ```
 *
 * Both the LHS coefficients `2Δt·(p_i − p_g)[k]` and the RHS change every iteration as the robot
 * moves.  **The target position `p_g` may also change** (e.g. via a `MoveTarget` global reaction):
 * the [Constraint.update] method reads the **current** `GoToTargetCLF` instance passed as
 * `cf`, so an updated target is picked up automatically without any model rebuild.
 *
 * The slack variable is mandatory for CLF feasibility and is always created (regardless of
 * [slackWeight]).
 *
 * @property target          navigation goal
 * @property convergenceRate Lyapunov decrease rate λ
 * @property slackWeight     objective penalty for the slack variable (default 1.0)
 */
class GoToTargetCLF(
    val target: Target,
    override val convergenceRate: Double = 1.0,
    override val slackWeight: Double? = 1.0,
) : CLF() {

    override val name: String = "go_to_target"

    override fun GRBModel.installCLF(uSelf: GRBVector): Constraint {
        val dim = uSelf.dimensions
        val slack = addVar(0.0, GRB.INFINITY, 0.0, GRB.CONTINUOUS, "slack_$name")
        val lhs = GRBLinExpr().apply {
            repeat(dim) { i -> addTerm(0.0, uSelf[i]) }
            addTerm(-1.0, slack)
        }
        val constr = addConstr(lhs, GRB.LESS_EQUAL, 0.0, "go_to_target${target.id}_CLF")

        return object : Constraint {
            override val slack = slack
            override val slackWeight = this@GoToTargetCLF.slackWeight

            override fun update(model: GRBModel, context: ControlFunctionContext) {
                require(context.settings.deltaTime > 0.0 && context.settings.deltaTime.isFinite()) {
                    "deltaTime must be finite and positive for GoToTargetCLF"
                }
                val dist = (context.self.position - target.position).toDoubleArray()
                val delta = context.settings.deltaTime
                val rhs = -convergenceRate * dist.squaredNorm() - delta.pow(2) * context.self.maxSpeed.pow(2)
                constr.set(GRB.DoubleAttr.RHS, rhs)
                for (i in dist.indices) {
                    model.chgCoeff(constr, uSelf[i], 2.0 * delta * dist[i])
                }
            }
        }
    }
}
