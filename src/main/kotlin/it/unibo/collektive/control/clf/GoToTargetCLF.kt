package it.unibo.collektive.control.clf

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBLinExpr
import com.gurobi.gurobi.GRBModel
import it.unibo.collektive.mathutils.minus
import it.unibo.collektive.mathutils.squaredNorm
import it.unibo.collektive.mathutils.toDoubleArray
import it.unibo.collektive.model.Robot
import it.unibo.collektive.model.Target
import it.unibo.collektive.solver.gurobi.Constraint
import it.unibo.collektive.solver.gurobi.GRBVector
import it.unibo.collektive.solver.gurobi.QpSettings
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
 * moves. The target reference is captured once when the model is installed, so target motion is
 * still picked up as long as the same [target] object is updated over time.
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

            override fun update(
                model: GRBModel,
                self: Robot,
                otherRobot: Robot?,
                settings: QpSettings,
                deltaTime: Double,
            ) {
                val dist = (self.position - target.position).toDoubleArray()
                val rhs = -convergenceRate * dist.squaredNorm() - deltaTime.pow(2) * self.maxSpeed.pow(2)
                constr.set(GRB.DoubleAttr.RHS, rhs)
                for (i in dist.indices) {
                    model.chgCoeff(constr, uSelf[i], 2.0 * deltaTime * dist[i])
                }
            }
        }
    }
}
