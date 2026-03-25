package it.unibo.collektive.control.cbf

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBLinExpr
import com.gurobi.gurobi.GRBModel
import it.unibo.collektive.control.ControlFunction
import it.unibo.collektive.control.ControlFunctionContext
import it.unibo.collektive.mathutils.minus
import it.unibo.collektive.mathutils.squaredNorm
import it.unibo.collektive.mathutils.toDoubleArray
import it.unibo.collektive.model.Obstacle
import it.unibo.collektive.solver.gurobi.Constraint
import it.unibo.collektive.solver.gurobi.GRBVector
import kotlin.math.pow

/**
 * Static obstacle-avoidance barrier under ZOH dynamics.
 *
 * Discrete-time CBF constraint (installed once, updated every iteration):
 * ```
 * 2(p_i − p_o)ᵀ u_i + slack ≥ −(η/Δt) · h_obs
 * ```
 * where `h_obs = ‖p_i − p_o‖² − (r_o + d_o)²`.
 *
 * The obstacle position is fixed, so only the robot position `p_i` changes across iterations,
 * making the LHS coefficients and RHS straightforward to refresh via [GRBModel.chgCoeff].
 *
 * The [Constraint.update] also reads the obstacle from the current `cf` instance when
 * provided (allowing a different `ObstacleAvoidanceCBF` instance to be passed each step while
 * still reusing the same model structure).
 *
 * @property obstacle   the static obstacle to avoid
 * @property eta        decay-rate parameter
 * @property slackWeight penalty for the soft version; `null` → hard constraint
 */
class ObstacleAvoidanceCBF(
    val obstacle: Obstacle,
    override val eta: Double = 0.5,
    override val slackWeight: Double? = null,
) : CBF() {

    override val name: String = "obstacle_avoidance_CBF"

    override fun GRBModel.installCBF(uSelf: GRBVector, uOther: GRBVector?): Constraint {
        val slack = slackWeight?.let {
            addVar(0.0, GRB.INFINITY, 0.0, GRB.CONTINUOUS, "slack_$name")
        }
        val lhs = GRBLinExpr().apply {
            repeat(uSelf.dimensions) { i -> addTerm(0.0, uSelf[i]) }
            slack?.let { addTerm(1.0, it) }
        }
        val constr = addConstr(lhs, GRB.GREATER_EQUAL, 0.0, name)

        return object : Constraint {
            override val slack = slack
            override val slackWeight = this@ObstacleAvoidanceCBF.slackWeight

            override fun update(model: GRBModel, controlFunction: ControlFunction, context: ControlFunctionContext) {
                val obstacle =
                    (controlFunction as? ObstacleAvoidanceCBF)?.obstacle ?: this@ObstacleAvoidanceCBF.obstacle
                val distance = (context.self.position - obstacle).toDoubleArray()
                val h = distance.squaredNorm() - (obstacle.radius + obstacle.margin).pow(2)
                val rhs = -(eta / context.settings.deltaTime) * h
                constr.set(GRB.DoubleAttr.RHS, rhs)
                for (i in distance.indices) {
                    model.chgCoeff(constr, uSelf[i], 2.0 * distance[i])
                }
            }
        }
    }
}
