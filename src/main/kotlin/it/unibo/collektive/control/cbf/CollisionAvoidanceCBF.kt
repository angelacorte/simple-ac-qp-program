package it.unibo.collektive.control.cbf

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBLinExpr
import com.gurobi.gurobi.GRBModel
import it.unibo.collektive.control.ControlFunction
import it.unibo.collektive.control.ControlFunctionContext
import it.unibo.collektive.mathutils.minus
import it.unibo.collektive.mathutils.squaredNorm
import it.unibo.collektive.mathutils.toDoubleArray
import it.unibo.collektive.solver.gurobi.Constraint
import it.unibo.collektive.solver.gurobi.GRBVector
import kotlin.math.max
import kotlin.math.pow

/**
 * Robot–robot collision avoidance barrier under ZOH dynamics.
 *
 * Discrete-time CBF constraint (installed once, updated every iteration):
 * ```
 * 2(p_i - p_j)ᵀ(u_i - u_j) + slack ≥ −(η/Δt) · h_col
 * ```
 * where `h_col = ‖p_i − p_j‖² − d_min²`.
 *
 * The LHS coefficients `2(p_i − p_j)[k]` and the RHS `−(η/Δt)·h_col` change every step as the
 * device move.  [GRBModel.chgCoeff] is used to update them in-place without rebuilding the model.
 *
 * @property eta        decay-rate parameter
 * @property slackWeight objective penalty for the slack variable; `null` → hard constraint (no slack)
 */
class CollisionAvoidanceCBF(override val eta: Double = 0.5, override val slackWeight: Double? = null) : CBF() {

    override val name: String = "collision_avoidance_CBF"

    override fun GRBModel.installCBF(uSelf: GRBVector, uOther: GRBVector?): Constraint {
        checkNotNull(uOther) { "CollisionAvoidanceCBF requires uOther (pairwise constraint)" }
        val dim = uSelf.dimensions
        val slack = slackWeight?.let {
            addVar(0.0, GRB.INFINITY, 0.0, GRB.CONTINUOUS, "slack_$name")
        }
        val lhs = GRBLinExpr().apply {
            repeat(dim) { i ->
                addTerm(0.0, uSelf[i])
                addTerm(0.0, uOther[i])
            }
            slack?.let { addTerm(1.0, it) }
        }
        val constr = addConstr(lhs, GRB.GREATER_EQUAL, 0.0, name)

        return object : Constraint {
            override val slack = slack
            override val slackWeight = this@CollisionAvoidanceCBF.slackWeight

            override fun update(model: GRBModel, controlFunction: ControlFunction, context: ControlFunctionContext) {
                checkNotNull(context.otherRobot) {
                    "CollisionAvoidanceCBF.update: otherRobot must not be null"
                }
                val distance = (context.self.position - context.otherRobot.position).toDoubleArray()
                val minDistance = max(context.self.safeMargin, context.otherRobot.safeMargin)
                val h = distance.squaredNorm() - minDistance.pow(2)
                val rhs = -(eta / context.settings.deltaTime) * h
                constr.set(GRB.DoubleAttr.RHS, rhs)
                for (i in distance.indices) {
                    model.chgCoeff(constr, uSelf[i], 2.0 * distance[i])
                    model.chgCoeff(constr, uOther[i], -2.0 * distance[i])
                }
            }
        }
    }
}
