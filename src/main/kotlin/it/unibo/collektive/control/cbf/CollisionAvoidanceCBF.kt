package it.unibo.collektive.control.cbf

import it.unibo.collektive.control.ControlFunction
import it.unibo.collektive.control.ControlFunctionContext
import it.unibo.collektive.mathutils.minus
import it.unibo.collektive.mathutils.squaredNorm
import it.unibo.collektive.mathutils.times
import it.unibo.collektive.mathutils.toDoubleArray
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
class CollisionAvoidanceCBF(override val eta: Double = 0.5, override val slackWeight: Double? = null) :
    PairwiseLinearBarrierCBF() {

    override val name: String = "collision_avoidance"

    override val installConstraintName: String = "collision_avoidance_CBF"

    override fun buildUpdate(
        controlFunction: ControlFunction,
        context: ControlFunctionContext,
    ): LinearConstraintUpdate {
        val current = (controlFunction as? CollisionAvoidanceCBF) ?: this
        val other = otherRobot(context)
        val distance = (context.self.position - other.position).toDoubleArray()
        val minDistance = max(context.self.safeMargin, other.safeMargin)
        val h = distance.squaredNorm() - minDistance.pow(2)
        return LinearConstraintUpdate(
            rhs = -(current.eta / context.settings.deltaTime) * h,
            selfCoefficients = 2.0 * distance,
            otherCoefficients = -2.0 * distance,
        )
    }
}
