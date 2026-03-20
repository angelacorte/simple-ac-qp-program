package it.unibo.collektive.control.cbf

import it.unibo.collektive.control.ControlFunction
import it.unibo.collektive.control.ControlFunctionContext
import it.unibo.collektive.mathutils.minus
import it.unibo.collektive.mathutils.squaredNorm
import it.unibo.collektive.mathutils.times
import it.unibo.collektive.mathutils.toDoubleArray
import kotlin.math.pow

/**
 * Communication-range barrier: keeps two robots within [range] of each other.
 *
 * Robustified discrete-time CBF under ZOH dynamics (installed once, updated every iteration):
 * ```
 * −2(p_i − p_j)ᵀ(u_i − u_j) + slack ≥ −(η/Δt)·h_com + 4Δt·u_max²
 * ```
 * where `h_com = R² − ‖p_i − p_j‖²`.
 *
 * Both the LHS coefficients and the RHS change with robot positions and Δt.
 * [GRBModel.chgCoeff] updates them without structural changes.
 *
 * @property range      maximum allowed distance between the two robots
 * @property eta        decay-rate parameter
 * @property slackWeight penalty for the soft version; `null` → hard constraint
 */
class CommunicationRangeCBF(
    val range: Double,
    override val eta: Double = 0.5,
    override val slackWeight: Double? = null,
) : PairwiseLinearBarrierCBF() {

    override val name: String = "communication_range"

    override val installConstraintName: String = "communication_range_CBF"

    override fun buildUpdate(
        controlFunction: ControlFunction,
        context: ControlFunctionContext,
    ): LinearConstraintUpdate {
        val current = (controlFunction as? CommunicationRangeCBF) ?: this
        val other = otherRobot(context)
        val distance = (context.self.position - other.position).toDoubleArray()
        val h = current.range.pow(2) - distance.squaredNorm()
        val delta = context.settings.deltaTime
        return LinearConstraintUpdate(
            rhs = -(current.eta / delta) * h + delta * (context.self.maxSpeed + other.maxSpeed).pow(2),
            selfCoefficients = -2.0 * distance,
            otherCoefficients = 2.0 * distance,
        )
    }
}
