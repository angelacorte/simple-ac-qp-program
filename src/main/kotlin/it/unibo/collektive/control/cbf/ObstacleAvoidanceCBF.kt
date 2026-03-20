package it.unibo.collektive.control.cbf

import it.unibo.collektive.control.ControlFunction
import it.unibo.collektive.control.ControlFunctionContext
import it.unibo.collektive.mathutils.minus
import it.unibo.collektive.mathutils.squaredNorm
import it.unibo.collektive.mathutils.times
import it.unibo.collektive.mathutils.toDoubleArray
import it.unibo.collektive.model.Obstacle
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
) : AbstractLinearCBF() {

    override val name: String = "obstacle_avoidance"

    override val installConstraintName: String = "obstacle_avoidance_CBF"

    override fun buildUpdate(
        controlFunction: ControlFunction,
        context: ControlFunctionContext,
    ): LinearConstraintUpdate {
        val current = (controlFunction as? ObstacleAvoidanceCBF) ?: this
        val distance = (context.self.position - current.obstacle).toDoubleArray()
        val h = distance.squaredNorm() - (current.obstacle.radius + current.obstacle.margin).pow(2)
        return LinearConstraintUpdate(
            rhs = -(current.eta / context.settings.deltaTime) * h,
            selfCoefficients = 2.0 * distance,
        )
    }
}
