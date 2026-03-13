package it.unibo.collektive.control.cbf

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBLinExpr
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.control.ControlFunctionContext
import it.unibo.collektive.mathutils.minus
import it.unibo.collektive.mathutils.squaredNorm
import it.unibo.collektive.mathutils.toDoubleArray
import it.unibo.collektive.solver.gurobi.ConstraintNames
import it.unibo.collektive.solver.gurobi.GRBVector
import it.unibo.collektive.solver.gurobi.addSlackOrNull
import kotlin.math.max
import kotlin.math.pow

/**
 * Robot–robot collision avoidance barrier;
 * enforces separation from the context's other robot under ZOH dynamics.
 *
 * The exact discrete-time inequality enforced is:
 * `2(p_i,k - p_j,k)^T (u_i,k - u_j,k) >= -(\eta / \Delta t) h_{ij,k}^col`
 * where `h_{ij,k}^col = ||p_i,k - p_j,k||^2 - d_{min}^2`.
 *
 * @property eta the tuning parameter governing the decay rate of the barrier constraint.
 * @property slackWeight the penalty weight applied to the slack variable (if present).
 */
class CollisionAvoidanceCBF(override val eta: Double = 0.5, override val slackWeight: Double? = null) : CBF() {
    override val name: String = "collision_avoidance"

    override fun GRBModel.applyCBF(uSelf: GRBVector, uOther: GRBVector?, context: ControlFunctionContext): GRBVar? {
        check(context.otherRobot != null && uOther != null) {
            "Other robot must not be null to apply Collision Avoidance CBF"
        }
        val distance = (context.self.position - context.otherRobot.position).toDoubleArray()
        val maxDist = max(context.self.safeMargin, context.otherRobot.safeMargin)
        val h = distance.squaredNorm() - maxDist.pow(2)
        val dt = context.settings.deltaTime
        // RHS: -(\eta / \Delta t) * h_{ij,k}^{col}
        val rhs = -(eta / dt) * h
        // LHS: 2 * r_{ij}^T * (u_i - u_j)
        val lhs = GRBLinExpr()
        for (index in distance.indices) {
            lhs.addTerm(2.0 * distance[index], uSelf[index])
            lhs.addTerm(-2.0 * distance[index], uOther[index])
        }
        val slack: GRBVar? = addSlackOrNull(this@CollisionAvoidanceCBF, lhs)
        addConstr(
            lhs,
            GRB.GREATER_EQUAL,
            rhs,
            ConstraintNames.collision("${context.self.position}_${context.otherRobot.position}"),
        )
        return slack
    }
}
