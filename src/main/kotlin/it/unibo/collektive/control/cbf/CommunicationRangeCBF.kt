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
 * Communication-range barrier;
 * enforces a maximum connection distance [range] between two robots.
 * When [slackWeight] is provided, creates a slack variable to soften the constraint;
 * otherwise the constraint is hard.
 *
 * Implements the robustified discrete-time CBF (ZOH dynamics).
 */
class CommunicationRangeCBF(
    private val range: Double,
    override val eta: Double = 0.5,
    override val slackWeight: Double? = null,
) : CBF() {
    override val name: String = "communication_range"

    override fun GRBModel.applyCBF(uSelf: GRBVector, uOther: GRBVector?, context: ControlFunctionContext): GRBVar? {
        check(uOther != null && context.otherRobot != null) { "Cannot apply Comm CBF with null other robot" }
        val distance = (context.self.position - context.otherRobot.position).toDoubleArray()
        val h = range.pow(2) - distance.squaredNorm()
        val uMax = max(context.self.maxSpeed, context.otherRobot.maxSpeed)
        val dt = context.settings.deltaTime
        val rhs = -(eta / dt) * h + 4.0 * dt * uMax.pow(2)
        val lhs = GRBLinExpr()
        for (index in distance.indices) {
            lhs.addTerm(-2.0 * distance[index], uSelf[index])
            lhs.addTerm(2.0 * distance[index], uOther[index])
        }
        val slack: GRBVar? = addSlackOrNull(this@CommunicationRangeCBF, lhs)
        addConstr(lhs, GRB.GREATER_EQUAL, rhs, ConstraintNames.comm("${context.self.position}_${context.otherRobot.position}"))
        return slack
    }
}

