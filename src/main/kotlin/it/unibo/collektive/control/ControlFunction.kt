package it.unibo.collektive.control

import com.gurobi.gurobi.GRBExpr
import com.gurobi.gurobi.GRBLinExpr
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBQuadExpr
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.model.Robot
import it.unibo.collektive.solver.gurobi.GRBVector
import it.unibo.collektive.solver.gurobi.QpSettings

/**
 * Represents a generic control function (e.g., a CLF or a CBF) to be enforced within an optimization problem.
 * It defines the common attributes and methods needed to inject mathematical constraints into the QP model.
 */
interface ControlFunction {
    /**
     * The unique name or identifier of the control function, often used for naming constraints in the solver.
     */
    val name: String

    /**
     * The penalty weight applied to the slack variable associated with this control function.
     * If null, the constraint is treated as hard (no slack),
     * or a default weight is used depending on the implementation.
     */
    val slackWeight: Double?

    /**
     * Applies the constraint to the [model] and returns the generated slack variable (if any).
     * The constraint should be formulated in terms of the control variables [uSelf] and optionally [uOther],
     * depending on whether it's a single-robot or pairwise constraint.
     * The [context] provides access to the robot states and settings needed to formulate the constraint.
     */
    fun applyToModel(model: GRBModel, uSelf: GRBVector, uOther: GRBVector?, context: ControlFunctionContext): GRBVar?

    /**
     * Adds the contribution of the slack variable to the [objective] function,
     * weighted by [slackWeight] or a default value from [context].
     */
    fun addSlackToObjective(objective: GRBExpr, slack: GRBVar, context: ControlFunctionContext) {
        val weight = slackWeight ?: context.settings.rhoSlack
        when (objective) {
            is GRBLinExpr -> objective.addTerm(weight, slack)
            is GRBQuadExpr -> objective.addTerm(weight, slack)
            else -> error("Cannot add slack to objective of type ${objective::class.simpleName}")
        }
    }
}

/**
 * Context containing all the necessary states and configurations to apply a [ControlFunction] to a robot.
 *
 * @property self the primary [Robot] applying the control function.
 * @property otherRobot an optional secondary [Robot] involved in pairwise constraints (e.g., collision avoidance).
 * @property settings the tuning parameters and simulation settings for the ADMM QP solver.
 */
data class ControlFunctionContext(
    val self: Robot,
    val otherRobot: Robot? = null,
    val settings: QpSettings = QpSettings(),
)
