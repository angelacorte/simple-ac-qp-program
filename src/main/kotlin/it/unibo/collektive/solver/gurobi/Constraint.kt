package it.unibo.collektive.solver.gurobi

import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.control.ControlFunction
import it.unibo.collektive.control.ControlFunctionContext

/**
 * A handle to variables and constraints that have been structurally installed into a [GRBModel] **exactly once**.
 *
 * After [ControlFunction.install] returns this handle, the model topology is frozen.
 * Every ADMM iteration must only call [update] to refresh numerical parameters (RHS, linear coefficients,
 * variable bounds) and then call [GRBModel.update] once before [GRBModel.optimize].
 *
 * Gurobi's [GRBModel.addVar] / [GRBModel.addConstr] are comparatively expensive: they allocate
 * internal data structures and require a full model rebuild on every [GRBModel.update] call.
 * [GRBModel.chgCoeff] and attribute setters (RHS, bounds) are cheap in-place mutations that do
 * **not** trigger a structural rebuild, so they are the right tool for per-iteration updates.
 */
interface Constraint {

    /**
     * The slack decision variable introduced for this constraint,
     * or `null` if the constraint is enforced as hard (no slack).
     */
    val slack: GRBVar?

    /**
     * The objective penalty weight applied to [slack].
     * `null` means "use the solver's default ([QpSettings.rhoSlack])".
     */
    val slackWeight: Double?

    /**
     * Updates only the numerical parameters (RHS, linear coefficients) of the installed constraint
     * using the latest robot state in [context] and the current [controlFunction] instance.
     *
     * The [controlFunction] parameter allows CLFs whose goal positions can change at runtime (e.g. [GoToTargetCLF]
     * when a target moves) to propagate the updated position without rebuilding the constraint.
     * CBF implementations typically only need [context].
     *
     * **Must not** call [GRBModel.addVar], [GRBModel.addConstr], [GRBModel.addQConstr], or any other
     * structural-modification API. Structural additions at this stage corrupt the model.
     *
     * Call [GRBModel.update] once after all constraints have been updated, not inside this method.
     *
     * @param model the model whose coefficients/RHS are being mutated
     * @param controlFunction    the current (possibly updated) instance of the control function that owns this constraint
     * @param context current robot positions, speeds, and solver settings
     */
    fun update(model: GRBModel, controlFunction: ControlFunction, context: ControlFunctionContext)
}
