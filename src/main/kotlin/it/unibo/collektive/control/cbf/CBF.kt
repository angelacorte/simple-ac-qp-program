package it.unibo.collektive.control.cbf

import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.control.ControlFunction
import it.unibo.collektive.control.ControlFunctionContext
import it.unibo.collektive.solver.gurobi.GRBVector

/**
 * Pluggable barrier builder identified by [name] and exposing [applyToModel] to inject constraints.
 */
abstract class CBF : ControlFunction {
    override val name: String get() = "CBF"

    /**
     * The tuning parameter governing the decay rate or strictness of the barrier constraint.
     */
    abstract val eta: Double

    /**
     * Applies the specific Control Barrier Function constraint to the [this] GRBModel.
     *
     * @param uSelf the decision variables representing the control input of the local robot.
     * @param uOther the optional decision variables representing the control input of a neighboring robot.
     * @param context the context providing state variables like positions and solver settings.
     * @return the generated slack [GRBVar] if [slackWeight] is defined, or null otherwise.
     */
    abstract fun GRBModel.applyCBF(uSelf: GRBVector, uOther: GRBVector?, context: ControlFunctionContext): GRBVar?

    override fun applyToModel(
        model: GRBModel,
        uSelf: GRBVector,
        uOther: GRBVector?,
        context: ControlFunctionContext,
    ): GRBVar? = model.applyCBF(uSelf, uOther, context)
}
