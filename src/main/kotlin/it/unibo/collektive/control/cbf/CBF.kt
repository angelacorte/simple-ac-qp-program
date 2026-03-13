package it.unibo.collektive.control.cbf

import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.control.ControlFunction
import it.unibo.collektive.control.ControlFunctionContext
import it.unibo.collektive.solver.gurobi.GRBVector

/**
 * Pluggable barrier builder identified by [name] and exposing [add] to inject constraints.
 * Each CBF may hold a [slack] variable created during [add] and a [slackWeight] for the objective penalty.
 */
abstract class CBF : ControlFunction {
    override val name: String get() = "CBF"
    abstract val eta: Double

    abstract fun GRBModel.applyCBF(uSelf: GRBVector, uOther: GRBVector?, context: ControlFunctionContext): GRBVar?

    override fun applyToModel(
        model: GRBModel,
        uSelf: GRBVector,
        uOther: GRBVector?,
        context: ControlFunctionContext,
    ): GRBVar? = model.applyCBF(uSelf, uOther, context)
}
