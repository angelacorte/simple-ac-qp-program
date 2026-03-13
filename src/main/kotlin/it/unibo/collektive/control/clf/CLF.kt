package it.unibo.collektive.control.clf

import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.control.ControlFunction
import it.unibo.collektive.control.ControlFunctionContext
import it.unibo.collektive.solver.gurobi.GRBVector

abstract class CLF : ControlFunction {
    override val name: String = "CLF"

    abstract val convergenceRate: Double

    override fun applyToModel(
        model: GRBModel,
        uSelf: GRBVector,
        uOther: GRBVector?,
        context: ControlFunctionContext,
    ): GRBVar? = model.applyCLF(uSelf, context)

    abstract fun GRBModel.applyCLF(uSelf: GRBVector, context: ControlFunctionContext): GRBVar?
}
