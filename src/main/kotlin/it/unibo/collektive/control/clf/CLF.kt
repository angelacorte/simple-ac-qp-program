package it.unibo.collektive.control.clf

import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.control.ControlFunction
import it.unibo.collektive.control.ControlFunctionContext
import it.unibo.collektive.solver.gurobi.GRBVector

/**
 * Represents a Control Lyapunov Function (CLF) used to enforce task-related convergence,
 * such as navigating towards a specific target.
 */
abstract class CLF : ControlFunction {
    override val name: String = "CLF"

    /**
     * The rate at which the Lyapunov function is forced to decrease, dictating the convergence speed.
     */
    abstract val convergenceRate: Double

    override fun applyToModel(
        model: GRBModel,
        uSelf: GRBVector,
        uOther: GRBVector?,
        context: ControlFunctionContext,
    ): GRBVar? = model.applyCLF(uSelf, context)

    /**
     * Applies the specific Control Lyapunov Function constraint to the [this] GRBModel.
     *
     * @param uSelf the decision variables representing the control input of the local robot.
     * @param context the context providing state variables like the robot's position and solver settings.
     * @return the generated slack [GRBVar] to ensure feasibility.
     */
    abstract fun GRBModel.applyCLF(uSelf: GRBVector, context: ControlFunctionContext): GRBVar?
}
