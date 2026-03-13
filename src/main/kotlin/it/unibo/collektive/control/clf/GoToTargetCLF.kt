package it.unibo.collektive.control.clf

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.control.ControlFunctionContext
import it.unibo.collektive.model.Target
import it.unibo.collektive.mathutils.minus
import it.unibo.collektive.mathutils.squaredNorm
import it.unibo.collektive.mathutils.toDoubleArray
import it.unibo.collektive.solver.gurobi.ConstraintNames
import it.unibo.collektive.solver.gurobi.GRBVector
import it.unibo.collektive.solver.gurobi.toLinExpr
import kotlin.math.pow

/**
 * Discrete-time CLF (DCLF) constraint for goal reaching under ZOH dynamics.
 * Enforces the affine sufficient constraint by incorporating the required slack variable.
 */
class GoToTargetCLF(
    val target: Target,
    override val convergenceRate: Double = 1.0,
    override val slackWeight: Double? = 1.0,
) : CLF() {
    override val name: String = "go_to_target"

    override fun GRBModel.applyCLF(uSelf: GRBVector, context: ControlFunctionContext): GRBVar? {
        require(context.settings.deltaTime.isFinite() && context.settings.deltaTime > 0.0) {
            "deltaTime must be finite and greater than zero to build DCLF constraint"
        }
        require(context.self.maxSpeed.isFinite() && context.self.maxSpeed >= 0.0) {
            "maxSpeed must be finite and non-negative"
        }
        val distanceVec = (context.self.position - target.position).toDoubleArray()
        val dt = context.settings.deltaTime
        // RHS: -\eta * ||e||^2 - \Delta t^2 * u_{max}^2
        val rhs = -convergenceRate * distanceVec.squaredNorm() - dt.pow(2) * context.self.maxSpeed.pow(2)
        // LHS: 2 * \Delta t * e^T * u
        val lhs = uSelf.toLinExpr(distanceVec, 2.0 * dt)
        val slack = addVar(0.0, GRB.INFINITY, 0.0, GRB.CONTINUOUS, ConstraintNames.slack(name))
        lhs.addTerm(-1.0, slack)
        addConstr(lhs, GRB.LESS_EQUAL, rhs, ConstraintNames.clf(target.id.toString()))
        return slack
    }
}
