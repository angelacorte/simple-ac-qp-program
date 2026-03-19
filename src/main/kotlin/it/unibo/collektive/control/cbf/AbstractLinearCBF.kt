package it.unibo.collektive.control.cbf

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBLinExpr
import com.gurobi.gurobi.GRBModel
import it.unibo.collektive.control.ControlFunction
import it.unibo.collektive.control.ControlFunctionContext
import it.unibo.collektive.solver.gurobi.Constraint
import it.unibo.collektive.solver.gurobi.ConstraintNames
import it.unibo.collektive.solver.gurobi.GRBVector

abstract class AbstractLinearCBF : CBF() {
    protected abstract val installConstraintName: String

    protected open val requiresOtherControl: Boolean = false

    protected abstract fun buildUpdate(
        controlFunction: ControlFunction,
        context: ControlFunctionContext,
    ): LinearConstraintUpdate

    override fun GRBModel.installCBF(uSelf: GRBVector, uOther: GRBVector?): Constraint {
        val otherControl = if (requiresOtherControl) {
            checkNotNull(uOther) { "${this@AbstractLinearCBF::class.simpleName} requires uOther (pairwise constraint)" }
        } else {
            uOther
        }
        val slack = slackWeight?.let {
            addVar(0.0, GRB.INFINITY, 0.0, GRB.CONTINUOUS, ConstraintNames.slack(name))
        }
        val lhs = GRBLinExpr().apply {
            repeat(uSelf.dimensions) { i ->
                addTerm(0.0, uSelf[i])
                otherControl?.let { addTerm(0.0, it[i]) }
            }
            slack?.let { addTerm(1.0, it) }
        }
        val constraint = addConstr(lhs, GRB.GREATER_EQUAL, 0.0, installConstraintName)

        return object : Constraint {
            override val slack = slack
            override val slackWeight = this@AbstractLinearCBF.slackWeight

            override fun update(model: GRBModel, controlFunction: ControlFunction, context: ControlFunctionContext) {
                val update = buildUpdate(controlFunction, context)
                constraint.set(GRB.DoubleAttr.RHS, update.rhs)
                for (i in update.selfCoefficients.indices) {
                    model.chgCoeff(constraint, uSelf[i], update.selfCoefficients[i])
                }
                update.otherCoefficients?.let { coefficients ->
                    val other = checkNotNull(otherControl)
                    for (i in coefficients.indices) {
                        model.chgCoeff(constraint, other[i], coefficients[i])
                    }
                }
            }
        }
    }
}

class LinearConstraintUpdate(
    val rhs: Double,
    val selfCoefficients: DoubleArray,
    val otherCoefficients: DoubleArray? = null,
)

abstract class PairwiseLinearBarrierCBF : AbstractLinearCBF() {
    final override val requiresOtherControl: Boolean = true

    protected fun otherRobot(context: ControlFunctionContext) = checkNotNull(context.otherRobot) {
        "${this::class.simpleName}.update: otherRobot must not be null"
    }
}
