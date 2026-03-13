package it.unibo.collektive.control.cbf

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.control.ControlFunctionContext
import it.unibo.collektive.solver.gurobi.GRBVector
import it.unibo.collektive.solver.gurobi.addSlackOrNull
import it.unibo.collektive.solver.gurobi.toQuadExpr
import kotlin.math.pow

/**
 * Max-speed constraint CBF;
 * enforces the upper velocity bound as a quadratic constraint.
 *
 * The inequality enforced is:
 * `||u_i,k||^2 <= u_{max}^2`
 *
 * @property eta an unused tuning parameter for this specific CBF (kept for interface compatibility).
 * @property slackWeight the penalty weight applied to the slack variable (if present).
 */
class MaxSpeedCBF(override val eta: Double = 1.0, override val slackWeight: Double? = null) : CBF() {
    override val name: String = "max_speed"

    override fun GRBModel.applyCBF(uSelf: GRBVector, uOther: GRBVector?, context: ControlFunctionContext): GRBVar? {
        val lhs = uSelf.toQuadExpr()
        val slack: GRBVar? = addSlackOrNull(this@MaxSpeedCBF, lhs)
        addQConstr(
            lhs,
            GRB.LESS_EQUAL,
            context.self.maxSpeed.pow(2),
            "u_norm",
        )
        return slack
    }
}
