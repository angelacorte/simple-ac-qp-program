package it.unibo.collektive.control.cbf

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.solver.gurobi.GRBVector
import it.unibo.collektive.solver.gurobi.QpSettings
import it.unibo.collektive.solver.gurobi.createAndAddSlack
import it.unibo.collektive.solver.gurobi.toQuadExpr
import kotlin.math.pow

/**
 * Max-speed constraint CBF;
 * enforces `||u||^2 <= maxSpeed^2` as a quadratic constraint.
 * When [withSlack] is true, adds a slack variable to relax the constraint.
 *
 * @param withSlack whether to add a slack variable to relax the constraint.
 * @param slackWeight penalty weight for the slack variable (default: 0.0)
 */
object MaxSpeedCBF : CBF {
    override val name: String = "max_speed"

    override fun GRBModel.applyCBF(uSelf: GRBVector, uOther: GRBVector?, context: CFContext) {
        addQConstr(
            uSelf.toQuadExpr(),
            GRB.LESS_EQUAL,
            context.self.maxSpeed.pow(2),
            "u_norm",
        )
    }
}
