package it.unibo.collektive.control.cbf

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBException
import com.gurobi.gurobi.GRBLinExpr
import com.gurobi.gurobi.GRBModel
import it.unibo.collektive.model.minus
import it.unibo.collektive.model.squaredNorm
import it.unibo.collektive.model.toDoubleArray
import it.unibo.collektive.solver.gurobi.ConstraintNames
import it.unibo.collektive.solver.gurobi.GRBVector
import kotlin.math.max
import kotlin.math.pow

/**
 * Robot–robot collision avoidance barrier;
 * enforces separation from [CFContext.other].
 * When [withSlack] is `true`, creates a slack variable to soften the constraint;
 * otherwise the constraint is hard.
 *
 * @param withSlack whether to add a slack variable to relax the constraint.
 * @param slackWeight penalty weight for the slack variable (default: 0.0)
 */
object CollisionAvoidanceCBF : CBF {
    override val name: String = "collision_avoidance"

    override fun GRBModel.applyCBF(uSelf: GRBVector, uOther: GRBVector?, context: CFContext) {
        check(context.otherRobot != null && uOther != null) {
            "Other robot must not be null"
        }
        // COLLISION AVOIDANCE 2(p1 - p2)^T (u1 - u2) + \gamma [ ||p1-p2||^2 - dmin^2 ] >= 0
        // 2(p1 - p2)^T (u1 - u2) >= - \gamma [ ||p1-p2||^2 - dmin^2 ]
        val distance = (context.self.position - context.otherRobot.position).toDoubleArray()
        val maxDist = max(context.self.safeMargin, context.otherRobot.safeMargin)
        val collision = GRBLinExpr()
        val collRight = -(context.settings.gammaCollision / context.settings.deltaTime) * (distance.squaredNorm() - maxDist.pow(2))
        for (index in 0 until distance.size) {
            collision.addTerm(2.0 * distance[index], uSelf[index])
            collision.addTerm(-2.0 * distance[index], uOther[index])
        }
        try {
            addConstr(collision, GRB.GREATER_EQUAL, collRight, ConstraintNames.collision("${context.self.position}_${context.otherRobot.position}"))
        } catch (e: GRBException) {
            println("Error for collision avoidance CBF: ${e.message}")
        }
    }
}
