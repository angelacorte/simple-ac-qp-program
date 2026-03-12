package it.unibo.collektive.control.cbf

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBException
import com.gurobi.gurobi.GRBLinExpr
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.model.minus
import it.unibo.collektive.model.squaredNorm
import it.unibo.collektive.model.toDoubleArray
import it.unibo.collektive.solver.gurobi.ConstraintNames
import it.unibo.collektive.solver.gurobi.GRBVector
import it.unibo.collektive.solver.gurobi.QpSettings
import it.unibo.collektive.solver.gurobi.createAndAddSlack
import kotlin.math.max
import kotlin.math.pow

/**
 * Communication-range barrier;
 * enforces max distance [range] between two robots.
 * When [withSlack] is `true`, creates a slack variable to soften the constraint;
 * otherwise the constraint is hard.
 *
 * @param range the maximum allowed communication distance (ignored when `null`).
 * @param withSlack whether to add a slack variable to relax the constraint.
 * @param slackWeight penalty weight for the slack variable (default: 0.0)
 */
class CommunicationRangeCBF(
    private val range: Double,
    override val slackWeight: Double? = null,
) : CBF {
    override val name: String = "comm_range"
    override var slack: GRBVar? = null

    override fun GRBModel.applyCBF(uSelf: GRBVector, uOther: GRBVector?, context: CFContext) {
        check(uOther != null && context.otherRobot != null) {
            "Cannot apply CBF with other robot as null value, $uOther"
        }
        // COMM DISTANCE -2(p1 - p2)^T (u1 -u2) + \gamma [ R^2 - ||p1 - p2||^2 ] >= 0
        // COMM DISTANCE -2(p1 - p2)^T (u1 -u2) >= - \gamma [ R^2 - ||p1 - p2||^2 ]
        val distance = (context.self.position - context.otherRobot.position).toDoubleArray()
        val communication = GRBLinExpr()
        val uMax = max(context.self.maxSpeed, context.otherRobot.maxSpeed)
        val commRight = -(context.settings.gammaComm / context.settings.deltaTime) * (range.pow(2) - distance.squaredNorm()) + context.settings.deltaTime * uMax.pow(2)
        for (index in 0 until distance.size) {
            communication.addTerm(-2.0 * distance[index], uSelf[index])
            communication.addTerm(2.0 * distance[index], uOther[index])
        }
        slack?.let { communication.addTerm(context.settings.rhoCommSlack, it) }
        try {
            addConstr(communication, GRB.GREATER_EQUAL, commRight, name)
        } catch (e: GRBException) {
            println("Error for communication range CBF: ${e.message}")
        }
    }
}
