package it.unibo.collektive.control.cbf

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.control.ControlFunctionContext
import it.unibo.collektive.model.Obstacle
import it.unibo.collektive.mathutils.minus
import it.unibo.collektive.mathutils.squaredNorm
import it.unibo.collektive.mathutils.toDoubleArray
import it.unibo.collektive.solver.gurobi.ConstraintNames
import it.unibo.collektive.solver.gurobi.GRBVector
import it.unibo.collektive.solver.gurobi.addSlackOrNull
import it.unibo.collektive.solver.gurobi.toLinExpr
import kotlin.math.pow

/**
 * Obstacle-avoidance barrier under ZOH dynamics;
 * adds a keep-out CBF against a static obstacle.
 */
class ObstacleAvoidanceCBF(val obstacle: Obstacle, override val eta: Double = 0.5, override val slackWeight: Double? = null) : CBF() {
    override val name: String = "obstacle_avoidance"

    override fun GRBModel.applyCBF(uSelf: GRBVector, uOther: GRBVector?, context: ControlFunctionContext): GRBVar? {
        val distance = (context.self.position - obstacle).toDoubleArray()
        val safeDistance = obstacle.radius + obstacle.margin
        val h = distance.squaredNorm() - safeDistance.pow(2)
        val dt = context.settings.deltaTime
        // RHS: -(\eta / \Delta t) * h_{i,k}^{obs}
        val rhs = -(eta / dt) * h
        // LHS: 2 * (p_i - p_o)^T * u_i
        val lhs = uSelf.toLinExpr(distance, 2.0)
        val slack: GRBVar? = addSlackOrNull(this@ObstacleAvoidanceCBF, lhs)
        addConstr(lhs, GRB.GREATER_EQUAL, rhs, ConstraintNames.obstacle("local"))
        return slack
    }
}
