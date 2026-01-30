package it.unibo.collektive.qp.controlFunctions

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.qp.dsl.GRBVector
import it.unibo.collektive.qp.dsl.ScalarVector
import it.unibo.collektive.qp.dsl.toTargetCLF
import it.unibo.collektive.qp.utils.Target

/**
 * GO-TO-TARGET CLF 2(p - p_g)^T u <= -c || p - p_g ||^2 + \delta
 */
fun GRBModel.goToTargetCLF(
    target: Target,
    position: ScalarVector,
    u: GRBVector,
    delta: GRBVar,
) {
    val pg: ScalarVector = doubleArrayOf(target.x, target.y)
    // convergence rate should vary based on deltaTime, if small, c should be smaller, if deltaTime ~ 1sec then in {0.5, 5}
    // if c is big, faster convergence
    // 2(p - p_g)^T u = 2(p_x - p_g,x) u_x + 2(p_y - p_g,y) u_y
    toTargetCLF(
        point = position,
        goalPoint = pg,
        u = u,
        convergenceRate = 1.0,
        delta = delta,
        name = "goToTargetCLF"
    )
    addConstr(delta, GRB.GREATER_EQUAL, 0.0, "slack") // bound delta >= 0
}
