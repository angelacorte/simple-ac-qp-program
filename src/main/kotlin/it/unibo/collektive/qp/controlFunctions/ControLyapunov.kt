package it.unibo.collektive.qp.controlFunctions

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.qp.config.QpSettings
import it.unibo.collektive.qp.dsl.ConstraintNames
import it.unibo.collektive.qp.dsl.GRBVector
import it.unibo.collektive.qp.dsl.toTargetCLF
import it.unibo.collektive.qp.utils.Robot
import it.unibo.collektive.qp.utils.Target

/**
 * GO-TO-TARGET CLF 2(p - p_g)^T u <= -c || p - p_g ||^2 + \delta
 * Where p_g is the [target] where we want to go, p is the current [position] for the device,
 * [u] is the decision variable for the optimization problem, and [delta] the slack variable.
 * [target] [position] [u] [delta]
 */
fun GRBModel.goToTargetCLF(target: Target, position: DoubleArray, u: GRBVector, delta: GRBVar, settings: QpSettings = QpSettings()) {
    val pg: DoubleArray = doubleArrayOf(target.x, target.y)
    // convergence rate should vary based on deltaTime, if small, c should be smaller,
    // if deltaTime ~ 1sec then in {0.5, 5}
    // if c is big, faster convergence
    // 2(p - p_g)^T u = 2(p_x - p_g,x) u_x + 2(p_y - p_g,y) u_y
    toTargetCLF(
        currentPos = position,
        goalPos = pg,
        u = u,
        convergenceRate = settings.convergenceRate,
        delta = delta,
        name = ConstraintNames.clf(target.id.toString()),
    )
    addConstr(delta, GRB.GREATER_EQUAL, 0.0, ConstraintNames.slack(target.id.toString())) // bound delta >= 0
}
