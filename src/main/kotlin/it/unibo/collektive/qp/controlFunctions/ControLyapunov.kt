package it.unibo.collektive.qp.controlFunctions

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBVar
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
fun GRBModel.goToTargetCLF(target: Target, position: DoubleArray, u: GRBVector, delta: GRBVar) {
    val pg: DoubleArray = doubleArrayOf(target.x, target.y)
    // convergence rate should vary based on deltaTime, if small, c should be smaller,
    // if deltaTime ~ 1sec then in {0.5, 5}
    // if c is big, faster convergence
    // 2(p - p_g)^T u = 2(p_x - p_g,x) u_x + 2(p_y - p_g,y) u_y
    toTargetCLF(
        currentPos = position,
        goalPos = pg,
        u = u,
        convergenceRate = 1.0,
        delta = delta,
        name = "goToTargetCLF",
    )
    addConstr(delta, GRB.GREATER_EQUAL, 0.0, "slack") // bound delta >= 0
}

/**
 * objective: min ||u - u_nom||^2 + phi * delta^2 .
 * objective is quadratic in (ux, uy), linear in delta (slack)
 * ||u - u^nom||^2 + \phi \delta^2
 * ||u - u^nom||^2 = (ux - ux^nom)^2 + (uy - uy^nom)^2
 * [robot] [target] [u] [delta]
 */
fun GRBModel.minimizeNominal(target: Target, robot: Robot, u: GRBVector, delta: GRBVar): Unit = TODO(
    "minimizeDeviation(u = u, uNominal = (target.position - robot.position).toDoubleArray(), delta = delta, phi = 2.0)",
)
