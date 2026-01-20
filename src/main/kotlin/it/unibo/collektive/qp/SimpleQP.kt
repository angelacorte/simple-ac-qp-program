package it.unibo.collektive.qp

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBEnv
import com.gurobi.gurobi.GRBLinExpr
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBQuadExpr
import it.unibo.alchemist.collektive.device.CollektiveDevice
import it.unibo.alchemist.model.positions.Euclidean2DPosition
import it.unibo.collektive.aggregate.api.Aggregate
import it.unibo.collektive.alchemist.device.sensors.EnvironmentVariables
import it.unibo.collektive.alchemist.device.sensors.LocationSensor
import it.unibo.collektive.qp.utils.Robot
import it.unibo.collektive.qp.utils.SpeedControl2D
import it.unibo.collektive.qp.utils.Target
import it.unibo.collektive.qp.utils.getRobot
import it.unibo.collektive.qp.utils.getTarget
import it.unibo.collektive.qp.utils.moveNodeToPosition
import it.unibo.collektive.qp.utils.moveTargetIfNeeded
import it.unibo.collektive.qp.utils.plus
import it.unibo.collektive.qp.utils.setLicense

// PROBLEM:
// two "robots" have to go towards a goal point
// they must not go through a certain area in the middle of their trajectory
// minimize

fun Aggregate<Int>.entrypoint(
    device: CollektiveDevice<Euclidean2DPosition>,
    env: EnvironmentVariables,
    position: LocationSensor,
) = context(device, env, position) {
    val targetPosition = getTarget()
    val robotPosition = with(env) { getRobot() }
    val velocity = singleRobotToTarget(robotPosition, targetPosition)
    moveNodeToPosition(robotPosition + velocity)
    moveTargetIfNeeded(0,20)
}

// alogrithm ADMM come baseline di consenso del QP (SOTA)
// mandano dati sulla mia decisione ma non correlati direttamente (privacy), garanzie di convergenza
// nella coordinazione tra gli agenti
// eventualmente in programmazione aggregata

// FIRST STEP
// one robot should get to the target

// SECOND STEP
// metto un ostacolo tra i robot e il target

// THIRD STEP
// mettere il boundary tra i robots
/**
    min ||x_g - x||^2

    s.t. x_1 = A_x0 + B_uk,
         ||u_k|| <= u_max

    Find the optimal control to go towards the defined target,
    without taking in account any obstacle.
*/
fun singleRobotToTarget(robot: Robot, target: Target): SpeedControl2D {
    // Tell Gurobi exactly where the license is
    setLicense()

    // create environment in manual mode (because of license file specification)
    val env = GRBEnv(true)
    env.start()
    // create an optimization model inside the environment
    val model = GRBModel(env)

    // decision variables
    val x = model.addVar(-GRB.INFINITY, GRB.INFINITY, 0.0, GRB.CONTINUOUS, "x")
    val y = model.addVar(-GRB.INFINITY, GRB.INFINITY, 0.0, GRB.CONTINUOUS, "y")

    // control input (velocity or displacement) bounds represent admissible control directions
    val ux = model.addVar(-robot.maxSpeed, robot.maxSpeed, 0.0, GRB.CONTINUOUS, "ux")
    val uy = model.addVar(-robot.maxSpeed, robot.maxSpeed, 0.0, GRB.CONTINUOUS, "uy")

    // x-coordinate dynamics ( x_0 + u_x )
    val dynX = GRBLinExpr()
    dynX.addConstant(robot.x) // x0 runtime state
    dynX.addTerm(1.0, ux) // + 1ux
    // x = x0 + ux
    model.addConstr(x, GRB.EQUAL, dynX, "dyn_x")

    // y-coordinate dynamics (y_0 + u_y)
    val dynY = GRBLinExpr()
    dynY.addConstant(robot.y) // y0 (runtime state)
    dynY.addTerm(1.0, uy) // + 1uy
    // y = y0 + uy
    model.addConstr(y, GRB.EQUAL, dynY, "dyn_y")

    // norm constraint on the control input ux^2 + uy^2 <= maxSpeed^2
    val normU = GRBQuadExpr()
    normU.addTerm(1.0, ux, ux)
    normU.addTerm(1.0, uy, uy)
    model.addQConstr(normU, GRB.LESS_EQUAL, robot.maxSpeed * robot.maxSpeed, "u_norm")

    // minimize squared Euclidean distance to the target (x1 - xg)^2 + (y1 - yg)^2
    val obj = GRBQuadExpr()
    // quadratic terms
    obj.addTerm(1.0, x, x)
    obj.addTerm(1.0, y, y)
    // linear terms
    obj.addTerm(-2.0 * target.x, x)
    obj.addTerm(-2.0 * target.y, y)

    model.setObjective(obj, GRB.MINIMIZE)
    model.optimize() // solve

    // extract optimal control
    val uxOpt = ux.get(GRB.DoubleAttr.X)
    val uyOpt = uy.get(GRB.DoubleAttr.X)
    println("Optimal control: u = ($uxOpt, $uyOpt)")
    // free resources
    model.dispose()
    env.dispose()
    return SpeedControl2D(uxOpt, uyOpt)
}
