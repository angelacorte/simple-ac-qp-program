package it.unibo.collektive.qp

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBEnv
import com.gurobi.gurobi.GRBLinExpr
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBQuadExpr
import it.unibo.alchemist.collektive.device.CollektiveDevice
import it.unibo.alchemist.model.Position
import it.unibo.alchemist.model.molecules.SimpleMolecule
import it.unibo.alchemist.model.positions.Euclidean2DPosition
import it.unibo.collektive.aggregate.api.Aggregate
import it.unibo.collektive.alchemist.device.sensors.EnvironmentVariables
import it.unibo.collektive.alchemist.device.sensors.LocationSensor
import it.unibo.collektive.qp.utils.Point
import it.unibo.collektive.qp.utils.Robot
import it.unibo.collektive.qp.utils.SpeedControl2D
import it.unibo.collektive.qp.utils.Target
import it.unibo.collektive.qp.utils.plus

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
    val robotPosition = getRobot(env)
    val velocity = singleRobotToTarget(robotPosition, targetPosition)
    moveNodeToPosition(robotPosition + velocity)
    moveTargetIfNeeded()
}

// FIRST STEP
// one robot should get to the target
/**
    min ||x_g - x||^2

    s.t. x_1 = A_x0 + B_uk,
         ||u_k|| <= u_max
*/
fun singleRobotToTarget(robot: Robot, target: Target): SpeedControl2D {
    // Tell Gurobi exactly where the license is
    System.setProperty("GRB_LICENSE_FILE", "/Users/angela/Library/gurobi/gurobi.lic") // todo this should not be hardcoded

    // create environment in manual mode (because of license file specification0
    val env = GRBEnv(true)
    env.start()
    // create an optimization model inside the environment
    val model = GRBModel(env)

    // decision variables
    val x1 = model.addVar(-GRB.INFINITY, GRB.INFINITY, 0.0, GRB.CONTINUOUS, "x1")
    val y1 = model.addVar(-GRB.INFINITY, GRB.INFINITY, 0.0, GRB.CONTINUOUS, "y1")

    // control input (velocity or displacement) bounds represent admissible control directions
    val ux = model.addVar(-robot.maxSpeed, robot.maxSpeed, 0.0, GRB.CONTINUOUS, "ux")
    val uy = model.addVar(-robot.maxSpeed, robot.maxSpeed, 0.0, GRB.CONTINUOUS, "uy")

    // x-coordinate dynamics
    val dynX = GRBLinExpr()
    dynX.addConstant(robot.x) // x0 (runtime state)
    dynX.addTerm(1.0, ux) // + ux
    // x1 = x0 + ux
    model.addConstr(x1, GRB.EQUAL, dynX, "dyn_x")

    // y-coordinate dynamics
    val dynY = GRBLinExpr()
    dynY.addConstant(robot.y) // y0 (runtime state)
    dynY.addTerm(1.0, uy) // + uy
    // y1 = y0 + uy
    model.addConstr(y1, GRB.EQUAL, dynY, "dyn_y")

    // euclidean norm constraint on the control input ux^2 + uy^2 <= maxSpeed^2
    val normU = GRBQuadExpr()
    normU.addTerm(1.0, ux, ux)
    normU.addTerm(1.0, uy, uy)
    model.addQConstr(normU, GRB.LESS_EQUAL, robot.maxSpeed * robot.maxSpeed, "u_norm")

    // minimize squared Euclidean distance to the target (x1 - xg)^2 + (y1 - yg)^2
    val obj = GRBQuadExpr()
    // quadratic terms
    obj.addTerm(1.0, x1, x1)
    obj.addTerm(1.0, y1, y1)
    // linear terms
    obj.addTerm(-2.0 * target.x, x1)
    obj.addTerm(-2.0 * target.y, y1)
//     constant term (optional, does not affect the optimizer)
//    obj.addConstant(target.x * target.x + target.y * target.y)

    model.setObjective(obj, GRB.MINIMIZE)
    model.optimize() // solve

    // extract optimal control
    val uxOpt = ux.get(GRB.DoubleAttr.X)
    val uyOpt = uy.get(GRB.DoubleAttr.X)

    val x1Opt = x1.get(GRB.DoubleAttr.X)
    val y1Opt = y1.get(GRB.DoubleAttr.X)

    println("Optimal control: u = ($uxOpt, $uyOpt)")
    println("Next position: x = ($x1Opt, $y1Opt)")

    // free resources
    model.dispose()
    env.dispose()
    return SpeedControl2D(uxOpt, uyOpt)
}

context(device: CollektiveDevice<Euclidean2DPosition>)
fun moveNodeToPosition(newPosition: Point) {
    val envPos: Position<Euclidean2DPosition> = device.environment.makePosition(newPosition.x, newPosition.y)
    device.environment.moveNodeToPosition(device.node, envPos as Euclidean2DPosition)
}

context(device: CollektiveDevice<Euclidean2DPosition>)
private fun moveTargetIfNeeded() {
    if (device.environment.simulation.time.toDouble() >= 50.0) {
        val targetNode = device.environment.nodes.firstOrNull { it.contains(SimpleMolecule("Target")) }
            ?: error("This should not occur")
        val position: Position<Euclidean2DPosition> = device.environment.makePosition(0, 20)
        device.environment.moveNodeToPosition(targetNode, position as Euclidean2DPosition)
    }
}

context(position: LocationSensor)
fun getTarget(): Target = position.targetsPosition().firstOrNull().let {
    if (it == null) error("Currently, this should never be null")
    Target(it.x, it.y)
}

context(position: LocationSensor)
fun getRobot(env: EnvironmentVariables): Robot = position.coordinates().let {
    Robot(it.x, it.y, 3.0, env["MaxSpeed"])
}
