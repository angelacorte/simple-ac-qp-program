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
import it.unibo.collektive.qp.utils.Obstacle
import it.unibo.collektive.qp.utils.Robot
import it.unibo.collektive.qp.utils.SpeedControl2D
import it.unibo.collektive.qp.utils.Target
import it.unibo.collektive.qp.utils.getObstacle
import it.unibo.collektive.qp.utils.getRobot
import it.unibo.collektive.qp.utils.getRobotsToAvoid
import it.unibo.collektive.qp.utils.getTarget
import it.unibo.collektive.qp.utils.moveNodeToPosition
import it.unibo.collektive.qp.utils.moveTargetTo
import it.unibo.collektive.qp.utils.plus
import it.unibo.collektive.qp.utils.setLicense
import kotlin.math.max
import kotlin.math.min

// PROBLEM:
// two "robots" have to go towards a goal point
// they must not go through a certain area in the middle of their trajectory
// minimize

fun Aggregate<Int>.entrypointWithObstacleAndRobotAvoidance(
    device: CollektiveDevice<Euclidean2DPosition>,
    env: EnvironmentVariables,
    position: LocationSensor,
) = context(device, env, position) {
    val obstaclePosition = with(env) { getObstacle() }
    val target = getTarget(env["TargetID"] as Number)
    val robot = with(env) { getRobot(localId) }
    val robotsToAvoid = getRobotsToAvoid(robot.id).also { env["avoid"] = it } // todo should be aggregate
    val velocity = singleRobotToTargetWithObstacleAndRobotAvoidance(robot, target, obstaclePosition, robotsToAvoid)
    env["Velocity"] = velocity
    moveNodeToPosition(robot + velocity)
    if (device.environment.simulation.time.toDouble() >= 50.0) {
        moveTargetTo(target.id, target.id, target.id)
    }
}

// alogrithm ADMM come baseline di consenso del QP (SOTA)
// mandano dati sulla mia decisione ma non correlati direttamente (privacy), garanzie di convergenza
// nella coordinazione tra gli agenti
// eventualmente in programmazione aggregata

// SECOND STEP
// metto un ostacolo tra i robot e il target

// THIRD STEP
// mettere il boundary tra i robots

/**
        min ||u - u^nom||^2 + \delta
 s.t.   2(p - p_o)^T u + \gamma [ ||p - p_o||^2 - (r_o ^ 2 -+ d_o^2) ] >= 0 (OBSTACLE AVOIDANCE)
        2(p1 - p2)^T (u1 - u2) + \gamma [ (p1-p2)^T(p1-p2) - dmin^2 ] >= 0 (ROBOT AVOIDANCE)
        ||u_k|| <= u_max
        2(p - p_g)^T u <= -c || p - p_g ||^2 + \delta

Find the optimal control to go towards the defined target,
without taking in account any obstacle.
 */
fun singleRobotToTargetWithObstacleAndRobotAvoidance(
    robot: Robot,
    target: Target,
    obstacle: Obstacle,
    robotsToAvoid: List<Robot>,
): SpeedControl2D {
    // Tell Gurobi exactly where the license is
    setLicense()

    // create environment in manual mode (because of license file specification)
    val env = GRBEnv(true)
    env.start()
    // create an optimization model inside the environment
    val model = GRBModel(env)

    // decision variables
    // control input (velocity or displacement) bounds represent admissible control directions
    val ux = model.addVar(-robot.maxSpeed, robot.maxSpeed, 0.0, GRB.CONTINUOUS, "ux")
    val uy = model.addVar(-robot.maxSpeed, robot.maxSpeed, 0.0, GRB.CONTINUOUS, "uy")
    val delta = model.addVar(0.0, GRB.INFINITY, 0.0, GRB.CONTINUOUS, "delta")

    // CONSTRAINTS

    // (OBSTACLE AVOIDANCE) linear CBF 2(p - p_o)^T u >= - \gamma [ ||p - p_o||^2 - (r_o ^ 2 -+ d_o^2) ]
    val dxo = robot.x - obstacle.x
    val dyo = robot.y - obstacle.y

    val obstAvoidance = GRBLinExpr()
    // 2(p - p-g)^T u = 2(p_x - p_o,x) u_x + 2(p_y - p_o,y) u_y
    obstAvoidance.addTerm(2.0 * dxo, ux)
    obstAvoidance.addTerm(2.0 * dyo, uy)
    // - \gamma[ ( (p_x - p_o,x)^2 + (p_y - p_o,y) ^2 - (r_o^2 + d_o^2) ]
    val cbfGamma = 0.5 // \gamma in {0.5 .. 5} = soft || in {5, 20} = hard || > infeasible QP
    val h =
        -cbfGamma * (dxo * dxo + dyo * dyo - (obstacle.radius * obstacle.radius + obstacle.margin * obstacle.margin))
    model.addConstr(obstAvoidance, GRB.GREATER_EQUAL, h, "obstacleAvoidance")

    // (ROBOT AVOIDANCE) linear CBF 2(p1 - p2)^T (u1 - u2) + \gamma [ (p1-p2)^T(p1-p2) - dmin^2 ] >= 0
    // move to the right
    // 2(p1 - p2)^T u1 >= -2(p1 - p2)^T u2 - \gamma [ (p1-p2)^T(p1-p2) - dmin^2 ]
    robotsToAvoid.forEach { avoid ->
        val minDist = max(avoid.margin, robot.margin) * max(avoid.margin, robot.margin)
        val dxr = robot.x - avoid.x //p1x - p2x
        val dyr = robot.y - avoid.y //p1y - p2y
        // p1 = (p1x, p1y)
        // p2 = (p2x, p2y)
        val distSquared = dxr * dxr + dyr * dyr // (p1 - p2)^T (p1 - p2) = (p1x - p2x)^2 + (p1y - p2y)^2
        val robotAvoidance = GRBLinExpr()

        // left side
        // 2(p1-p2)^T u1 // my velocity
        robotAvoidance.addTerm(2.0 * dxr, ux)
        robotAvoidance.addTerm(2.0 * dyr, uy)

        // vel robot 2
        val uxa = avoid.velocity.x
        val uya = avoid.velocity.y
        // right side
        // -2(p1-p2)^T u2 - \gamma [ (p1-p2)^T(p1-p2) - dmin^2 ]
        // (p1x - p2x) p1x = dxr * uxa
        // (p1y - p2y) p1y = dyr * uya
        val f = -2 * (dxr * uxa + dyr * uya) - cbfGamma * (distSquared - minDist)
        model.addConstr(robotAvoidance, GRB.GREATER_EQUAL, f, "robotAvoidance_${robot.id}against${avoid.id}")
    }

    // norm constraint on the control input ux^2 + uy^2 <= maxSpeed^2
    val normU = GRBQuadExpr()
    normU.addTerm(1.0, ux, ux)
    normU.addTerm(1.0, uy, uy)
    model.addQConstr(normU, GRB.LESS_EQUAL, robot.maxSpeed * robot.maxSpeed, "u_norm")

    // linear CLF 2(p - p_g)^T u <= -c || p - p_g ||^2 + \delta
    val clf = GRBLinExpr()
    val c = 1 // should vary based on deltaTime, if small, c should be smaller, if deltaTime ~ 1sec then in {0.5, 5}
    // if c is big, faster convergence
    // 2(p - p_g)^T u = 2(p_x - p_g,x) u_x + 2(p_y - p_g,y) u_y
    val dxg = robot.x - target.x
    val dyg = robot.y - target.y
    clf.addTerm(2.0 * dxg, ux)
    clf.addTerm(2.0 * dyg, uy)
    // -c || p - p_g ||^2 + \delta
    // - delta to the left
    clf.addTerm(-1.0, delta)
    val normSquared = dxg * dxg + dyg * dyg
    val v = -c * normSquared
    model.addConstr(clf, GRB.LESS_EQUAL, v, "clf")

    // bound delta >= 0
    model.addConstr(delta, GRB.GREATER_EQUAL, 0.0, "slack")

    // objective is quadratic in (ux, uy), linear in delta (slack)
    // ||u - u^nom||^2 + \phi \delta
    // ||u - u^nom||^2 = (ux - ux^nom)^2 + (uy - uy^nom)^2
    val uxnom = target.x - robot.x // this will be the velocity got from AC program
    val uynom = target.y - robot.y // this will be the velocity got from AC program
    val phi = 1.0 // weight for slack
    val obj = GRBQuadExpr()
    // quadratic terms
    obj.addTerm(1.0, ux, ux)
    obj.addTerm(1.0, uy, uy)
    // linear terms
    obj.addTerm(-2.0 * uxnom, ux)
    obj.addTerm(-2.0 * uynom, uy)
    obj.addTerm(phi, delta)
    model.setObjective(obj, GRB.MINIMIZE)
    model.optimize() // solve

    // extract optimal control
    val uxOpt = ux.get(GRB.DoubleAttr.X)
    val uyOpt = uy.get(GRB.DoubleAttr.X)
    println("Optimal control  for ${robot.id}: u = ($uxOpt, $uyOpt)")
    // free resources
    model.dispose()
    env.dispose()
    return SpeedControl2D(uxOpt, uyOpt)
}
