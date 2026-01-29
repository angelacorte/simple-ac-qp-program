package it.unibo.collektive.qp

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBEnv
import com.gurobi.gurobi.GRBLinExpr
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBQuadExpr
import it.unibo.collektive.qp.utils.Obstacle
import it.unibo.collektive.qp.utils.Robot
import it.unibo.collektive.qp.utils.SpeedControl2D
import it.unibo.collektive.qp.utils.Target
import it.unibo.collektive.qp.utils.setLicense
import kotlin.math.max

/**
       min ||u - u^nom||^2 + \delta
s.t.   2(p - p_o)^T u + \gamma [ ||p - p_o||^2 - (r_o ^ 2 -+ d_o^2) ] >= 0 (OBSTACLE AVOIDANCE)
       2(p1 - p2)^T (u1 - u2) + \gamma [ (p1-p2)^T(p1-p2) - dmin^2 ] >= 0 (ROBOT AVOIDANCE)
       -2(p1 - p2)^T (u1 -u2) + \gamma [ R^2 - ||p1 - p2||^2 ] >= 0 (COMMUNICATION DISTANCE)
       ||u_k|| <= u_max
       2(p - p_g)^T u <= -c || p - p_g ||^2 + \delta

Find the optimal control to go towards the defined target,
without taking in account any obstacle.
 */
fun robotToTargetWithAvoidanceAndDistance(
    robot: Robot,
    target: Target,
    obstacle: Obstacle,
    robotsToAvoid: List<Robot> = emptyList(),
    robotsToBeConnected: List<Robot> = emptyList(),
    maxConnectionDistance: Double = Double.MIN_VALUE,
): SpeedControl2D {
    setLicense() // Tell Gurobi exactly where the license is
    val env = GRBEnv(true) // create environment in manual mode (because of license file specification)
    env.start()
    val model = GRBModel(env) // create an optimization model inside the environment

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
    val safeMargin = obstacle.radius + obstacle.margin
    val h = -cbfGamma * (dxo * dxo + dyo * dyo - (safeMargin * safeMargin))
    model.addConstr(obstAvoidance, GRB.GREATER_EQUAL, h, "obstacleAvoidance")

    if (robotsToAvoid.isNotEmpty()) {
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
    }

    if(robotsToBeConnected.isNotEmpty() && maxConnectionDistance != Double.MIN_VALUE) {
        // (COMMUNICATION DISTANCE) CBF -2(p1 - p2)^T (u1 -u2) + \gamma [ R^2 - ||p1 - p2||^2 ] >= 0
        // move to the right
        // -2(p1 - p2)^T (u1 -u2) >= - \gamma [ R^2 - ||p1 - p2||^2 ]
        // -2(p1 - p2)^T u1 >= 2(p1 - p2)^T u2 - \gamma [ R^2 - ||p1 - p2||^2 ]
        // ||p1 - p2||^2 = (p1 - p2)^T (p1 -p2)
        val maxDist = maxConnectionDistance * maxConnectionDistance
        robotsToBeConnected.forEach { connect ->
            val dxr = robot.x - connect.x //p1x - p2x
            val dyr = robot.y - connect.y //p1y - p2y
            val distSquared = dxr * dxr + dyr * dyr
            val commRange = GRBLinExpr()

            // -2(p1-p2)^T u1 // my velocity
            commRange.addTerm(-2.0 * dxr, ux)
            commRange.addTerm(-2.0 * dyr, uy)

            // vel robot 2
            val uxa = connect.velocity.x
            val uya = connect.velocity.y

            // 2(p1-p2)^T u2 - \gamma [ R^2 - (p1-p2)^T(p1-p2)  ]
            // (p1-p2)^T(p1-p2) = (p1x - p2x) p1x + (p1y - p2y) p1y
            // (p1x - p2x) p1x = dxr * uxa
            // (p1y - p2y) p1y = dyr * uya
            val f = 2.0 * (dxr * uxa + dyr * uya) - cbfGamma * (maxDist - distSquared)
            model.addConstr(commRange, GRB.GREATER_EQUAL, f, "communicationRange_${robot.id}with${connect.id}")
        }
    }

    // norm constraint on the control input ux^2 + uy^2 <= maxSpeed^2
    val normU = GRBQuadExpr()
    normU.addTerm(1.0, ux, ux)
    normU.addTerm(1.0, uy, uy)
    model.addQConstr(normU, GRB.LESS_EQUAL, robot.maxSpeed * robot.maxSpeed, "u_norm")

    // (GO TO TARGET) linear CLF 2(p - p_g)^T u <= -c || p - p_g ||^2 + \delta
    val clf = GRBLinExpr()
    val c = 1 // should vary based on deltaTime, if small, c should be smaller, if deltaTime ~ 1sec then in {0.5, 5}
    // if c is big, faster convergence
    // 2(p - p_g)^T u = 2(p_x - p_g,x) u_x + 2(p_y - p_g,y) u_y
    val dxg = robot.x - target.x // (p_x - p_g,x)
    val dyg = robot.y - target.y // (p_y - p_g,y)
    clf.addTerm(2.0 * dxg, ux) // 2(p_x - p_g,x) u_x
    clf.addTerm(2.0 * dyg, uy) // 2(p_y - p_g,y) u_y
    // -c || p - p_g ||^2 + \delta
    // - delta to the left
    clf.addTerm(-1.0, delta)

    // right term -c || p - p_g ||^2
    val normSquared = dxg * dxg + dyg * dyg
    val v = -c * normSquared
    model.addConstr(clf, GRB.LESS_EQUAL, v, "clf")

    // bound delta >= 0
    model.addConstr(delta, GRB.GREATER_EQUAL, 0.0, "slack")

    // objective is quadratic in (ux, uy), linear in delta (slack)
    // ||u - u^nom||^2 + \phi \delta^2
    // ||u - u^nom||^2 = (ux - ux^nom)^2 + (uy - uy^nom)^2
    val uxNom = target.x - robot.x // this will be the velocity got from AC program
    val uyNom = target.y - robot.y // this will be the velocity got from AC program
    val phi = 1.0 // weight for slack

    val deltaUX = model.addVar(-GRB.INFINITY,GRB.INFINITY,0.0,GRB.CONTINUOUS,"deltaux")
    val linDux = GRBLinExpr()
    linDux.addTerm(1.0, ux)
    linDux.addTerm(-1.0, deltaUX)
    model.addConstr(linDux, GRB.EQUAL, uxNom, "lin_dux")

    val deltaUY = model.addVar(-GRB.INFINITY,GRB.INFINITY,0.0,GRB.CONTINUOUS,"deltauy")
    val linDuy = GRBLinExpr()
    linDuy.addTerm(1.0, uy)
    linDuy.addTerm(-1.0, deltaUY)
    model.addConstr(linDuy, GRB.EQUAL, uyNom, "lin_duy")

    val obj = GRBQuadExpr()
    obj.addTerm(1.0, deltaUX, deltaUX)  // ||u - u^nom||^2 x,
    obj.addTerm(1.0, deltaUY, deltaUY) // ||u - u^nom||^2 y
    obj.addTerm(phi, delta, delta) // delta^2

    model.setObjective(obj, GRB.MINIMIZE)
    model.optimize() // solve

    // extract optimal control
    val uxOptimal = ux.get(GRB.DoubleAttr.X)
    val uyOptimal = uy.get(GRB.DoubleAttr.X)
    println("Optimal control for ${robot.id}: u = ($uxOptimal, $uyOptimal)")
    // free resources
    model.dispose()
    env.dispose()
    return SpeedControl2D(uxOptimal, uyOptimal)
}
