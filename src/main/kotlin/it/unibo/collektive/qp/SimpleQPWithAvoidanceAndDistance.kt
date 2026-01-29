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

// PROBLEM:
// two "robots" have to go towards a goal point
// they must not go through a certain area in the middle of their trajectory
// minimize

fun Aggregate<Int>.entrypointWithAvoidanceAndDistance(
    device: CollektiveDevice<Euclidean2DPosition>,
    env: EnvironmentVariables,
    position: LocationSensor,
) = context(device, env, position) {
    val obstaclePosition = with(env) { getObstacle() }
    val target = getTarget(env["TargetID"] as Number)
    val robot = with(env) { getRobot(localId) }
    val robotsToAvoid = getRobotsToAvoid(robot.id).also { env["avoid"] = it } // todo should be aggregate
    val communicationDistance = env["CommunicationDistance"] as Double
    val velocity = robotToTargetWithAvoidanceAndDistance(robot, target, obstaclePosition, robotsToAvoid, robotsToAvoid,communicationDistance)
    env["Velocity"] = velocity
    moveNodeToPosition(robot + velocity)
    if (device.environment.simulation.time.toDouble() >= 50.0) {
        moveTargetTo(target.id, target.id, target.id)
    }
}

