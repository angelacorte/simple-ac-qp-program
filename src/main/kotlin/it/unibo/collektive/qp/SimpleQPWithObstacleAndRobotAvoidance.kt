package it.unibo.collektive.qp

import it.unibo.alchemist.collektive.device.CollektiveDevice
import it.unibo.alchemist.model.positions.Euclidean2DPosition
import it.unibo.collektive.aggregate.api.Aggregate
import it.unibo.collektive.alchemist.device.sensors.EnvironmentVariables
import it.unibo.collektive.alchemist.device.sensors.LocationSensor
import it.unibo.collektive.qp.utils.getObstacle
import it.unibo.collektive.qp.utils.getRobot
import it.unibo.collektive.qp.utils.getRobotsToAvoid
import it.unibo.collektive.qp.utils.getTarget
import it.unibo.collektive.qp.utils.moveNodeToPosition
import it.unibo.collektive.qp.utils.moveTargetTo
import it.unibo.collektive.qp.utils.plus

// PROBLEM:
// two "robots" have to go towards a goal point
// they must not go through a certain area in the middle of their trajectory
// minimize

fun Aggregate<Int>.entrypointWithObstacleAndRobotAvoidance(
    device: CollektiveDevice<Euclidean2DPosition>,
    env: EnvironmentVariables,
    position: LocationSensor,
) = context(device, env, position) {
    val obstaclePosition = getObstacle()
    val target = getTarget(env["TargetID"] as Number)
    val robot = with(env) { getRobot(localId) }
    val robotsToAvoid = getRobotsToAvoid(robot.id).also { env["avoid"] = it } // todo should be aggregate
    val velocity = robotToTargetWithObstacleAndRobotAvoidance(robot, target, obstaclePosition, robotsToAvoid)
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

