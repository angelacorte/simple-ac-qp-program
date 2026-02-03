package it.unibo.collektive.qp.carol

import it.unibo.alchemist.collektive.device.CollektiveDevice
import it.unibo.alchemist.model.positions.Euclidean2DPosition
import it.unibo.collektive.aggregate.api.Aggregate
import it.unibo.collektive.aggregate.api.neighboring
import it.unibo.collektive.aggregate.api.share
import it.unibo.collektive.aggregate.values
import it.unibo.collektive.alchemist.device.sensors.EnvironmentVariables
import it.unibo.collektive.alchemist.device.sensors.LocationSensor
import it.unibo.collektive.qp.dsl.robotToTargetWithAvoidanceAndDistance
import it.unibo.collektive.qp.utils.getObstacle
import it.unibo.collektive.qp.utils.getRobot
import it.unibo.collektive.qp.utils.getTarget
import it.unibo.collektive.qp.utils.minus
import it.unibo.collektive.qp.utils.moveNodeToPosition
import it.unibo.collektive.qp.utils.moveTargetTo
import it.unibo.collektive.qp.utils.plus

fun Aggregate<Int>.entrypointSimpleCarol(
    device: CollektiveDevice<Euclidean2DPosition>,
    env: EnvironmentVariables,
    position: LocationSensor,
) = context(device, env, position) {
    val obstaclePosition = getObstacle()
    val targetPosition = getTarget(env["TargetID"] as Number)
    val localInfos = with(env) { getRobot(localId) }
    share(localInfos) { robotInfo ->
        val neighboringRobots = robotInfo.neighbors.values.list
        val myVelocity = robotToTargetWithAvoidanceAndDistance(robotInfo.local.value, targetPosition, obstaclePosition, neighboringRobots)
        val newPosition = robotInfo.local.value + myVelocity
        moveNodeToPosition(newPosition)
        robotInfo.local.value.copy(x = newPosition.x, y = newPosition.y, velocity = myVelocity)
    }
    val obs = device.environment.makePosition(obstaclePosition.x, obstaclePosition.y)
    env["distanceToObstacle"] = device.environment.getPosition(device.node).distanceTo(obs)
    if (device.environment.simulation.time.toDouble() in 50.0 .. 51.0) {
        moveTargetTo(targetPosition.id, targetPosition.id, targetPosition.id)
    }
}

fun Aggregate<Int>.entrypointCarol(
    device: CollektiveDevice<Euclidean2DPosition>,
    env: EnvironmentVariables,
    position: LocationSensor,
) = context(device, env, position) {
    val obstaclePosition = getObstacle()
    val communicationDistance = env["CommunicationDistance"] as Double
    val targetPosition = getTarget(env["TargetID"] as Number)
    val localInfos = with(env) { getRobot(localId) }
    val neighboringRobots = neighboring(localInfos).neighbors.values.list
    if(neighboringRobots.isNotEmpty()) env["distTo"] = localInfos.position - neighboringRobots.first().position
    val myVelocity = robotToTargetWithAvoidanceAndDistance(localInfos, targetPosition, obstaclePosition, neighboringRobots, neighboringRobots, communicationDistance)
    val newPosition = localInfos + myVelocity
    moveNodeToPosition(newPosition)
    env["Velocity"] = myVelocity
    if (device.environment.simulation.time.toDouble() >= 50.0) {
        moveTargetTo(targetPosition.id, targetPosition.id, targetPosition.id)
    }
}
