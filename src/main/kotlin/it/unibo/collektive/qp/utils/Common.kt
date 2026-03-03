package it.unibo.collektive.qp.utils

import it.unibo.alchemist.collektive.device.CollektiveDevice
import it.unibo.alchemist.model.Position
import it.unibo.alchemist.model.molecules.SimpleMolecule
import it.unibo.alchemist.model.positions.Euclidean2DPosition
import it.unibo.collektive.alchemist.device.sensors.EnvironmentVariables
import it.unibo.collektive.alchemist.device.sensors.LocationSensor

context(device: CollektiveDevice<Euclidean2DPosition>)
/**
 * Relocates the current node to [newPosition] within the environment.
 */
fun moveNodeToPosition(
    newPosition: Vector2D,
) {
    val envPos: Position<Euclidean2DPosition> = device.environment.makePosition(newPosition.x, newPosition.y)
    device.environment.moveNodeToPosition(device.node, envPos as Euclidean2DPosition)
}

context(device: CollektiveDevice<Euclidean2DPosition>)
/**
 * Relocates a node identified by [nodeID] to [newPosition].
 */
fun moveNodeToPosition(
    nodeID: Int,
    newPosition: Vector2D,
) {
    val envPos: Position<Euclidean2DPosition> = device.environment.makePosition(newPosition.x, newPosition.y)
    val node = device.environment.nodes.find { it.id == nodeID }
    if (node != null) {
        device.environment.moveNodeToPosition(node, envPos as Euclidean2DPosition)
    } else {
        error("Could not find a node with ID $nodeID")
    }
}

context(device: CollektiveDevice<Euclidean2DPosition>)
/**
 * Moves a target node [targetId] [to] the provided coordinates.
 */
fun moveTargetTo(
    targetId: Number,
    vararg to: Number,
) {
    val targetNode = device.environment.nodes.find { it.getConcentration(SimpleMolecule("Target")) == targetId }
        ?: error("Target $targetId not found")
    val position: Position<Euclidean2DPosition> = device.environment.makePosition(to.asList())
    device.environment.moveNodeToPosition(targetNode, position as Euclidean2DPosition)
}

context(position: LocationSensor)
/**
 * Fetches a target by [targetId] from the current location sensor context.
 */
fun getTarget(
    targetId: Number,
): Target = position.targetsPosition().find { it.id == targetId } ?: error("Target $targetId not found.")

context(position: LocationSensor, env: EnvironmentVariables)
/**
 * Builds a [Robot] view for the current device state using environment variables.
 */
fun getRobot(): Robot =
    position.coordinates().let {
        val velocity = env.getOrDefault("Velocity", SpeedControl2D(0.0, 0.0))
        Robot(it.x, it.y, 3.0, velocity, env["MaxSpeed"])
    }

context(device: CollektiveDevice<*>)
/**
 * Returns the first obstacle in the environment, failing fast if none is present.
 */
fun getObstacle(): Obstacle {
    val obstacle =
        device.environment.nodes.firstOrNull { it.contains(SimpleMolecule("Obstacle")) }
            ?: error("Currently, there are no obstacles in the environment")
    val obstaclePos = device.environment.getPosition(obstacle).coordinates
    val radius = obstacle.getConcentration(SimpleMolecule("SafeRadius")) as Double
    val margin = obstacle.getConcentration(SimpleMolecule("SafeMargin")) as Double
    return Obstacle(obstaclePos[0], obstaclePos[1], radius, margin)
}

// context(device: CollektiveDevice<*>)
// /**
// * Collects every other robot (except [currentRobot]) enriched with velocity and speed metadata.
// */
// fun getRobotsToAvoid(currentRobot: Int): List<Robot> = device.environment.nodes
//    .filter { it.contains(SimpleMolecule("Robot")) }
//    .filterNot { it.id == currentRobot }
//    .map { node ->
//        val coord = device.environment.getPosition(node).coordinates
//        val margin = node.getConcentration(SimpleMolecule("SafeMargin")) as Double
//        val velMolecule = SimpleMolecule("Velocity")
//        val velocity: SpeedControl2D = if (node.contains(velMolecule)) {
//            node.getConcentration(velMolecule) as SpeedControl2D
//        } else {
//            SpeedControl2D(0.0, 0.0)
//        }
//        val maxSpeed = node.getConcentration(SimpleMolecule("MaxSpeed")) as Double
//        Robot(coord[0], coord[1], margin, velocity, maxSpeed)
//    }
