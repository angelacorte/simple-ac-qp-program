package it.unibo.collektive.qp.utils

import it.unibo.alchemist.collektive.device.CollektiveDevice
import it.unibo.alchemist.model.Position
import it.unibo.alchemist.model.molecules.SimpleMolecule
import it.unibo.alchemist.model.positions.Euclidean2DPosition
import it.unibo.collektive.alchemist.device.sensors.EnvironmentVariables
import it.unibo.collektive.alchemist.device.sensors.LocationSensor
import java.nio.file.Files
import java.nio.file.Path
import java.nio.file.Paths

context(device: CollektiveDevice<Euclidean2DPosition>)
fun moveNodeToPosition(newPosition: Point) {
    val envPos: Position<Euclidean2DPosition> = device.environment.makePosition(newPosition.x, newPosition.y)
    device.environment.moveNodeToPosition(device.node, envPos as Euclidean2DPosition)
}

context(device: CollektiveDevice<Euclidean2DPosition>)
fun moveTargetTo(targetId: Number, vararg to: Number) {
    val targetNode = device.environment.nodes.find { it.getConcentration(SimpleMolecule("Target")) == targetId }
        ?: error("Target $targetId not found")
    val position: Position<Euclidean2DPosition> = device.environment.makePosition(to.asList())
    device.environment.moveNodeToPosition(targetNode, position as Euclidean2DPosition)
}

context(position: LocationSensor)
fun getTarget(targetId: Number): Target =
    position.targetsPosition().find { it.id == targetId } ?: error("Target $targetId not found.")

context(position: LocationSensor, env: EnvironmentVariables)
fun getRobot(robotId: Number): Robot = position.coordinates().let {
    val velocity = env.getOrDefault("Velocity", SpeedControl2D(0.0, 0.0))
    Robot(it.x, it.y, robotId, 3.0, velocity, env["MaxSpeed"])
}

context(device: CollektiveDevice<*>)
fun getObstacle(): Obstacle {
    val obstacle =
        device.environment.nodes.firstOrNull { it.contains(SimpleMolecule("Obstacle")) }
            ?: error("Currently, there are no obstacles in the environment")
    val obstaclePos = device.environment.getPosition(obstacle).coordinates
    val radius = obstacle.getConcentration(SimpleMolecule("SafeRadius")) as Double
    val margin = obstacle.getConcentration(SimpleMolecule("SafeMargin")) as Double
    return Obstacle(obstaclePos[0], obstaclePos[1], radius, margin)
}

context(device: CollektiveDevice<*>)
fun getRobotsToAvoid(currentRobot: Number): List<Robot> = device.environment.nodes
    .filter { it.contains(SimpleMolecule("Robot")) }
    .filterNot { it.id == currentRobot }
    .map { node ->
        val coord = device.environment.getPosition(node).coordinates
        val margin = node.getConcentration(SimpleMolecule("SafeMargin")) as Double
        val velMolecule = SimpleMolecule("Velocity")
        val velocity: SpeedControl2D = if (node.contains(velMolecule)) {
            node.getConcentration(velMolecule) as SpeedControl2D
        } else {
            SpeedControl2D(0.0, 0.0)
        }
        val maxSpeed = node.getConcentration(SimpleMolecule("MaxSpeed")) as Double
        Robot(coord[0], coord[1], node.id, margin, velocity, maxSpeed)
    }

/**
 * Return the first existing candidate path for the Gurobi license file.
 * Candidates (in order):
 *  - GRB_LICENSE_FILE environment variable
 *  - GRB_LICENSE_FILE JVM system property
 *  - default under user.home/Library/gurobi/gurobi.lic
 */
private fun resolveLicensePath(): Path? {
    val envPath = System.getenv("GRB_LICENSE_FILE")
    if (!envPath.isNullOrBlank()) {
        val p = Paths.get(envPath)
        if (Files.exists(p)) return p
    }
    val sysProp = System.getProperty("GRB_LICENSE_FILE")
    if (!sysProp.isNullOrBlank()) {
        val p = Paths.get(sysProp)
        if (Files.exists(p)) return p
    }
    val defaultPath = Paths.get(System.getProperty("user.home"), "Library", "gurobi", "gurobi.lic")
    return if (Files.exists(defaultPath)) defaultPath else null
}

fun setLicense() {
    val found = resolveLicensePath()
    if (found != null) {
        System.setProperty("GRB_LICENSE_FILE", found.toString())
        return
    }
    val defaultPath = Paths.get(System.getProperty("user.home"), "Library", "gurobi", "gurobi.lic")
    throw IllegalStateException(
        "Gurobi license file not found. Set the GRB_LICENSE_FILE environment variable or JVM property to the license file path, or place the license in '$defaultPath'",
    )
}
