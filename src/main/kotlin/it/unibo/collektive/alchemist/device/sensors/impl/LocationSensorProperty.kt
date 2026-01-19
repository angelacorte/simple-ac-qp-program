package it.unibo.collektive.alchemist.device.sensors.impl

import it.unibo.alchemist.model.Environment
import it.unibo.alchemist.model.Node
import it.unibo.alchemist.model.NodeProperty
import it.unibo.alchemist.model.Position
import it.unibo.alchemist.model.molecules.SimpleMolecule
import it.unibo.collektive.alchemist.device.sensors.LocationSensor
import it.unibo.collektive.qp.utils.Coordinate
import it.unibo.collektive.qp.utils.Target
import org.apache.commons.math3.random.RandomGenerator

/**
 * An implementation of a location sensor property for nodes in an Alchemist environment.
 *
 * @param T the concentration type managed by the node
 * @param P the position type used in the environment
 * @property environment the simulation environment
 * @property node the node associated with this sensor property
 * @property random an instance of a random number generator for simulating measurement noise
 * @property stdDev the standard deviation for Gaussian noise added to target positions
 */
class LocationSensorProperty<T : Any, P : Position<P>>(
    private val environment: Environment<T, P>,
    override val node: Node<T>,
    private val random: RandomGenerator,
    private val stdDev: Double = 0.5,
    private val blindSpotDistance: Double = Double.MAX_VALUE, // no blind spot by default
) : LocationSensor,
    NodeProperty<T> {

    override fun cloneOnNewNode(node: Node<T>): NodeProperty<T> =
        LocationSensorProperty(environment, node, random, stdDev)

    override fun coordinates(): Coordinate {
        val position = environment.getPosition(node).coordinates
        return Coordinate(position[0], position[1])
    }

    override fun surroundings(): List<Coordinate> = environment.getNeighborhood(node).map { node ->
        environment.getPosition(node).coordinates.let { Coordinate(it[0], it[1]) }
    }

    override fun targetsPosition(): List<Coordinate> = environment.nodes
        .filter { node ->
            node.contains(SimpleMolecule("Movable"))
        }.map { target ->
            environment.getPosition(target)
        }.filter { position ->
            position.distanceTo(environment.getPosition(node)) <= blindSpotDistance
        }.map { position ->
            val newX = position.coordinates[0] + (random.nextGaussian() * stdDev)
            val newY = position.coordinates[1] + (random.nextGaussian() * stdDev)
            Coordinate(newX, newY)
        }
}
