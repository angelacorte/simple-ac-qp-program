package it.unibo.alchemist.model.actions

import it.unibo.alchemist.model.Action
import it.unibo.alchemist.model.Environment
import it.unibo.alchemist.model.Node
import it.unibo.alchemist.model.Reaction
import it.unibo.alchemist.model.molecules.SimpleMolecule
import it.unibo.alchemist.model.positions.Euclidean2DPosition

/**
 * Represents a movement action that updates the position of a node within a Euclidean 2D environment.
 * This class defines the motion dynamics, including velocity, boundaries, and trajectory behavior.
 *
 * @param T the concentration type managed by the node
 * @param environment the simulation environment in which the node resides
 * @param node the node under movement
 */
class MoveNode<T>(environment: Environment<T, Euclidean2DPosition>, node: Node<T>) :
    AbstractMoveNode<T, Euclidean2DPosition>(environment, node, false) {

    override fun getNextPosition(): Euclidean2DPosition {
        val maxSpeed = getSpeed(node.id, "MaxSpeed")
        val velX = getSpeed(node.id, "VelX").coerceIn(-maxSpeed, maxSpeed)
        val velY = getSpeed(node.id, "VelY").coerceIn(-maxSpeed, maxSpeed)
        return Euclidean2DPosition(velX, velY)
    }

    private fun getSpeed(nodeID: Int, moleculeName: String): Double {
        val node = environment.nodes.first { it.id == nodeID && it.contains(SimpleMolecule("Robot")) }
        return if (node.contains(SimpleMolecule("Robot")) && node.contains(SimpleMolecule(moleculeName))) {
            node.getConcentration(SimpleMolecule(moleculeName)) as Double
        } else {
            0.0
        }
    }

    override fun cloneAction(p0: Node<T>, p1: Reaction<T>): Action<T> = MoveNode(environment, node)
}
