package it.unibo.alchemist.model.movestrategies.target

import it.unibo.alchemist.model.Node
import it.unibo.alchemist.model.Position
import it.unibo.alchemist.model.Reaction
import it.unibo.alchemist.model.molecules.SimpleMolecule
import it.unibo.alchemist.model.movestrategies.SpeedSelectionStrategy
import it.unibo.collektive.model.SpeedControl2D
import kotlin.math.hypot
import kotlin.math.sqrt

class SpeedFromMolecule<T, P : Position<P>>(
    private val node: Node<T>,
    private val reaction: Reaction<T>
) : SpeedSelectionStrategy<T, P> {

    override fun getNodeMovementLength(target: P?): Double {
        val speed = node.getConcentration(SimpleMolecule("Velocity")) as? SpeedControl2D ?: return 0.0
        val speedMagnitude = hypot(speed.x, speed.y)
        val deltaTime = node.getConcentration(SimpleMolecule("DeltaTime")) as? Double
            ?: (1.0 / reaction.timeDistribution.rate)
        return speedMagnitude * deltaTime
    }
}
