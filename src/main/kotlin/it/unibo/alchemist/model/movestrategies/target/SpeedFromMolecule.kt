package it.unibo.alchemist.model.movestrategies.target

import it.unibo.alchemist.model.Node
import it.unibo.alchemist.model.Position
import it.unibo.alchemist.model.Reaction
import it.unibo.alchemist.model.molecules.SimpleMolecule
import it.unibo.alchemist.model.movestrategies.SpeedSelectionStrategy
import it.unibo.collektive.mathutils.norm
import it.unibo.collektive.model.SpeedControl2D
import it.unibo.collektive.model.zeroSpeed

class SpeedFromMolecule<T, P: Position<P>>(private val node: Node<T>, private val reaction: Reaction<*>): SpeedSelectionStrategy<T, P> {
    override fun getNodeMovementLength(target: P?): Double {
        val speed = node.getConcentration(SimpleMolecule("Velocity")) as? SpeedControl2D ?: zeroSpeed()
        return speed.norm() / reaction.rate
    }
}
