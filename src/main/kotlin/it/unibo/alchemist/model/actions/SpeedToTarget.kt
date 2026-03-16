package it.unibo.alchemist.model.actions

import it.unibo.alchemist.model.Action
import it.unibo.alchemist.model.Environment
import it.unibo.alchemist.model.Node
import it.unibo.alchemist.model.Position
import it.unibo.alchemist.model.Reaction
import it.unibo.alchemist.model.movestrategies.routing.StraightLine
import it.unibo.alchemist.model.movestrategies.target.SpeedFromMolecule
import it.unibo.alchemist.model.movestrategies.target.TakeTargetFromMolecule

class SpeedToTarget<T, P: Position<P>>(
    environment: Environment<T, P>,
    node: Node<T>,
    reaction: Reaction<T>,
): AbstractConfigurableMoveNode<T, P>(
    environment,
    node,
    StraightLine<T,P>(),
    TakeTargetFromMolecule(environment, node),
    SpeedFromMolecule(node, reaction),
) {

    override fun cloneAction(
        p0: Node<T?>,
        p1: Reaction<T>,
    ): Action<T> = SpeedToTarget(environment, node, p1)

    override fun interpolatePositions(current: P?, target: P?, maxWalk: Double): P? =
        this.nextPosition
}
