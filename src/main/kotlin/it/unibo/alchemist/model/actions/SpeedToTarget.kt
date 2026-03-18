package it.unibo.alchemist.model.actions

import it.unibo.alchemist.model.Action
import it.unibo.alchemist.model.Environment
import it.unibo.alchemist.model.Node
import it.unibo.alchemist.model.Position
import it.unibo.alchemist.model.Reaction
import it.unibo.alchemist.model.molecules.SimpleMolecule
import it.unibo.alchemist.model.movestrategies.routing.StraightLine
import it.unibo.alchemist.model.movestrategies.target.SpeedFromMolecule
import it.unibo.alchemist.model.movestrategies.target.TakeTargetFromMolecule
import it.unibo.collektive.model.zeroSpeed

class SpeedToTarget<T, P : Position<P>>(
    environment: Environment<T, P>,
    node: Node<T>,
    reaction: Reaction<T>
) : AbstractConfigurableMoveNode<T, P>(
    environment,
    node,
    StraightLine<T, P>(),
    TakeTargetFromMolecule(environment, node),
    SpeedFromMolecule(node, reaction),
    true,
) {
    override fun cloneAction(node: Node<T>, reaction: Reaction<T>): Action<T> =
        SpeedToTarget(environment, node, reaction)

    override fun interpolatePositions(current: P?, target: P?, maxWalk: Double): P? =
        current?.let { curr ->
            target?.let { goal ->
                val distance = curr.distanceTo(goal)
                when {
                    distance == 0.0 || maxWalk == 0.0 -> goal
                    else -> {
                        environment.makePosition(
                            curr.coordinates.mapIndexed { index, value ->
                                value + (goal.coordinates[index] - value) * maxWalk / distance
                            }
                        ) as P
                    }
                }
            }
        } ?: current
}
