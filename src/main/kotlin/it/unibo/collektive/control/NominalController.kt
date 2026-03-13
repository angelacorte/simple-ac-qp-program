package it.unibo.collektive.control

import it.unibo.collektive.model.Robot
import it.unibo.collektive.model.SpeedControl2D
import it.unibo.collektive.model.Target
import it.unibo.collektive.mathutils.nominal

/**
 * Computes the nominal control for a robot, representing its desired
 * behavior in the absence of constraints (e.g., obstacles or neighbors).
 */
fun interface NominalController {
    fun compute(robot: Robot): SpeedControl2D
}

/**
 * A simple proportional controller driving the robot toward a static target.
 */
class GoToTargetNominal(private val target: Target, private val gain: Double = 0.5) : NominalController {
    override fun compute(robot: Robot): SpeedControl2D =
        robot.position.nominal(target.position, gain)
}
