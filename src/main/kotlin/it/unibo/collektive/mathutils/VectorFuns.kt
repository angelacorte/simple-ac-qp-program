package it.unibo.collektive.mathutils

import it.unibo.collektive.model.Coordinate
import it.unibo.collektive.model.SpeedControl2D
import it.unibo.collektive.model.Vector2D

/**
 * Operator to sum a given [it.unibo.collektive.model.SpeedControl2D] with an [other].
 */
operator fun SpeedControl2D.plus(other: SpeedControl2D): SpeedControl2D = SpeedControl2D(x + other.x, y + other.y)

/**
 * Operator to divide a given [SpeedControl2D] with an [div].
 */
operator fun SpeedControl2D.div(div: Double) = SpeedControl2D(x / div, y / div)

/**
 * Operator to scale a [SpeedControl2D] by a scalar [factor].
 */
operator fun SpeedControl2D.times(factor: Double) = SpeedControl2D(x * factor, y * factor)

/**
 * Operator to sum a given [it.unibo.collektive.model.Vector2D] with an [other].
 */
operator fun Vector2D.plus(other: Vector2D): Vector2D = Coordinate(x + other.x, y + other.y)

/**
 * Operator to sub a given [Vector2D] with an [other].
 */
operator fun Vector2D.minus(other: Vector2D): Vector2D = Coordinate(x - other.x, y - other.y)

/**
 * Squared Euclidean norm of the vector.
 */
fun Vector2D.norm(): Double = x * x + y * y

/**
 * Proportional controller driving the point toward [pGoal] with gain [controlGain].
 */
fun Vector2D.nominal(pGoal: Vector2D, controlGain: Double = 0.5): SpeedControl2D {
    val error = this - pGoal
    return SpeedControl2D(-controlGain * error.x, -controlGain * error.y)
}

/**
 * Converts a 2D geometric point into a constant vector.
 *
 * This adapter allows geometric data structures to be directly used
 * within the optimization DSL.
 */
fun Vector2D.toDoubleArray(): DoubleArray = doubleArrayOf(this.x, this.y)

/**
 * Operator to Multiply a given [Vector2D] with an [other].
 */
operator fun Vector2D.times(other: Vector2D): Double = x * other.x + y * other.y

/**
 * Operator to Multiply a given scalar with an [other] [Vector2D].
 */
operator fun Double.times(other: Vector2D): Vector2D = Coordinate(this * other.x, this * other.y)
