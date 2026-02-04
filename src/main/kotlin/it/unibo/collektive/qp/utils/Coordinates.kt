package it.unibo.collektive.qp.utils

interface Vector2D {
    val x: Double
    val y: Double
    val position: Vector2D
        get() = this
    val dimension: Int
        get() = 2
}

data class Obstacle(override val x: Double, override val y: Double, val radius: Double, val margin: Double) : Vector2D

data class Robot<ID>(
    override val x: Double,
    override val y: Double,
    val id: ID,
    val safeMargin: Double,
    val velocity: SpeedControl2D = SpeedControl2D(0.0, 0.0),
    val maxSpeed: Double = Double.MAX_VALUE,
) : Vector2D

data class Target(override val x: Double, override val y: Double, val id: Number) : Vector2D

data class Coordinate(override val x: Double, override val y: Double) : Vector2D

data class SpeedControl2D(override val x: Double, override val y: Double) : Vector2D

operator fun Vector2D.plus(other: Vector2D): Vector2D = Coordinate(x + other.x, y + other.y)

operator fun Vector2D.minus(other: Vector2D): Vector2D = Coordinate(x - other.x, y - other.y)

/**
 * Converts a 2D geometric point into a constant vector.
 *
 * This adapter allows geometric data structures to be directly used
 * within the optimization DSL.
 */
fun Vector2D.toDoubleArray(): DoubleArray = doubleArrayOf(this.x, this.y)

