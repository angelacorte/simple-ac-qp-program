package it.unibo.collektive.qp.utils

interface Point2D {
    val x: Double
    val y: Double
    val position: Point2D
        get() = this
    val dimension: Int
        get() = 2
}

data class Obstacle(override val x: Double, override val y: Double, val radius: Double, val margin: Double) : Point2D

data class Robot(
    override val x: Double,
    override val y: Double,
    val id: Number,
    val margin: Double,
    val velocity: SpeedControl2D = SpeedControl2D(0.0, 0.0),
    val maxSpeed: Double = Double.MAX_VALUE,
) : Point2D

data class Target(override val x: Double, override val y: Double, val id: Number) : Point2D

data class Coordinate(override val x: Double, override val y: Double) : Point2D

data class SpeedControl2D(override val x: Double, override val y: Double) : Point2D

operator fun Point2D.plus(other: Point2D): Point2D = Coordinate(x + other.x, y + other.y)

operator fun Point2D.minus(other: Point2D): Point2D = Coordinate(x - other.x, y - other.y)

/**
 * Converts a 2D geometric point into a constant vector.
 *
 * This adapter allows geometric data structures to be directly used
 * within the optimization DSL.
 */
fun Point2D.toDoubleArray(): DoubleArray = doubleArrayOf(this.x, this.y)
