package it.unibo.collektive.qp.utils

interface Point {
    val x: Double
    val y: Double
    val position: Point
        get() = this
}

data class Obstacle(override val x: Double, override val y: Double, val radius: Double, val margin: Double) : Point

data class Robot(
    override val x: Double,
    override val y: Double,
    val id: Number,
    val margin: Double,
    val velocity: SpeedControl2D = SpeedControl2D(0.0, 0.0),
    val maxSpeed: Double = Double.MAX_VALUE,
) : Point

data class Target(override val x: Double, override val y: Double, val id: Number) : Point

data class Coordinate(override val x: Double, override val y: Double) : Point

data class SpeedControl2D(override val x: Double, override val y: Double) : Point

operator fun Point.plus(other: Point): Point = Coordinate(x + other.x, y + other.y)
