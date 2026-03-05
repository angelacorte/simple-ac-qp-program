package it.unibo.collektive.qp.utils

/**
 * 2D [dimension] point-like contract exposing coordinates and
 * a self-referencing [position] for DSLs that expect a `([x], [y])` pair.
 */
interface Vector2D {
    val x: Double
    val y: Double
    val position: Vector2D
        get() = this
    val dimension: Int
        get() = 2
}

/**
 * The [x], [y] coordinates position of an obstacle, and relative [radius] and additional [margin] of danger.
 */
data class Obstacle(override val x: Double, override val y: Double, val radius: Double, val margin: Double) : Vector2D

/**
 * Mobile agent with a control vector and safety envelope.
 * A robot has a [safeMargin], a [control] that represent the current vector of speed, and the [maxSpeed] applicable.
 */
data class Robot(
    override val x: Double,
    override val y: Double,
    val safeMargin: Double,
    val control: SpeedControl2D = SpeedControl2D(0.0, 0.0),
    val maxSpeed: Double = Double.MAX_VALUE,
) : Vector2D

/**
 * Static target used as navigation goal, identified thanks to its [id].
 */
data class Target(override val x: Double, override val y: Double, val id: Number) : Vector2D

/**
 * Basic agnostic [x], [y] coordinates.
 */
data class Coordinate(override val x: Double, override val y: Double) : Vector2D {
    override fun toString(): String {
        return "Coord($x, $y)"
    }
}

/**
 * Planar control input.
 */
data class SpeedControl2D(override val x: Double, override val y: Double) : Vector2D {
    override fun toString(): String {
        return "Control($x, $y)"
    }
}

/**
 * Clamp both control components inside a speed interval.
 */
fun SpeedControl2D.coerceSpeedIn(minSpeed: Double, maxSpeed: Double): SpeedControl2D =
    SpeedControl2D(x.coerceIn(minSpeed, maxSpeed), y.coerceIn(minSpeed, maxSpeed))

/**
 * Operator to sum a given [SpeedControl2D] with an [other].
 */
operator fun SpeedControl2D.plus(other: SpeedControl2D): SpeedControl2D = SpeedControl2D(x + other.x, y + other.y)

/**
 * Operator to divide a given [SpeedControl2D] with an [div].
 */
operator fun SpeedControl2D.div(div: Double) = SpeedControl2D(x / div, y / div)

/**
 * Average a list of control vectors component-wise.
 */
fun List<SpeedControl2D>.avg(): SpeedControl2D =
    fold(SpeedControl2D(0.0, 0.0)) { acc, x -> acc + x }.div(this.size.toDouble())

/**
 * Operator to sum a given [Vector2D] with an [other].
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

/**
 * Zero-initialized position.
 */
fun initVector2D(): Coordinate = Coordinate(0.0, 0.0)

/**
 * Zero control input utility.
 */
fun zeroSpeed(): SpeedControl2D = SpeedControl2D(0.0, 0.0)
