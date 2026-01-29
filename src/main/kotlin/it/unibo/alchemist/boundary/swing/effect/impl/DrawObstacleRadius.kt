@file:Suppress("DEPRECATION")

package it.unibo.alchemist.boundary.swingui.effect.impl

import it.unibo.alchemist.boundary.swingui.effect.api.Effect
import it.unibo.alchemist.boundary.ui.api.Wormhole2D
import it.unibo.alchemist.model.Environment
import it.unibo.alchemist.model.Node
import it.unibo.alchemist.model.Position2D
import it.unibo.alchemist.model.Time
import it.unibo.alchemist.model.molecules.SimpleMolecule
import java.awt.BasicStroke
import java.awt.Color
import java.awt.Graphics2D
import java.awt.Point
import kotlin.math.abs
import kotlin.math.nextUp

class DrawObstacleRadius : Effect {
    override fun getColorSummary(): Color = Color.BLACK

    @Transient
    var lastUpdated = Time.NEGATIVE_INFINITY

    override fun <T : Any?, P : Position2D<P>> apply(
        graphics: Graphics2D,
        node: Node<T>,
        environment: Environment<T, P>,
        wormhole: Wormhole2D<P>,
    ) {
        if (environment.simulation.time != lastUpdated) {
            lastUpdated = environment.simulation.time
        }
        runCatching {
            environment.getPosition(node)
        }.onSuccess { nodePosition ->
            val viewPoint = wormhole.getViewPoint(nodePosition)
            val isRobot = node.contains(robot)
            val margin = node.getConcentration(safeMargin).toDouble()
            val isObstacle = node.contains(obstacle)
            val radius: Double = (if (node.contains(safeRadius)) node.getConcentration(safeRadius) else 0.0) as Double
            val size: Double =
                when {
                    isRobot -> margin
                    isObstacle -> radius + margin
                    else -> 0.0.nextUp()
                }
            val sizeAsPosition: P = environment.makePosition(size, size)
            val sizeFromLocation = sizeAsPosition + nodePosition.coordinates
            val sizeInScreenCoordinates =
                (wormhole.getViewPoint(sizeFromLocation) - viewPoint)
                    .let { Point(abs(it.x), abs(it.y)) }
                    .takeIf { it.x > MIN_NODE_SIZE && it.y > MIN_NODE_SIZE }
                    ?: Point(MIN_NODE_SIZE, MIN_NODE_SIZE)
            val boundingBoxSize = sizeInScreenCoordinates / 2
            val boundingBox =
                listOf(
                    viewPoint + boundingBoxSize,
                    viewPoint + boundingBoxSize.mirrorX(),
                    viewPoint + boundingBoxSize.mirrorY(),
                    viewPoint - boundingBoxSize,
                )
            if (boundingBox.any { wormhole.isInsideView(it) }) {
                if(isRobot) {
                    graphics.color = Color.GRAY//hsbColor(60f, alpha = 0.3f)
                    graphics.drawOval(
                        viewPoint.x - sizeInScreenCoordinates.x / 2,
                        viewPoint.y - sizeInScreenCoordinates.y / 2,
                        sizeInScreenCoordinates.x,
                        sizeInScreenCoordinates.y,
                    )
                }
                if (isObstacle) {
                    graphics.color = Color.ORANGE//hsbColor(30f, alpha = 0.4f)
                    graphics.drawOval(
                        viewPoint.x - sizeInScreenCoordinates.x,
                        viewPoint.y - sizeInScreenCoordinates.y,
                        2 * sizeInScreenCoordinates.x,
                        2 * sizeInScreenCoordinates.y,
                    )
                    val innerSizeAsPosition: P = environment.makePosition(radius, radius)
                    val innerSizeFromLocation = innerSizeAsPosition + nodePosition.coordinates
                    val innerSizeScreen =
                        (wormhole.getViewPoint(innerSizeFromLocation) - viewPoint)
                            .let { Point(abs(it.x), abs(it.y)) }
                            .takeIf { it.x > MIN_NODE_SIZE && it.y > MIN_NODE_SIZE }
                            ?: Point(MIN_NODE_SIZE, MIN_NODE_SIZE)

                    graphics.color = hsbColor(0f, alpha = 0.6f)
//                    graphics.color = Color.RED//hsbColor(0f, alpha = 0.5f)
                    graphics.fillOval(
                        viewPoint.x - innerSizeScreen.x,
                        viewPoint.y - innerSizeScreen.y,
                        2 * innerSizeScreen.x,
                        2 * innerSizeScreen.y,
                    )
                }
            }
        }
    }

    companion object {
        val robot = SimpleMolecule("Robot")
        val safeMargin = SimpleMolecule("SafeMargin")
        val obstacle = SimpleMolecule("Obstacle")
        val safeRadius = SimpleMolecule("SafeRadius")
        const val MIN_NODE_SIZE = 1

        private fun Any?.toInt(): Int? =
            when (this) {
                is Int -> this
                is Number -> this.toInt()
                is String -> this.toInt()
                null -> null
                Unit -> null
                else -> error("Unexpected integer: $this")
            }

        private fun Any?.toDouble(): Double =
            when (this) {
                is Double -> this
                is Number -> this.toDouble()
                null -> 0.0
                Unit -> 0.0
                else -> error("Unexpected integer: $this")
            }

        private operator fun Point.plus(other: Point): Point = Point(x + other.x, y + other.y)

        private operator fun Point.minus(other: Point): Point = Point(x - other.x, y - other.y)

        private operator fun Point.times(factor: Int): Point = Point((x * factor), (y * factor))

        private operator fun Point.div(factor: Int): Point = Point((x / factor), (y / factor))

        private fun Point.mirrorX(): Point = Point(-x, y)

        private fun Point.mirrorY(): Point = Point(x, -y)

        fun hsbColor(
            hueDeg: Float,
            alpha: Float,
            saturation: Float = 1f,
            brightness: Float = 1f
        ): Color {
            val rgb = Color.HSBtoRGB(hueDeg / 360f, saturation, brightness)
            return Color(
                (rgb shr 16) and 0xFF,
                (rgb shr 8) and 0xFF,
                rgb and 0xFF,
                (alpha * 255).toInt()
            )
        }
    }
}
