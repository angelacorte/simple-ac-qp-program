package it.unibo.collektive.entrypoints

import it.unibo.alchemist.collektive.device.CollektiveDevice
import it.unibo.alchemist.model.positions.Euclidean2DPosition
import it.unibo.collektive.admm.admmEntrypoint
import it.unibo.collektive.aggregate.api.Aggregate
import it.unibo.collektive.alchemist.device.getObstacle
import it.unibo.collektive.alchemist.device.getRobot
import it.unibo.collektive.alchemist.device.getTarget
import it.unibo.collektive.alchemist.device.sensors.LocationSensor
import it.unibo.collektive.alchemist.device.sensors.TimeSensor
import it.unibo.collektive.control.GoToTargetNominal
import it.unibo.collektive.control.cbf.CollisionAvoidanceCBF
import it.unibo.collektive.control.cbf.MaxSpeedCBF
import it.unibo.collektive.control.cbf.ObstacleAvoidanceCBF
import it.unibo.collektive.control.clf.GoToTargetCLF
import it.unibo.collektive.mathutils.toDoubleArray
import it.unibo.collektive.model.Target
import it.unibo.collektive.solver.gurobi.QpSettings
import kotlin.Double
import kotlin.Int

/**
 * Multiple Targets simulation entrypoint.
 */
fun Aggregate<Int>.multipleTargetEntrypoint(
    position: LocationSensor,
    timeSensor: TimeSensor,
    device: CollektiveDevice<Euclidean2DPosition>,
) = context(position, device, timeSensor) {
    val robot = getRobot()
    val target: Target = getTarget(device["TargetID"] as Number)
    admmEntrypoint(
        robot,
        device["TimeDistribution"] as Double? ?: 1.0,
        device["MaxIterations"] as? Int ?: 100,
        uNominal = GoToTargetNominal(target).compute(robot).toDoubleArray(),
        localCLF = listOf(GoToTargetCLF(target)),
        localCBF = listOf(ObstacleAvoidanceCBF(getObstacle()), MaxSpeedCBF()),
        pairwiseCBF = listOf(
            CollisionAvoidanceCBF(),
        ),
        settings = QpSettings().base(device),
    )
}
