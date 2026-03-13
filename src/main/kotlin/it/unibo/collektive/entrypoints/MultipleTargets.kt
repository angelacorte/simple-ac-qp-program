package it.unibo.collektive.entrypoints

import it.unibo.alchemist.collektive.device.CollektiveDevice
import it.unibo.alchemist.model.positions.Euclidean2DPosition
import it.unibo.collektive.admm.Tolerance
import it.unibo.collektive.admm.controlLoop
import it.unibo.collektive.aggregate.api.Aggregate
import it.unibo.collektive.alchemist.device.applyControl
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
import it.unibo.collektive.stdlib.time.localDeltaTime
import kotlin.time.DurationUnit

fun Aggregate<Int>.multipleTargetEntrypoint(
    position: LocationSensor,
    timeSensor: TimeSensor,
    device: CollektiveDevice<Euclidean2DPosition>,
) = context(position, device) {
    val maxIter: Int = device["MaxIterations"]
    val tolerance = Tolerance(device["PrimalTolerance"], device["DualTolerance"])
    val robot = getRobot()
    val target: Target = getTarget(device["TargetID"] as Number)
    val obstacle = getObstacle()
    val timeDistribution: Double = device["TimeDistribution"]
    val deltaTime: Double =
        localDeltaTime(timeSensor.getTimeAsInstant()).toDouble(DurationUnit.SECONDS)
            .takeIf { it > 0.0 } ?: (1.0 / timeDistribution)
    val uNominal = GoToTargetNominal(target).compute(robot).toDoubleArray()
    val res = controlLoop(
        robot = robot,
        uNominal = uNominal,
        maxIter = maxIter,
        settings = QpSettings(deltaTime = deltaTime, tolerance = tolerance),
        localCLF = listOf(GoToTargetCLF(target)),
        localCBFs = listOf(ObstacleAvoidanceCBF(obstacle), MaxSpeedCBF()),
        pairwiseCBFs = listOf(
            CollisionAvoidanceCBF(),
        ),
    )
    if (res.first) robot.applyControl(res.second, deltaTime) // stop if residuals < threshold
}
