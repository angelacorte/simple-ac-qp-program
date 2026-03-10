package it.unibo.collektive.qp.dsl

import it.unibo.alchemist.collektive.device.CollektiveDevice
import it.unibo.alchemist.model.positions.Euclidean2DPosition
import it.unibo.collektive.aggregate.api.Aggregate
import it.unibo.collektive.alchemist.device.sensors.LocationSensor

/**
 * Main aggregate entrypoint: runs distributed ADMM to compute a safe control and applies it when converged.
 */
fun Aggregate<Int>.entrypointOwner(position: LocationSensor, device: CollektiveDevice<Euclidean2DPosition>) =
    entrypointWithMode(position, device, withOwner = true)
