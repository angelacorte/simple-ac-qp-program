package it.unibo.collektive.solver.gurobi

import it.unibo.alchemist.collektive.device.CollektiveDevice
import it.unibo.collektive.admm.Tolerance

/**
 * Centralized tuning parameters for the ADMM QP solver.
 *
 * Controls: [rhoSlack] (default slack weight), [rhoADMM] (ADMM penalty),
 * [rhoResidual] (residual balancing), [tolerance] (primal and dual residual thresholds),
 * and [logEnabled] (enable solver logging).
 *
 * Per-iteration values such as the simulation delta-time are intentionally kept out of this class
 * so a cached solver can safely reuse the same settings for the whole environment.
 */
data class QpSettings(
    val constraintPrefix: String = "qp",
    val logEnabled: Boolean = false,
    val rhoADMM: Double = 10.0,
    val rhoResidual: Double = 0.5,
    val rhoSlack: Double = 2.0,
    val tolerance: Tolerance = Tolerance(DEFAULT_TOLERANCE, DEFAULT_TOLERANCE),
) {
    /**
     * Given a [CollektiveDevice], creates a new [QpSettings]
     * instance with parameters overridden by device properties if present.
     */
    fun base(device: CollektiveDevice<*>): QpSettings = copy(
        logEnabled = device["LogEnabled"] as? Boolean ?: logEnabled,
        rhoADMM = device["RhoADMM"] as? Double ?: rhoADMM,
        rhoResidual = device["RhoResidual"] as? Double ?: rhoResidual,
        rhoSlack = device["RhoSlack"] as? Double ?: rhoSlack,
        tolerance = Tolerance(
            primal = (device["PrimalTolerance"] as? Double) ?: tolerance.primal,
            dual = (device["DualTolerance"] as? Double) ?: tolerance.dual,
        ),
    )

    /**
     * Companion object for [QpSettings].
     */
    companion object {
        /**
         * Default value for residual tolerance.
         */
        const val DEFAULT_TOLERANCE: Double = 1e-3
    }
}
