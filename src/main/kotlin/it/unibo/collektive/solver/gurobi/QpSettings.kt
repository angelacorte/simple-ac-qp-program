package it.unibo.collektive.solver.gurobi

import it.unibo.collektive.admm.Tolerance

/**
 * Centralized tuning parameters for the ADMM QP solver.
 *
 * Controls: [rhoSlack] (default slack weight), [rhoADMM] (ADMM penalty),
 * [rhoResidual] (residual balancing), [tolerance] (primal and dual residual thresholds),
 * [logEnabled] (enable solver logging), [constraintPrefix] (constraint naming prefix),
 * and [deltaTime] (discrete time-step).
 */
data class QpSettings(
    val constraintPrefix: String = "qp",
    val deltaTime: Double = 0.01,
    val logEnabled: Boolean = false,
    val rhoADMM: Double = 10.0,
    val rhoResidual: Double = 0.5,
    val rhoSlack: Double = 2.0,
    val tolerance: Tolerance = Tolerance(DEFAULT_TOLERANCE, DEFAULT_TOLERANCE),
) {
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
