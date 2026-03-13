package it.unibo.collektive.solver.gurobi

import it.unibo.collektive.admm.Tolerance

/**
 * Centralized tuning parameters for the ADMM QP solver.
 *
 * Controls: [rhoSlack] (default slack weight), [rhoADMM] (ADMM penalty),
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
    val tolerance: Tolerance = Tolerance(1e-3, 1e-3),
)
