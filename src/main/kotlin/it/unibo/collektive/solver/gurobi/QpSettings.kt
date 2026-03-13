package it.unibo.collektive.solver.gurobi

/**
 * Centralized tuning parameters for the ADMM QP solver.
 *
 * Controls: [rhoSlack] (default slack weight), [rhoADMM] (ADMM penalty),
 * [logEnabled] (enable solver logging), [constraintPrefix] (constraint naming prefix),
 * and [deltaTime] (discrete time-step).
 */
data class QpSettings(
    val rhoSlack: Double = 2.0,
    val rhoADMM: Double = 10.0,
    val logEnabled: Boolean = false,
    val constraintPrefix: String = "qp",
    val deltaTime: Double = 0.01,
)
