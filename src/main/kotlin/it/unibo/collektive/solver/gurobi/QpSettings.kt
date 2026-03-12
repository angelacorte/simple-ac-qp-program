package it.unibo.collektive.solver.gurobi

/**
 * Centralized tuning parameters for QP/CBF formulations.
 *
 * Controls: [rhoSlack] (slack weight, linear/quadratic via [slackQuadratic]), [rhoADMM] (ADMM penalty),
 * [gammaCollision]/[gammaComm]/[gammaObstacle] (CBF gains), [convergenceRate] (CLF rate),
 * [logEnabled] (enable solver logging), [constraintPrefix] (constraint naming prefix),
 * [slackQuadratic] (true for quadratic slack penalty, false for linear).
 */
data class QpSettings(
    val rhoSlack: Double = 2.0,
    val rhoCommSlack: Double = 2.0,
    val rhoADMM: Double = 10.0,
    val gammaCollision: Double = 0.5,
    val gammaComm: Double = 0.1,
    val gammaObstacle: Double = 0.5,
    val convergenceRate: Double = 1.0,
    val logEnabled: Boolean = false,
    val constraintPrefix: String = "qp",
    val slackQuadratic: Boolean = false,
    val deltaTime: Double = 0.01, /** Default discrete time-step in seconds (e.g. 100 events/s → ∆t = 0.01 s). */
)
