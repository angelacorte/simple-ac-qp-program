package it.unibo.collektive.control.clf

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.model.Target
import it.unibo.collektive.solver.gurobi.ConstraintNames
import it.unibo.collektive.solver.gurobi.GRBVector
import it.unibo.collektive.solver.gurobi.QpSettings
import it.unibo.collektive.solver.gurobi.toLinExpr
import it.unibo.collektive.model.minus
import it.unibo.collektive.model.squaredNorm
import it.unibo.collektive.model.toDoubleArray
import kotlin.math.pow

/**
 * Discrete-time CLF (DCLF) constraint for goal reaching under ZOH dynamics.
 *
 * From the decrease template V_{k+1} ≤ (1−λ)V_k + δ with V_k = ‖e_k‖²
 * and the upper bound ‖u_k‖² ≤ u²_max, the affine sufficient constraint is:
 *
 *     2∆t eᵀ u − δ  ≤  −λ ‖e‖²  −  ∆t² u²_max
 *
 * where e = p − p_g, λ = [QpSettings.convergenceRate], ∆t = [QpSettings.deltaTime].
 */
fun GRBModel.goToTargetCLF(
    target: Target,
    position: DoubleArray,
    u: GRBVector,
    delta: GRBVar,
    maxSpeed: Double,
    settings: QpSettings = QpSettings(),
) {
    require(settings.deltaTime.isFinite() && settings.deltaTime > 0.0) {
        "deltaTime must be finite and greater than zero to build DCLF constraint"
    }
    require(maxSpeed.isFinite() && maxSpeed >= 0.0) { "maxSpeed must be finite and non-negative" }
    require(target.x.isFinite() && target.y.isFinite()) { "Target coordinates must be finite" }
    require(position.all { it.isFinite() }) { "Position coordinates must be finite" }
    val pg = target.position.toDoubleArray()
    val distanceVec = position - pg
    val dt = settings.deltaTime
    // left side: 2∆t eᵀ u − δ
    val left = u.toLinExpr(distanceVec, 2.0 * dt)
    left.addTerm(-1.0, delta)
    // right side: −λ ‖e‖² − ∆t² u²_max
    val right = -settings.convergenceRate * distanceVec.squaredNorm() - dt.pow(2) * maxSpeed.pow(2)
    addConstr(left, GRB.LESS_EQUAL, right, ConstraintNames.clf(target.id.toString()))
    addConstr(delta, GRB.GREATER_EQUAL, 0.0, ConstraintNames.slack(target.id.toString()))
}
