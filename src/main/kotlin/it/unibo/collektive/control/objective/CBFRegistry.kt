package it.unibo.collektive.control.objective

import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.control.cbf.CollisionAvoidanceCBF
import it.unibo.collektive.control.cbf.CommunicationRangeCBF
import it.unibo.collektive.control.cbf.ObstacleCBF
import it.unibo.collektive.model.Obstacle
import it.unibo.collektive.model.Robot
import it.unibo.collektive.solver.gurobi.GRBVector
import it.unibo.collektive.solver.gurobi.QpSettings

/**
 * Context for CBF builders carrying [self], optional [other]/[obstacle],
 * optional [communicationRange] and relative slack [commSlack], and solver [settings].
 */
data class CBFContext(
    val self: Robot,
    val other: Robot? = null,
    val obstacle: Obstacle? = null,
    val communicationRange: Double? = null,
    val commSlack: GRBVar? = null,
    val settings: QpSettings = QpSettings(),
)

/**
 * Pluggable barrier builder identified by [name] and exposing [add] to inject constraints.
 */
interface CBF {
    val name: String

    /** Adds this CBF to [model] using local variables [uSelf], optional neighbor variables [uOther], and [ctx]. */
    fun add(model: GRBModel, uSelf: GRBVector, uOther: GRBVector?, ctx: CBFContext)
}

/** Simple registry to collect active barriers. */
object CBFRegistry {
    private val CBFS = mutableListOf<CBF>()

    /** Registers [cbf] if another with the same [CBF.name] is not present. */
    fun register(cbf: CBF) {
        if (CBFS.none { it.name == cbf.name }) CBFS += cbf
    }

    /** Returns all registered barriers. */
    fun all(): List<CBF> = CBFS.toList()

    /** Default CBF set to opt into explicitly. */
    fun defaults(): List<CBF> = listOf(ObstacleCBF, CollisionAvoidanceCBF, CommunicationRangeCBF)
}

/**
 * Apply all registered CBFs for a single-agent (local) problem.
 */
fun applyLocalCBFs(model: GRBModel, u: GRBVector, ctx: CBFContext, cbfs: List<CBF>) =
    cbfs.forEach { it.add(model, u, null, ctx) }

/**
 * Apply all registered CBFs for a pairwise problem.
 */
fun applyPairwiseCBFs(model: GRBModel, ui: GRBVector, uj: GRBVector, ctx: CBFContext, cbfs: List<CBF>) =
    cbfs.forEach { it.add(model, ui, uj, ctx) }
