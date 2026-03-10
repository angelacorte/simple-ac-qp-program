package it.unibo.collektive.qp.controlFunctions

import com.gurobi.gurobi.GRBModel
import it.unibo.collektive.control.cbf.CollisionCbf
import it.unibo.collektive.control.cbf.CommunicationRangeCbf
import it.unibo.collektive.control.cbf.ObstacleCbf
import it.unibo.collektive.solver.gurobi.QpSettings
import it.unibo.collektive.solver.gurobi.GRBVector
import it.unibo.collektive.model.Obstacle
import it.unibo.collektive.model.Robot

/**
 * Context for CBF builders carrying [self], optional [other]/[obstacle],
 * optional [communicationRange], and solver [settings].
 */
data class CbfContext(
    val self: Robot,
    val other: Robot? = null,
    val obstacle: Obstacle? = null,
    val communicationRange: Double? = null,
    val settings: QpSettings = QpSettings(),
)

/**
 * Pluggable barrier builder identified by [name] and exposing [add] to inject constraints.
 */
interface Cbf {
    val name: String

    /** Adds this CBF to [model] using local variables [uSelf], optional neighbor variables [uOther], and [ctx]. */
    fun add(model: GRBModel, uSelf: GRBVector, uOther: GRBVector?, ctx: CbfContext)
}

/** Simple registry to collect active barriers. */
object CbfRegistry {
    private val cbfs = mutableListOf<Cbf>()
    private var defaultsRegistered = false

    /** Registers [cbf] if another with the same [Cbf.name] is not present. */
    fun register(cbf: Cbf) {
        if (cbfs.none { it.name == cbf.name }) cbfs += cbf
    }

    /** Returns all registered barriers, auto-registering defaults once. */
    fun all(): List<Cbf> {
        if (!defaultsRegistered) {
            register(ObstacleCbf)
            register(CollisionCbf)
            register(CommunicationRangeCbf)
            defaultsRegistered = true
        }
        return cbfs.toList()
    }
}

/**
 * Apply all registered CBFs for a single-agent (local) problem.
 */
fun applyLocalCbfs(model: GRBModel, u: GRBVector, ctx: CbfContext) =
    CbfRegistry.all().forEach { it.add(model, u, null, ctx) }

/**
 * Apply all registered CBFs for a pairwise problem.
 */
fun applyPairwiseCbfs(model: GRBModel, ui: GRBVector, uj: GRBVector, ctx: CbfContext) =
    CbfRegistry.all().forEach { it.add(model, ui, uj, ctx) }
