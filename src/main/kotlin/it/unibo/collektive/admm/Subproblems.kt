package it.unibo.collektive.admm

import it.unibo.collektive.control.ControlFunctionContext
import it.unibo.collektive.control.cbf.CBF
import it.unibo.collektive.control.clf.CLF
import it.unibo.collektive.model.Robot
import it.unibo.collektive.model.SpeedControl2D
import it.unibo.collektive.solver.gurobi.LocalQP
import it.unibo.collektive.solver.gurobi.PairwiseQP
import it.unibo.collektive.solver.gurobi.QpSettings

// Using thread-local storage ensures that parallel simulation instances (each running on its own thread)
// maintain independent caches and never share a Gurobi model across threads.
private val localTemplates: ThreadLocal<MutableMap<Any, LocalQP>> =
    ThreadLocal.withInitial { mutableMapOf() }

private val pairwiseTemplates: ThreadLocal<MutableMap<Any, PairwiseQP>> =
    ThreadLocal.withInitial { mutableMapOf() }

/**
 * Solves the local ADMM QP for the agent identified by [selfId].
 *
 * On the first call for a given [selfId], a [LocalQP] is created via
 * [LocalQP.create] (which adds all variables and constraints once).
 * On every subsequent call the cached template is reused: only numerical parameters
 * (bounds, RHS, coefficients, objective) are refreshed before calling [GRBModel.optimize].
 *
 * @param selfId    unique, stable identifier for the calling robot (used as cache key)
 * @param robot     current robot state
 * @param uNominal  nominal control input
 * @param duals     current per-neighbour dual variables
 * @param settings  solver settings
 * @param localCLFs CLF list — must have the **same length** on every call for a given [selfId]
 * @param localCBFs CBF list — same length requirement as [localCLFs]
 */
fun <ID : Comparable<ID>> solveLocalQP(
    selfId: ID,
    robot: Robot,
    uNominal: DoubleArray,
    duals: Map<ID, DualParams>,
    settings: QpSettings = QpSettings(),
    localCLFs: List<CLF>,
    localCBFs: List<CBF> = emptyList(),
): SpeedControl2D {
    val template = localTemplates.get().getOrPut(selfId as Any) {
        LocalQP.create(robot, localCLFs, localCBFs, settings)
    }
    val context = ControlFunctionContext(self = robot, settings = settings)
    return template.solve(robot, uNominal, duals, context, localCLFs, localCBFs)
}

/**
 * Solves the pairwise ADMM QP for the directed edge `(selfId, neighborId)`.
 *
 * A [PairwiseQP] is created on cache miss and reused on subsequent calls for the same edge.
 * The template is indexed by the pair `(selfId, neighborId)`; new neighbours get a fresh template
 * automatically.
 *
 * @param selfId       this agent's stable identifier
 * @param neighborId   the neighbour's stable identifier
 * @param robot        this agent's current state
 * @param other        the neighbour's current state
 * @param incidentDuals current dual variables for this edge
 * @param settings     solver settings
 * @param pairwiseCBFs CBF list — must have the **same length** on every call for a given edge key
 */
fun <ID : Comparable<ID>> solvePairwiseQP(
    selfId: ID,
    neighborId: ID,
    robot: Robot,
    other: Robot,
    incidentDuals: IncidentDuals,
    settings: QpSettings = QpSettings(),
    pairwiseCBFs: List<CBF> = emptyList(),
): SuggestedControl {
    val template = pairwiseTemplates.get().getOrPut(selfId to neighborId) {
        PairwiseQP.create(robot, other, pairwiseCBFs, settings)
    }
    val context = ControlFunctionContext(self = robot, otherRobot = other, settings = settings)
    return template.solve(robot, other, incidentDuals, context, pairwiseCBFs)
}
