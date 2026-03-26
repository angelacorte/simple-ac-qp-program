package it.unibo.collektive.solver

import com.github.benmanes.caffeine.cache.Caffeine
import com.github.benmanes.caffeine.cache.LoadingCache
import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBEnv
import com.gurobi.gurobi.GRBModel
import it.unibo.alchemist.model.Environment
import it.unibo.collektive.admm.DualParams
import it.unibo.collektive.admm.IncidentDuals
import it.unibo.collektive.admm.SuggestedControl
import it.unibo.collektive.control.cbf.CBF
import it.unibo.collektive.control.clf.CLF
import it.unibo.collektive.model.Robot
import it.unibo.collektive.model.SpeedControl2D
import it.unibo.collektive.solver.gurobi.LocalQP
import it.unibo.collektive.solver.gurobi.PairwiseQP
import it.unibo.collektive.solver.gurobi.QpSettings
import it.unibo.collektive.solver.gurobi.setLicense
import it.unibo.collektive.solver.gurobi.setupLogger

class Solver(
    val settings: QpSettings,
) {

    private lateinit var local: LocalQP

    private lateinit var pairwise: PairwiseQP

    private val env: GRBEnv = setLicense().let {
        GRBEnv(true).also {
            it.set(GRB.IntParam.OutputFlag, if (settings.logEnabled) 1 else 0)
            it.start()
        }
    }

    val isLocalModelAvailable: Boolean get() = this::local.isInitialized

    val isPairwiseModelAvailable: Boolean get() = this::pairwise.isInitialized

    fun setupLocalModel(
        robot: Robot,
        localCLFs: List<CLF>,
        localCBFs: List<CBF>,
    ) {
        if (!isLocalModelAvailable) {
            val model = GRBModel(env).also { if (settings.logEnabled) it.setupLogger() }
            local = LocalQP.create(model, robot, localCLFs, localCBFs)
        }
    }

    fun setupPairwiseModel(
        robot: Robot,
        otherRobot: Robot,
        pairwiseCBFs: List<CBF>,
    ) {
        if (!isPairwiseModelAvailable) {
            val model = GRBModel(env).also { if (settings.logEnabled) it.setupLogger() }
            pairwise = PairwiseQP.create(model, robot, otherRobot, pairwiseCBFs)
        }
    }

    fun <ID: Comparable<ID>> updateAndSolveLocal(
        robot: Robot,
        uNominal: DoubleArray,
        duals: Map<ID, DualParams>,
        deltaTime: Double,
    ): SpeedControl2D = local.updateAndSolve(robot, uNominal, duals, settings, deltaTime)

    fun updateAndSolvePairwise(
        robot: Robot,
        otherRobot: Robot,
        duals: IncidentDuals,
        deltaTime: Double,
    ): SuggestedControl = pairwise.updateAndSolve(robot, otherRobot, duals, settings, deltaTime)

}

object SimulationSolver {
    private val activeSolver: LoadingCache<Environment<*, *>, Solver> = Caffeine.newBuilder()
        .weakKeys()
        .build { key -> Solver(QpSettings()) }

    val Environment<*, *>.solver: Solver get() = activeSolver.getIfPresent(this) ?: error("Could not find solver for $this")

    fun Environment<*, *>.solver(settings: QpSettings): Solver =
        activeSolver.getIfPresent(this) ?: Solver(settings).also { activeSolver.put(this, it) }
}
