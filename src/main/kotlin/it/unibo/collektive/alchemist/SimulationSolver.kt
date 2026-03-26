package it.unibo.collektive.alchemist

import com.github.benmanes.caffeine.cache.Caffeine
import com.github.benmanes.caffeine.cache.LoadingCache
import it.unibo.alchemist.model.Environment
import it.unibo.collektive.solver.Solver
import it.unibo.collektive.solver.gurobi.QpSettings

object SimulationSolver {
    private val activeSolver: LoadingCache<Environment<*, *>, Solver> = Caffeine.newBuilder()
        .weakKeys()
        .build { key -> Solver(QpSettings()) }

    val Environment<*, *>.solver: Solver
        get() = activeSolver.getIfPresent(this) ?: error("Could not find solver for $this")

    fun Environment<*, *>.solver(settings: QpSettings): Solver =
        activeSolver.getIfPresent(this) ?: Solver(settings).also { activeSolver.put(this, it) }
}
