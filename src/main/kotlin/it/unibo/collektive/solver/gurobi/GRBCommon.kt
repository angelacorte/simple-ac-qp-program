@file:Suppress("MatchingDeclarationName")

package it.unibo.collektive.solver.gurobi

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBEnv
import com.gurobi.gurobi.GRBLinExpr
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBQuadExpr
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.model.minus
import it.unibo.collektive.model.squaredNorm
import it.unibo.collektive.model.times
import it.unibo.collektive.model.zeroVec
import java.nio.file.Files
import java.nio.file.Path
import java.nio.file.Paths

/**
 * Represents a vector of Gurobi decision variables [vars].
 *
 * This class abstracts a collection of scalar optimization variables as a single
 * vector-valued decision variable, enabling dimension-independent formulations
 * of control laws and constraints.
 *
 * Typical use cases include control inputs such as velocities or accelerations.
 */
class GRBVector(val vars: Array<GRBVar>) {
    /** Underlying scalar decision variables. */
    val dimensions: Int get() = vars.size

    /** Returns the ith decision variable. */
    operator fun get(index: Int): GRBVar = vars[index]
}

/**
 * Adds a vector of continuous decision variables to the optimization model.
 *
 * All components share the same lower and upper bounds and are created as
 * independent scalar variables internally.
 *
 * @param dimension dimensionality of the vector
 * @param lowerBound lower bound for each component
 * @param upperBound upper bound for each component
 * @param name base name for the variables (indexed automatically)
 * @return a `Vector` representing the newly created decision variables
 */
fun GRBModel.addVecVar(dimension: Int, lowerBound: Double, upperBound: Double, name: String): GRBVector =
    GRBVector(Array(dimension) { i -> addVar(lowerBound, upperBound, 0.0, GRB.CONTINUOUS, "$name[$i]") })

/**
 * Adds a scaled squared-norm term to this quadratic expression.
 *
 * Expands `rho * ||u - a||^2 = rho * Σ_i (u_i^2 - 2 a_i u_i + a_i^2)` where [u]
 * is a vector of decision variables and [a] is a constant vector.
 *
 * @param u decision-variable vector
 * @param a constant vector with the same dimension as [u]
 * @param rho non-negative weight applied to the norm
 * @throws IllegalArgumentException when [u] and [a] have different sizes
 */
fun GRBQuadExpr.addRhoNorm2Sq(u: GRBVector, a: DoubleArray, rho: Double = 1.0) {
    require(u.vars.size == a.size) { "u and a must have same length" }
    for (i in u.vars.indices) { // rho * || u - a ||^2
        addTerm(rho, u[i], u[i]) // rho * x_i^2
        addTerm(-2.0 * rho * a[i], u[i]) // -2*rho*a_i * x_i
        addConstant(rho * a[i] * a[i]) // + rho * a_i^2 (constant)
    }
}

/**
 * Computes the dot product between a constant vector and a vector of decision variables.
 *
 * Given a constant vector `this ∈ ℝⁿ` and a vector of Gurobi decision variables
 * `u ∈ ℝⁿ`, this function constructs the linear expression:
 *
 *     thisᵀ · u = Σᵢ this[i] · u[i]
 *
 * This operator is primarily used to encode time derivatives of scalar functions
 * defined over the system state, under the assumption of first-order dynamics:
 *
 *     ṗ = u
 *
 * In particular, for a scalar function h(p), whose gradient is ∇h(p),
 * the time derivative along the system trajectories is:
 *
 *     ḣ(p) = ∇h(p)ᵀ · u
 *
 * Hence, `Vec.dot` provides the algebraic bridge between continuous-time
 * control-theoretic formulations (CLFs and CBFs) and their instantaneous
 * quadratic-program implementation.
 *
 * Note:
 * - The time variable does not explicitly appear in the optimization problem.
 * - The resulting expression represents an infinitesimal (instantaneous)
 *   time derivative evaluated at the current state.
 * - Discrete-time execution is assumed to be handled externally via
 *   sample-and-hold control.
 *
 * @receiver a constant vector representing coefficients or a gradient (Vec)
 * @param u a vector of optimization variables (Vector)
 * @return a linear Gurobi expression representing the dot product thisᵀ · u
 *
 * @throws IllegalArgumentException if the two vectors have different dimensions
 */
fun GRBVector.toLinExpr(vector: DoubleArray, multiplier: Double = 1.0): GRBLinExpr {
    require(vector.size == dimensions) { "Dimension mismatch |v|=${vector.size}, |u|=$dimensions" }
    val expr = GRBLinExpr()
    for (i in vector.indices) expr.addTerm(multiplier * vector[i], this[i])
    return expr
}

/**
 * Constructs a quadratic expression representing the squared norm of a vector of decision variables.
 *
 * This expression can be used in quadratic constraints or objectives.
 *
 * @param u the vector of decision variables
 * @return a GRBQuadExpr representing the squared norm ||u||^2
 */
fun GRBVector.toQuadExpr(coefficient: Double = 1.0): GRBQuadExpr {
    val expr = GRBQuadExpr()
    expr.addRhoNorm2Sq(this, zeroVec(this.dimensions), coefficient)
//    for (i in 0 until dimensions) expr.addTerm(coefficient, this[i], this[i])
    return expr
}

/**
 * Attempts to locate a Gurobi license without hardcoding its path.
 *
 * Preference order:
 * 1) `GRB_LICENSE_FILE` environment variable
 * 2) `GRB_LICENSE_FILE` JVM system property
 * 3) default macOS location `~/Library/gurobi/gurobi.lic`
 *
 * When none are found, it throws with a descriptive message so callers can configure the path externally.
 */
private fun resolveLicensePath(): Path? {
    val candidates = listOfNotNull(
        System.getenv("GRB_LICENSE_FILE")?.takeIf { it.isNotBlank() }?.let { Paths.get(it) },
        System.getProperty("GRB_LICENSE_FILE")?.takeIf { it.isNotBlank() }?.let { Paths.get(it) },
        Paths.get(System.getProperty("user.home"), "Library", "gurobi", "gurobi.lic"),
    )
    return candidates.firstOrNull { Files.exists(it) }
}

/**
 * Sets the Gurobi license path at runtime by preferring environment variables and system properties.
 *
 * This avoids hardcoding the license location and provides a clear error when no license is discoverable.
 */
fun setLicense() {
    val found = resolveLicensePath()
    if (found != null) {
        System.setProperty("GRB_LICENSE_FILE", found.toString())
        return
    }
    val defaultPath = Paths.get(System.getProperty("user.home"), "Library", "gurobi", "gurobi.lic")
    error(
        "Gurobi license file not found. Set the GRB_LICENSE_FILE environment variable or JVM property " +
            "to the license file path, or place the license in '$defaultPath'",
    )
}

/**
 * Helper to create a GRBModel with license setup, optional logging
 * (via [QpSettings.logEnabled]) and automatic disposal.
 */
inline fun <T> withModel(settings: QpSettings = QpSettings(), block: (GRBModel) -> T): T {
    setLicense()
    val env = GRBEnv(true).also {
        if (!settings.logEnabled) it.set(GRB.IntParam.OutputFlag, 0)
        it.start()
    }
    val model = GRBModel(env).also { if (settings.logEnabled) it.setupLogger() }
    return try {
        block(model)
    } finally {
        model.dispose()
        env.dispose()
    }
}

/** Constraint name generator using the prefix in [QpSettings.constraintPrefix]. */
object ConstraintNames {
    /** Collision-avoidance constraint name for a given [edgeId]. */
    fun collision(edgeId: String) = "${settingsPrefix()}_collision_$edgeId"

    /** Communication-range constraint name for a given [edgeId]. */
    fun comm(edgeId: String) = "${settingsPrefix()}_comm_$edgeId"

    /** Obstacle-avoidance constraint name for a given obstacle [id]. */
    fun obstacle(id: String) = "${settingsPrefix()}_obstacle_$id"

    /** Target-tracking CLF constraint name for a given [id]. */
    fun clf(id: String) = "${settingsPrefix()}_clf_$id"

    /** Slack constraint name for a given [id]. */
    fun slack(id: String) = "${settingsPrefix()}_slack_$id"

    private fun settingsPrefix(): String = QpSettings().constraintPrefix
}
