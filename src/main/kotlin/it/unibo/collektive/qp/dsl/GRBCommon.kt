@file:Suppress("MatchingDeclarationName")

package it.unibo.collektive.qp.dsl

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBLinExpr
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBQuadExpr
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.qp.utils.minus
import it.unibo.collektive.qp.utils.squaredNorm
import it.unibo.collektive.qp.utils.times
import it.unibo.collektive.qp.utils.zeroVec
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
 * Adds a Control Lyapunov Function (CLF) constraint enforcing convergence
 * toward a target point.
 *
 * The constraint encodes the continuous-time inequality:
 *
 *   2 (p - p_g)ᵀ u ≤ -c ||p - p_g||² + δ
 *
 * under the first-order system dynamics ṗ = u.
 *
 * The slack variable `delta` allows soft constraint enforcement to preserve
 * feasibility when combined with other constraints.
 *
 * @param currentPos current position p
 * @param goalPos target position p_g
 * @param u control input vector
 * @param convergenceRate positive convergence rate
 * @param delta slack variable
 * @param name constraint name
 */
fun GRBModel.toTargetCLF(
    currentPos: DoubleArray,
    goalPos: DoubleArray,
    u: GRBVector,
    convergenceRate: Double,
    delta: GRBVar,
    name: String = "CLF",
) {
    val distanceVec = currentPos - goalPos
    val left = u.toLinExpr(distanceVec, 2.0) // GRBLinExpr()
    left.addTerm(-1.0, delta) // slack
    val right = -convergenceRate * distanceVec.squaredNorm()
    addConstr(left, GRB.LESS_EQUAL, right, name)
}

/**
 * Adds a relative Control Barrier Function (CBF) constraint between two entities.
 *
 * CBF = h_dot(x) + alpha(h(x)) >= 0
 * alpha(s) = γs , γ > 0
 *
 * > h_dot >= -γh
 * This function encodes safety or connectivity constraints based on the relative
 * position p1 - p2 and assumes first-order dynamics ṗ = u.
 *
 * The general form enforced is:
 *
 *   2 (p1 - p2)ᵀ u1 ≥ -2 (p1 - p2)ᵀ u2 - γ h
 *   coefU1 (p1 - p2)ᵀ u1 ≥ - coefU2 (p1 - p2)ᵀ u2 + h
 *
 * where u2 is treated as a known (exogenous) quantity.
 *
 * @param p1 position of the controlled entity
 * @param p2 position of the other entity
 * @param u1 control input of the controlled entity
 * @param u2 known velocity of the other entity
 * @param h barrier function value
 * @param name constraint name
 */
fun GRBModel.addCBF(
    p1: DoubleArray,
    p2: DoubleArray,
    u1: GRBVector,
    u2: DoubleArray,
    gamma: Double = 0.5,
    h: Double,
    name: String,
    coefU1: Double = 2.0,
    coefU2: Double = 2.0,
    inequality: Char = GRB.GREATER_EQUAL,
) {
    val dp = p1 - p2
    val left = u1.toLinExpr(dp, coefU1) // GRBLinExpr() // left side 2 (p1 - p2)ᵀ u1
    val right = coefU2 * (dp * u2) - gamma * h // right side
    addConstr(left, inequality, right, name)
}

//
// /**
// * Sets a quadratic objective minimizing the deviation from a nominal control.
// *
// * The objective minimizes:
// *
// *   ||u - u_nominal||² + φ δ²
// *
// * using auxiliary variables to express the deviation in a quadratic form
// * compatible with Gurobi.
// *
// * @param u control decision variable
// * @param uNominal nominal (reference) control
// * @param delta slack variable
// * @param phi weight for the slack penalty
// */
// fun GRBModel.minimizeDeviation(
//    u: GRBVector,
//    uNominal: DoubleArray,
//    delta: GRBVar,
//    phi: Double
// ) {
//    val deltaU = Array(u.dimensions) { i -> addVar(-GRB.INFINITY, GRB.INFINITY, 0.0, GRB.CONTINUOUS, "du[$i]") }
//    for (i in deltaU.indices) {
//        val lin = GRBLinExpr()
//        lin.addTerm(1.0, u[i])
//        lin.addTerm(-1.0, deltaU[i])
//        addConstr(lin, GRB.EQUAL, uNominal[i], "u_delta_$i")
//    }
//    val obj = GRBQuadExpr()
//    deltaU.forEach { obj.addTerm(1.0, it, it) } // ||u - u^nom||^2
//    obj.addTerm(phi, delta) // \phi \delta^2
//    setObjective(obj, GRB.MINIMIZE)
// }

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
