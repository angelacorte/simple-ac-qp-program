package it.unibo.collektive.qp.dsl

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBLinExpr
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBQuadExpr
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.qp.utils.Robot
import it.unibo.collektive.qp.utils.Target
import it.unibo.collektive.qp.utils.minus
import it.unibo.collektive.qp.utils.toDoubleArray

/**
 * Represents a vector of Gurobi decision variables.
 *
 * This class abstracts a collection of scalar optimization variables as a single
 * vector-valued decision variable, enabling dimension-independent formulations
 * of control laws and constraints.
 *
 * Typical use cases include control inputs such as velocities or accelerations.
 */
class GRBVector(val vars: Array<GRBVar>) {
    val dimensions: Int get() = vars.size
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
fun GRBModel.addVecVar(
    dimension: Int,
    lowerBound: Double,
    upperBound: Double,
    name: String
): GRBVector = GRBVector(Array(dimension) { i -> addVar(lowerBound, upperBound, 0.0, GRB.CONTINUOUS, "$name[$i]") })

/**
 * Performs component-wise subtraction between two constant vectors.
 *
 * This operator is used to compute relative positions or differences between vectors.
 *
 * @receiver the minuend constant vector
 * @param other the subtrahend constant vector
 * @return a new constant vector representing the component-wise difference
 */
operator fun DoubleArray.minus(other: DoubleArray): DoubleArray = DoubleArray(size) { i -> this[i] - other[i] }

operator fun DoubleArray.times(other: DoubleArray): Double {
    require(this.size == other.size) { "Dimension mismatch: expected ${this.size}, got ${other.size}" }
    return DoubleArray(size) { i -> this[i] * other[i] }.sum()
}

operator fun Double.times(other: DoubleArray): DoubleArray = DoubleArray(other.size) { i -> this * other[i] }

/**
 * Computes the squared Euclidean norm of a constant vector.
 *
 * This function is typically used in Control Lyapunov Function (CLF) or Control Barrier Function (CBF) scalar terms.
 *
 * @receiver the constant vector
 * @return the squared norm value
 */
fun DoubleArray.squaredNorm(): Double = sumOf { it * it } // v^Tv

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
    for (i in 0 until dimensions) expr.addTerm(coefficient, this[i], this[i])
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
    name: String = "CLF"
) {
    val distanceVec = currentPos - goalPos
    val left = u.toLinExpr(distanceVec, 2.0) //GRBLinExpr()
    left.addTerm(-1.0, delta) // slack
    val right = - convergenceRate * distanceVec.squaredNorm()
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
    inequality: Char = GRB.GREATER_EQUAL
) {
    val dp = p1 - p2
    val left = u1.toLinExpr(dp, coefU1) //GRBLinExpr() // left side 2 (p1 - p2)ᵀ u1
    val right = coefU2 * (dp * u2) - gamma * h // right side
    addConstr(left, inequality, right, name)
}

/**
 * Sets a quadratic objective minimizing the deviation from a nominal control.
 *
 * The objective minimizes:
 *
 *   ||u - u_nominal||² + φ δ²
 *
 * using auxiliary variables to express the deviation in a quadratic form
 * compatible with Gurobi.
 *
 * @param u control decision variable
 * @param uNominal nominal (reference) control
 * @param delta slack variable
 * @param phi weight for the slack penalty
 */
fun GRBModel.minimizeDeviation(
    u: GRBVector,
    uNominal: DoubleArray,
    delta: GRBVar,
    phi: Double
) {
    val deltaU = Array(u.dimensions) { i -> addVar(-GRB.INFINITY, GRB.INFINITY, 0.0, GRB.CONTINUOUS, "du[$i]") }
    for (i in deltaU.indices) {
        val lin = GRBLinExpr()
        lin.addTerm(1.0, u[i])
        lin.addTerm(-1.0, deltaU[i])
        addConstr(lin, GRB.EQUAL, uNominal[i], "u_delta_$i")
    }
    val obj = GRBQuadExpr()
    deltaU.forEach { obj.addTerm(1.0, it, it) } // ||u - u^nom||^2
    obj.addTerm(phi, delta, delta) // \phi \delta^2
    setObjective(obj, GRB.MINIMIZE)
}

/**
 * Creates a constant zero vector of the given dimension.
 *
 * This utility is typically used to represent static entities
 * (e.g., obstacles with zero velocity).
 */
fun zeroVec(dim: Int): DoubleArray = DoubleArray(dim) { 0.0 }

/**
 * objective: min ||u - u_nom||^2 + phi * delta^2
 * objective is quadratic in (ux, uy), linear in delta (slack)
 * ||u - u^nom||^2 + \phi \delta^2
 * ||u - u^nom||^2 = (ux - ux^nom)^2 + (uy - uy^nom)^2
 */
fun <ID: Comparable<ID>> GRBModel.minimizeNominal(
    target: Target,
    robot: Robot<ID>,
    u: GRBVector,
    delta: GRBVar,
) = minimizeDeviation(u = u, uNominal = (target.position - robot.position).toDoubleArray(), delta = delta, phi = 2.0)
