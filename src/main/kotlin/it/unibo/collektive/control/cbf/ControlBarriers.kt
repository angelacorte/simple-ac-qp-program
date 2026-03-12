package it.unibo.collektive.control.cbf

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBException
import com.gurobi.gurobi.GRBLinExpr
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.control.objective.CBF
import it.unibo.collektive.control.objective.CBFContext
import it.unibo.collektive.model.Obstacle
import it.unibo.collektive.model.Robot
import it.unibo.collektive.model.minus
import it.unibo.collektive.model.squaredNorm
import it.unibo.collektive.model.toDoubleArray
import it.unibo.collektive.model.zeroVec
import it.unibo.collektive.solver.gurobi.ConstraintNames
import it.unibo.collektive.solver.gurobi.GRBVector
import it.unibo.collektive.solver.gurobi.addCBF
import it.unibo.collektive.solver.gurobi.toQuadExpr
import kotlin.math.max
import kotlin.math.pow


/**
 * Enforces a max-speed constraint `||u||^2 <= maxSpeed^2` as a quadratic constraint.
 */
fun GRBModel.maxSpeedCBF(u: GRBVector, robot: Robot) {
    addQConstr(
        u.toQuadExpr(),
        GRB.LESS_EQUAL,
        robot.maxSpeed.pow(2),
        "u_norm",
    )
}

/**
 * Default obstacle-avoidance barrier registered in [it.unibo.collektive.control.objective.CBFRegistry];
 * adds keep-out CBF against [CBFContext.obstacle].
 */
object OLDObstacleCBF : CBF {
    override val name: String = "obstacle"
    override fun add(model: GRBModel, uSelf: GRBVector, uOther: GRBVector?, ctx: CBFContext) {
        val obstacle = ctx.obstacle ?: return
        model.addObstacleAvoidanceCBF(
            currentPosition = doubleArrayOf(ctx.self.x, ctx.self.y),
            obstacle = obstacle,
            u = uSelf,
            gamma = ctx.settings.gammaObstacle,
            deltaTime = ctx.settings.deltaTime,
            name = ConstraintNames.obstacle("local"),
        )
    }


    /**
     * Discrete-time CBF (DCBF) for obstacle avoidance under ZOH dynamics.
     *
     * Enforces:  2sᵀu ≥ −(η/∆t) h_obs
     *
     * where s = p − p_o, h_obs = ‖s‖² − (r_o + d_o)², η = [gamma], ∆t = [deltaTime].
     */
    fun GRBModel.addObstacleAvoidanceCBF(
        currentPosition: DoubleArray,
        obstacle: Obstacle,
        u: GRBVector,
        gamma: Double = 0.5,
        deltaTime: Double = 1.0,
        name: String = ConstraintNames.obstacle("global"),
    ) {
        // 2(p - p-g)^T u = 2(p_x - p_o,x) u_x + 2(p_y - p_o,y) u_y
        val obstaclePosition: DoubleArray = obstacle.toDoubleArray()
        val distance = currentPosition - obstaclePosition // ||p - p_o||^2
        val safeDistance = obstacle.radius + obstacle.margin
        // - \gamma [ ||p - p_o||^2 - (r_o + d_o)^2 ] ==== - \gamma ((p_x - p_o,x)^2 + (p_y - p_o,y) ^2 - (r_o + d_o)^2)
        val h = distance.squaredNorm() - safeDistance.pow(2)
        addCBF(
            p1 = currentPosition,
            p2 = obstaclePosition,
            u1 = u,
            u2 = zeroVec(u.dimensions),
            gamma = gamma / deltaTime,
            h = h, // ( (p_x - p_o,x)^2 + (p_y - p_o,y) ^2 - (r_o^2 + d_o^2)
            name = name,
            coefU1 = 2.0,
            coefU2 = 0.0,
        )
    }
}

/**
 * Default robot–robot collision avoidance barrier registered in [it.unibo.collektive.control.objective.CBFRegistry];
 * enforces separation from [CBFContext.other].
 */
object OLDCollisionAvoidanceCBF : CBF {
    override val name: String = "collision"
    override fun add(model: GRBModel, uSelf: GRBVector, uOther: GRBVector?, ctx: CBFContext) {
        val other = ctx.other
        if (other == null || uOther == null) return
        model.addCollisionAvoidanceCBF(
            uSelf,
            uOther,
            ctx.self,
            other,
            ctx.settings.gammaCollision,
            ctx.settings.deltaTime,
            ConstraintNames.collision("${ctx.self.position}_${other.position}"),
        )
    }



    /**
     * Discrete-time CBF (DCBF) for inter-robot collision avoidance under ZOH dynamics.
     *
     * Enforces:  2rᵀ(uᵢ − uⱼ) ≥ −(η/∆t) h_col
     *
     * where r = pᵢ − pⱼ, h_col = ‖r‖² − d²_min, η = [gamma], ∆t = [deltaTime].
     */
    fun GRBModel.addCollisionAvoidanceCBF(
        ui: GRBVector,
        uj: GRBVector,
        robot: Robot,
        other: Robot,
        gamma: Double = 0.5,
        deltaTime: Double = 1.0,
        name: String = ConstraintNames.collision("global"),
    ) {
        // COLLISION AVOIDANCE 2(p1 - p2)^T (u1 - u2) + \gamma [ ||p1-p2||^2 - dmin^2 ] >= 0
        // 2(p1 - p2)^T (u1 - u2) >= - \gamma [ ||p1-p2||^2 - dmin^2 ]
        val distance = (robot.position - other.position).toDoubleArray()
        val maxDist = max(robot.safeMargin, other.safeMargin)
        val collision = GRBLinExpr()
        val collRight = -(gamma / deltaTime) * (distance.squaredNorm() - maxDist.pow(2))
        for (index in 0 until distance.size) {
            collision.addTerm(2.0 * distance[index], ui[index])
            collision.addTerm(-2.0 * distance[index], uj[index])
        }
        try {
            addConstr(collision, GRB.GREATER_EQUAL, collRight, name)
        } catch (e: GRBException) {
            println("Error for collision avoidance CBF: ${e.message}")
        }
    }

}

/**
 * Default communication-range barrier registered in [it.unibo.collektive.control.objective.CBFRegistry];
 * enforces max distance [CBFContext.communicationRange].
 */
object OLDCommunicationRangeCBF : CBF {
    override val name: String = "comm_range"
    override fun add(model: GRBModel, uSelf: GRBVector, uOther: GRBVector?, ctx: CBFContext) {
        val other = ctx.other
        val range = ctx.communicationRange
        if (other == null || uOther == null || range == null) return
        model.addCommunicationRangeCBF(
            uSelf,
            uOther,
            ctx.self,
            other,
            range,
            ctx.settings.gammaComm,
            ctx.settings.deltaTime,
            ConstraintNames.comm("${ctx.self.position}_${other.position}"),
            ctx.commSlack,
        )
    }


    /**
     * Robust discrete-time CBF (DCBF) for communication-range maintenance under ZOH dynamics.
     *
     * Enforces:  −2rᵀ(uᵢ − uⱼ) ≥ −(η/∆t) h_com + 4∆t u²_max
     *
     * where r = pᵢ − pⱼ, h_com = R² − ‖r‖², η = [gamma], ∆t = [deltaTime].
     */
    fun GRBModel.addCommunicationRangeCBF(
        ui: GRBVector,
        uj: GRBVector,
        robot: Robot,
        other: Robot,
        range: Double,
        gamma: Double = 2.0,
        deltaTime: Double = 1.0,
        name: String = ConstraintNames.comm("global"),
        slack: GRBVar? = null,
    ) {
        // COMM DISTANCE -2(p1 - p2)^T (u1 -u2) + \gamma [ R^2 - ||p1 - p2||^2 ] >= 0
        // COMM DISTANCE -2(p1 - p2)^T (u1 -u2) >= - \gamma [ R^2 - ||p1 - p2||^2 ]
        val distance = (robot.position - other.position).toDoubleArray()
        val communication = GRBLinExpr()
        val uMax = max(robot.maxSpeed, other.maxSpeed)
        val commRight = -(gamma / deltaTime) * (range.pow(2) - distance.squaredNorm()) + 4.0 * deltaTime * uMax.pow(2)
        for (index in 0 until distance.size) {
            communication.addTerm(-2.0 * distance[index], ui[index])
            communication.addTerm(2.0 * distance[index], uj[index])
        }
        slack?.let { communication.addTerm(1.0, it) }
        try {
            addConstr(communication, GRB.GREATER_EQUAL, commRight, name)
        } catch (e: GRBException) {
            println("Error for communication range CBF: ${e.message}")
        }
    }
}
