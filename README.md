# CAROL: Coordinated Aggregate Robotics with Online control Lyapunov and barrier functions

**CAROL** is a fully distributed, multi-robot control framework written in Kotlin. It bridges the gap between **Aggregate Programming** (via the Collektive framework) and **Optimization-based Control** using Control Lyapunov Functions (CLFs) and Control Barrier Functions (CBFs).

By leveraging the **Alternating Direction Method of Multipliers (ADMM)**, CAROL allows a swarm of robots to negotiate safe and optimal trajectories in a fully distributed manner, ensuring goal convergence, obstacle avoidance, inter-robot collision avoidance, and communication range maintenance.

## ✨ Key Features

* **Distributed Consensus (ADMM)**: Solves coupled multi-agent Quadratic Programs (QPs) using local and pairwise micro-iterations, eliminating the need for a central coordinator.
* **Safety & Connectivity (CBFs)**:
    * *Obstacle Avoidance*: Hard constraints to dodge static environment hazards.
    * *Collision Avoidance*: Hard constraints to maintain a minimum safe distance between neighbors.
    * *Communication Range*: Soft constraints (with L1 penalty) to maintain network connectivity without causing solver freezing when physical boundaries force a split.
* **Goal Tracking (CLFs)**: Proportional nominal controllers paired with CLFs to ensure asymptotic convergence to target destinations.
* **Zero-Order Hold (ZOH) Discretization**: Exact discrete-time robustification of continuous-time dynamics, ensuring the QP solver remains strictly affine and mathematically sound.
* **Stateless Architecture**: Completely thread-safe and immutable constraint definitions, ready for parallelized processing and coroutines.

## 🏗️ Project Structure

The codebase is engineered following strict Separation of Concerns (SoC) and SOLID principles:

```
src/main/
│ ├── yaml/
│ │   └── carolADMM.yml                       Alchemist simulation configuration
│ │
│ └── kotlin/it/unibo/
│     ├── alchemist/                          Native Alchemist classes (actions, reactions, UI effects)
│     └── collektive/                         Framework core (Logic, Control, ADMM)
│         ├── Entrypoint.kt                   Main aggregate program
│         ├── admm/                           ADMM Consensus algorithm in AC (QP costs, core loop, state)
│         ├── alchemist/device/               Integration Layer: Bridge between Collektive and Alchemist (sensors, environment ops)
│         ├── control/                        Control Theory interfaces and nominal controllers
│         │   ├── cbf/                        Control Barrier Functions (Obstacles, Collisions, Comm range)
│         │   └── clf/                        Control Lyapunov Functions (Target tracking)
│         ├── mathutils/                      Math utilities and Linear Algebra operations
│         ├── model/                          Domain Models (Robot, Target, Obstacle, Coordinates)
│         └── solver/gurobi/                  Gurobi solver wrapper, DSL extensions, and tuning settings
```

## 🚀 Prerequisites

To run the simulation, you will need:
1. **Java Development Kit (JDK)**: Version 11 or higher (17+ recommended).
2. **Gurobi Optimizer**: A working installation of Gurobi.
3. **Gurobi License**: You must possess a valid `gurobi.lic` file.
    * The system will automatically search for it in `~/Library/gurobi/gurobi.lic` (macOS default).
    * Alternatively, you can set the `GRB_LICENSE_FILE` environment variable or JVM system property pointing to the absolute path of your license file.

## 🛠️ Configuration and Usage

The simulation scenarios are defined using YAML configuration files processed by the Alchemist simulator.

### Tuning the Simulation (`carolADMM.yml`)
You can tweak the environment and solver settings directly from the YAML file:
```yaml
variables:
  maxCommDistance: 9.0
  robotSafeMargin: 0.5
  robotMaxSpeed: 2.0
  primalTolerance: 1e-2
  dualTolerance: 1e-2
  maxIterations: 50
  timeDistribution: 100.0 # 100 Hz control loop
```

### Running the Simulation

1. Ensure your Gurobi license is properly set up.
2. Build the project using Gradle: ```./gradlew build```
3. Run the Alchemist simulation with the specified YAML configuration: ```./gradlew runCarolADMM```

## 🧪 Mathematical Details
This framework implements the mathematical formulations described in the reference documentation.
S oft constraints (like maintaining communication) are handled using an L1 penalty mechanism in the objective function. 
This allows the QP solver to gracefully degrade connectivity to prioritize physical survival (obstacle/collision avoidance) 
without causing an explosion in the objective cost, maintaining ADMM convergence stability.

