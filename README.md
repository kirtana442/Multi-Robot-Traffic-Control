# Multi-Robot Lane-Aware Traffic Control System

A robust, decentralized simulation environment designed for coordinated multi-robot navigation. This system implements lane-based rules, dynamic congestion awareness, and conflict-resolution strategies to maintain safety and throughput in high-density warehouse or industrial environments.

## Project Overview

This project addresses the challenge of navigating multiple robots through a shared graph-based environment. By utilizing dynamic reservation tables and lane-specific metadata, the system prevents collisions and resolves deadlocks in real-time.

**Key Capabilities:**
* **Dynamic Lane Metadata:** Adjusts robot behavior based on lane type (Narrow, Human Zone, etc.).
* **Conflict Resolution:** Uses a time-windowed reservation system to prevent resource contention.
* **Deadlock Recovery:** Detects cyclic waiting using "wait-for" graphs and resolves them via priority-based replanning.
* **Adaptive Intelligence:** Real-time speed scaling and path cost adjustments based on congestion hotspots.

---

## System Architecture

The project follows a modular architecture to separate data modeling, pathfinding logic, and simulation execution:

```text
Multi-Robot-Traffic-Control/
├── test_results       → 6 test_inputs each with outputs( trajectories.json, lane_heatmap, metrics.json)
├── models.py          → Data structures (Robot, Lane, SimulationMetrics)
├── planner.py         → A* path planning with lane-aware cost weight
├── coordinator.py     → Central logic for speed, reservations, and deadlock detection
├── simulator.py       → Main simulation engine and state management
├── visualization.py   → Matplotlib-based lane heatmap generation
├── main.py            → Entry point for a sample 8-robot simulation
└── test_suite.py      → Automated testing for stress and edge cases
```

---

## Core Features

### 1. Lane Model & Properties
The environment is represented as a directed graph $G = (V, E)$ where:
* **Nodes ($V$):** Distinct waypoints or intersections.
* **Edges ($E$):** Lanes with associated metadata including `max_speed`, `safety_level` (1-5), and `lane_type`.

### 2. Adaptive Speed Control
Robot velocity is not static. It is calculated dynamically to ensure safety in high-risk zones:
$$Speed = MaxSpeed \times CongestionFactor \times SafetyFactor \times TypeFactor$$
* **Narrow Lanes:** Speed reduced by 50%.
* **Human Zones:** Speed reduced by 70% to ensure safety.

### 3. Lane Reservation System
To prevent collisions, robots must "reserve" a lane for a specific time window $[T_{start}, T_{end}]$ before entry. 
* If a lane is occupied or reserved by a higher-priority robot, the system forces the requester to **wait** or **replan**.
* **Reservation Table:** `LaneID -> [(T_start, T_end, RobotID)]`

### 4. Deadlock Handling
The system proactively monitors for cyclic dependencies. When a deadlock is detected:
1.  A "wait-for" graph is constructed.
2.  `networkx.find_cycle` identifies the blockage.
3.  The lowest-priority robot in the cycle is forced to clear its reservations and recompute its trajectory.

---

## Performance Metrics & Outputs

The simulator produces three primary outputs for every run:

1.  **`trajectories.json`**: A complete timeline of every robot's position and arrival times.
2.  **`lane_heatmap.png`**: A visual representation of traffic pressure, where redder lanes indicate higher congestion/usage.
3.  **`metrics.json`**:
    * **Success Rate:** Percentage of robots that reached their goals.
    * **Throughput:** Robots completed per unit of time.
    * **Makespan:** Total time taken for all robots to finish.
    * **Safety Data:** Number of collisions prevented and deadlocks resolved.

---

## Test Scenarios

The system has been validated against six distinct scenarios in `test_suite.py`:

| Scenario | Objective | Observed Result |
| :--- | :--- | :--- |
| **Basic** | Verify pathfinding on an empty map. | 100% Success |
| **Deadlock Crossing** | Force robots into opposing head-on traffic. | Resolved via Replanning |
| **High Congestion** | Test cost-weight scaling with 8+ robots. | 100% Success (88 replans) |
| **High Density** | Stress test with 12 robots on a 20-node map. | High throughput maintained |
| **Safety Critical** | Validate speed reduction in Human Zones. | Zero safety violations |

---

## Installation & Usage

### Prerequisites
* Python 3.8+
* Dependencies: `networkx`, `matplotlib`, `numpy`

### Setup
```bash
# Clone the repository
git clone https://github.com/kirtana442/Multi-Robot-Traffic-Control.git
cd Multi-Robot-Traffic-Control

# Install dependencies
pip install networkx matplotlib
```

### Execution
To run the default simulation:
```bash
python main.py
```

To run the full comprehensive test suite:
```bash
python test_suite.py
```
*Results will be saved in the `./test_results/` directory.*

---

## Algorithm Logic: Lane-Aware Cost
Pathfinding uses a modified A* algorithm where the edge weight is calculated as:
$$Cost = BaseLength + CongestionPenalty + SafetyPenalty + TypePenalty$$
This ensures that robots naturally prefer longer, empty paths over short, congested, or unsafe corridors.
