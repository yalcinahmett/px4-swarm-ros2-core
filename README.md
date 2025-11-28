# Distributed Swarm Control with ROS 2 & PX4

This project implements a fully distributed swarm control system for Unmanned Aerial Vehicles (UAVs) using **ROS 2 Humble** and **PX4 Autopilot**. The system utilizes **Spectral Graph Theory** for formation control and **Artificial Potential Fields (APF)** for collision avoidance, enabling a robust and scalable swarm architecture.

## Key Features

*   **Distributed Consensus:** Agents maintain formation geometry using graph Laplacian-based consensus protocols without a central planner.
*   **Collision Avoidance:** Real-time repulsion forces prevent inter-agent collisions using artificial potential fields.
*   **Hybrid Control Architecture:** A weighted control loop balances formation maintenance, position tracking, and collision avoidance based on dynamic costs.
*   **Scalable Architecture:** Designed with dynamic ROS 2 subscribers and namespace handling to support $N$ drones.
*   **Math-Logic Separation:** Core mathematical algorithms are encapsulated in a standalone, Eigen-based C++ library (`GraphMath`), separating control logic from the ROS node.

## Tech Stack

*   **Middleware:** ROS 2 Humble
*   **Flight Stack:** PX4 Autopilot (v1.14+)
*   **Simulation:** Gazebo Harmonic
*   **Language:** C++
*   **Math Library:** Eigen3
*   **Communication:** Micro-XRCE-DDS

## System Architecture

The control loop operates in the `OFFBOARD` mode of PX4 via DDS.

### 1. State Machine
The node operates on a finite state machine:
- **IDLE:** Waits for initialization.
- **TAKEOFF:** Performs synchronized takeoff to a safe altitude.
- **MISSION:** Activates the distributed consensus algorithm.
- **LAND:** Performs safe landing procedures.

### 2. Control Algorithm
The velocity vector $\mathbf{v}_i$ for agent $i$ is computed as:

$$ \mathbf{v}_i = \alpha \mathbf{v}_{form} + \beta \mathbf{v}_{pos} + \gamma \mathbf{v}_{col} $$

Where:
*   $\mathbf{v}_{form}$: Gradient descent on the Graph Laplacian cost (Formation rigidity).
*   $\mathbf{v}_{pos}$: Vector towards the desired centroid offset.
*   $\mathbf{v}_{col}$: Repulsion vector based on inverse-square law ($1/d^2$).

## Installation

### Prerequisites
*   Ubuntu 22.04 LTS
*   ROS 2 Humble
*   PX4 Autopilot Toolchain
*   Eigen3 Library

### Build
Clone the repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/yalcinahmett/px4-swarm-ros2-core.git
cd ~/ros2_ws
colcon build 
source install/setup.bash
```
### Start Simulation
Run the headless simulation script to start Gazebo, Micro-XRCE-DDS, and PX4 instances:
```bash
cd src/px4-swarm-ros2-core/scripts
./start_headless_swarm.sh
```
### Start Swarm Control
Launch the ROS 2 nodes for the swarm agents:
```bash
ros2 launch px4-swarm-ros2-core swarm.launch.py
```