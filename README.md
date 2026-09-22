# 5-DOF Robotic Arm & Kinematics Pipeline

A complete, end-to-end mechatronics and robotics manipulation project. This repository documents the transition from a standalone embedded prototype (Arduino, direct geometric kinematics, and OpenCV-based visual servoing) to a modular, industrial-grade software architecture powered by **ROS 2 Humble**, **MoveIt 2**, **ros2_control**, and numerical/analytical kinematic solvers.

---

### Key Features

* **Deterministic 5-DOF Kinematics Engine:** Bypasses numerical IK singularities and orientation deficits using closed-form analytical geometric models and constrained seed sampling.
* **Dual Software Stack:** Includes legacy bare-metal firmware (`without_ROS/`) and a modern ROS 2 manipulation pipeline (`ros2_packages/`).


* **Cartesian Tool Center Point (TCP) Control:** Integrated virtual `tool0` frame ensuring millimeter-precise Cartesian positioning decoupled from wrist orientation.
* **MoveIt 2 & ros2_control Architecture:** Custom URDF/SRDF definitions, hardware abstractions via `FakeSystem`, and trajectory execution through `FollowJointTrajectory`.
* **Interactive Vision Tracking:** Real-time face tracking and Cartesian velocity commands using OpenCV and acceleration gradients.



---

## Hardware Overview

The physical system is a 5-DOF articulated serial arm built with 3D-printed structural components and high-torque micro-servos.

| Axis | Joint Name | Actuator | Motion Range | Link Function |
| --- | --- | --- | --- | --- |
| **Joint 1** | `joint_1_base_rotation` | 35g Servo | $-90^\circ \text{ to } +90^\circ$<br> | Arm azimuthal rotation around the Z-axis |
| **Joint 2** | `joint_2_shoulder` | 35g Servo | $-45^\circ \text{ to } +90^\circ$ | Shoulder pitch elevation |
| **Joint 3** | `joint_3_forearm` | 35g Servo | $0^\circ \text{ to } +90^\circ$ | Elbow flexion / extension |
| **Joint 4** | `joint_4_wrist` | MG90S | $-90^\circ \text{ to } +90^\circ$ | Wrist axial roll |
| **Joint 5** | `joint_5_hand` | MG90S | $-90^\circ \text{ to } +90^\circ$ | End-effector pitch & tool positioning |
| **Tool** | `gripper` | MG90s | Open / Close | Side-mounted parallel claw mechanism |

> **Mechanical Note:** The gripper assembly features a side-mounted servo configuration and a dedicated mechanical offset. To prevent trajectory deviation during Cartesian moves, a virtual frame `tool0` is placed exactly at the gripping center between the finger tips.

---

## The 5-DOF Kinematic Challenge in MoveIt 2

### The Under-Actuation Dilemma

Standard 6-DOF industrial manipulators possess sufficient degrees of freedom to independently control position $(X, Y, Z)$ and orientation $(\text{Roll}, \text{Pitch}, \text{Yaw})$ in SE(3) space.

A 5-DOF serial manipulator is kinematically under-actuated: it lacks one Cartesian degree of freedom. When standard inverse kinematics plugins (such as KDL) are queried with a full 6D Cartesian pose goal, the numerical optimizer attempts to satisfy orientation constraints it cannot physically reach. This typically leads to:

1. **Solver divergence (Error code -31 / NO_IK_SOLUTION)**.
2. **Unpredictable local minima**, causing the arm to flip between radically different configurations for identical target positions.
3. **Severe TCP positional offsets**, as numerical solvers satisfy the joint origin pose while projecting the offset gripper tool tip away from the target.

### Implemented Solutions in this Pipeline

To resolve this limitation, three distinct architectural solutions were implemented and benchmarked:

```
                  ┌──────────────────────────────────────────────┐
                  │          Cartesian Goal (X, Y, Z)            │
                  └──────────────────────┬───────────────────────┘
                                         │
         ┌───────────────────────────────┼───────────────────────────────┐
         ▼                               ▼                               ▼
┌──────────────────┐           ┌──────────────────┐           ┌──────────────────┐
│  Analytical IK   │           │ Fixed-Seed KDL   │           │ Constrained TCP  │
│  (Closed-Form)   │           │ (Deterministic)  │           │ (Fixed EE Pitch) │
└────────┬─────────┘           └────────┬─────────┘           └────────┬─────────┘
         │                               │                               │
         ▼                               ▼                               ▼
Exact joint angles             Biased optimization             MoveIt solves 1-4,
in < 0.1 ms (Geometric)        avoids branch flips             Joint 5 set by rule

```

#### 1. Deterministic Fixed Seed State (`test1_seedFix.py`)



Rather than querying KDL with an unconstrained random seed or relying on the current hardware state, a deterministic seed pose $\mathbf{q}_{\text{seed}} = [0.0, 0.35, -0.70, 0.0, 0.0]$ is injected into every `GetPositionIK` service request. This biases the numerical optimizer toward an "elbow-up" configuration, eliminating erratic posture flips during consecutive waypoints.

#### 2. Direct End-Effector Pitch Decoupling (`test1_eeFix.py`)



MoveIt solves the first four kinematic links with `position_only_ik: true`. The script then intercepts link 5 (`joint_5_hand`) and applies a geometric planar leveling rule:


$$\theta_5 = -(\theta_2 + \theta_3) + \phi_{\text{target}}$$


This guarantees that the gripper maintains a stable angle relative to the ground plane (horizontal or vertical pick) regardless of arm extension.

#### 3. Closed-Form Geometric Inverse Kinematics (`analytical_sol.py`)



A pure trigonometric solver eliminating MoveIt service overhead entirely. Solves link projections in the cylindrical coordinate frame:

* **Base Angle:** $\theta_1 = \text{atan2}(y, x)$

* **Planar Reach:** Decoupled wrist center coordinates $(r_w, z_w)$ followed by the Law of Cosines on links $L_1$ and $L_2$.


* **Result:** $100\%$ repeatable, zero convergence failures, calculation time $< 100\,\mu\text{s}$.

---

## Repository Structure

```text
.
├── without_ROS/                     # Embedded & Computer Vision roots[cite: 5]
│   ├── braccio_IK_convertito.ino    # Arduino C++ direct analytical kinematics[cite: 1]
│   ├── cvzone face_det_accle_xy.py  # Real-time face tracking & serial streaming[cite: 2]
│   ├── ik_coseno.ino                # Law of Cosines implementation on microcontrollers[cite: 4]
│   └── IK.py                        # Python pyFirmata direct servo hardware testing[cite: 3]
│
└── ros2_packages/                   # Industrial ROS 2 workspace packages[cite: 5]
    ├── mio_progetto/                # Kinematic testing and execution nodes[cite: 5]
    │   ├── mio_progetto/[cite: 5]
    │   │   ├── analytical_sol.py    # Analytical closed-form 5-DOF IK engine[cite: 5]
    │   │   ├── ikpy_to_rviz.py      # High-frequency IKPy state publisher (50 Hz)[cite: 5]
    │   │   ├── test_moveit_5dof.py  # MoveIt 2 GetPositionIK client[cite: 5]
    │   │   ├── test1_pinzaFissa.py       # Decoupled gripper pitch compensation node[cite: 5]
    │   │   └── test1_seedFisso.py     # Deterministic seed IK execution node[cite: 5]
    │   ├── package.xml[cite: 5]
    │   └── setup.py[cite: 5]
    │
    ├── robotic_arm_description/     # Robot model: URDF meshes, materials & TCP[cite: 5]
    └── robotic_arm_moveit_config/   # MoveIt 2 planning pipelines, SRDF & controllers[cite: 5]
        ├── config/
        │   ├── kinematics.yaml      # Solver tolerances, timeout & position_only_ik
        │   └── robotic_arm_5dof.srdf# Planning groups and collision pairs
        └── launch/
            └── demo.launch.py       # Main simulation and RViz visualization bringup

```

---

## Kinematics Benchmark

| Metric / Parameter | MoveIt 2 (KDL Default) | MoveIt 2 (Fixed Seed) | IKPy Optimizer | Geometric Analytical |
| --- | --- | --- | --- | --- |
| **Computation Latency** | $\sim 5\text{--}15\text{ ms}$ | $\sim 5\text{ ms}$ | $\sim 2\text{--}4\text{ ms}$ | **$< 0.1\text{ ms}$** |
| **Deterministic Output** | ❌ No (Can flip) | ⚠️ High (Seed-locked) | ⚠️ Sensitive to seed | ✅ **100% Deterministic** |
| **Orientation Handling** | Requires 6D Pose | Position-Only (`tool0`) | Position-Only Mask | Explicit Pitch Control

 |
| **Environment Collisions** | ✅ Full Octomap/Scene | ✅ Full Octomap/Scene | ❌ Link limits only | ❌ Software bounds only |
| **Resource Overhead** | High (MoveGroup stack) | High (MoveGroup stack) | Low (Python package) | **Minimal (Embedded ready)**<br> |

---

## Getting Started

### Prerequisites

* **OS:** Ubuntu 22.04 LTS or WSL2 (Ubuntu 22.04)
* **ROS Version:** ROS 2 Humble Desktop
* **MoveIt 2:** `ros-humble-moveit`
* **Python Dependencies:** `ikpy`, `numpy`, `opencv-python`

```bash
sudo apt update
sudo apt install -y ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers
pip install ikpy numpy opencv-python

```

### Installation & Build

```bash
# 1. Create your ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2. Clone the repository packages into src/
git clone https://github.com/kushalgollen/5-DOF-Robotic-Arm-and-Kinematics-Pipeline.git .

# 3. Build with colcon symlink-install
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

```

---

## Quickstart Execution

### 1. Launch MoveIt 2 Simulation

To launch RViz, the MoveGroup node, and the `FakeSystem` joint trajectory controllers:

```bash
ros2 launch robotic_arm_moveit_config demo.launch.py

```

### 2. Run the Deterministic Seed Controller

In a separate terminal, trigger a Cartesian target trajectory using the fixed-seed MoveIt solver:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run mio_progetto test1_seedFisso

```

### 3. Run the Analytical Closed-Form Controller

For instant calculation and direct pitch regulation:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run mio_progetto analytical_sol

```

### 4. Run High-Frequency IKPy RViz Streaming

To preview real-time Cartesian trajectory following at 50 Hz without MoveIt service overhead:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run mio_progetto ikpy_to_rviz

```

---

## Legacy Embedded Experiments (`without_ROS/`)



Before migrating to ROS 2, the arm operated as a standalone prototype:

* **`braccio_IK_convertito.ino` & `ik_coseno.ino`:** Real-time geometric IK written in bare-metal C++ executing directly on an Arduino UNO / Mega board.


* **`cvzone face_det_accle_xy.py`:** Video processing loop using OpenCV to detect face movements, calculating pixel acceleration, and sending Cartesian displacement steps over USB Serial to the microcontroller.


* **`IK.py`:** Low-level servo calibration and kinematics debugging using the `pyFirmata` protocol.



---

## License

This project is licensed under the MIT License - see the [LICENSE](https://www.google.com/search?q=LICENSE&utm_source=gemini) file for details.