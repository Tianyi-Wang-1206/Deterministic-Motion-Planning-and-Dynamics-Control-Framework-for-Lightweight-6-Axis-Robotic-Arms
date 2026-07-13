# Deterministic Motion Planning and Dynamics Control Framework for Lightweight 6-Axis Robotic Arms

![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-3498db.svg)
![C++17](https://img.shields.io/badge/C++-17-blue.svg)
![Python 3.12](https://img.shields.io/badge/Python-3.12-yellow.svg)
![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)

This repository presents a high-fidelity, mathematically rigorous 6-axis robotic arm simulation and control framework. Initiated by an asynchronous and detailed GUI (based on PyQt5), the system orchestrates high-level motion using a **custom-built, deterministic C++ planning engine**, bypassing standard frameworks like MoveIt2. This planner leverages a **closed-form Analytical Inverse Kinematics** solver (derived via Maple symbolic computation), combined with **Coal (formerly HPP-FCL)** for 3D collision avoidance and symbolic Jacobian determinant analysis for singularity detection. Trajectory generation is handled by **Ruckig**, providing time-optimal profiles for Joint Space (**MoveJ**), Cartesian Space (**MoveP**), Linear (**MoveL**), and Circular (**MoveC**) Cartesian motions.

At the core, **Controller Chaining** (based on **ros2_control**) interpolates these trajectories into a tight 1000Hz reference for a custom **Computed Torque Control (CTC)** downstream controller. To ensure physical realism, the CTC bypasses **MuJoCo**’s default PID joints to run the physics engine in pure **effort mode**. The CTC processes the feedback states via a **1D Kalman Filter** state observation model to reconstruct noise-free joint velocities, and executes **Pinocchio** dynamics calculations to compute rigid-body inertia, gravity, and Coriolis matrices. Combined with **friction/armature compensation** (smoothly neutralizing viscous damping, Coulomb stiction, and motor rotor inertia), this architecture achieves a tracking precision of $<0.01\text{ mm}$.

To bridge the gap between nominal simulation parameters and real-world physical discrepancies, an automated **system identification** (SysID) framework is integrated to extract joint-level viscous/Coulomb friction and motor armature coefficients, which are difficult to measure directly during manufacturing. Utilizing a bounded **Fourier excitation trajectory** coupled with an offline **Least-Squares solver**, the identified parameters align closely with the ground-truth physical properties defined in the URDF and MuJoCo XML files. Furthermore, the architecture incorporates a robust software emergency stop (E-Stop) and active recovery state machine. Upon triggering, it overrides high-level commands to execute precise, kinematic-limit-aware deceleration trajectories that safely halt the manipulator, while providing a seamless recovery sequence back to the active closed-loop tracking state.

To facilitate real-time diagnostics and visual verification of tracking performance, the framework incorporates a synchronized **"shadow robot"** virtual twin. This visualization layer is driven directly by the real-time C++ CTC controller via a lock-free `RealtimePublisher`. It renders the exact internal target commands (`q_target_`) at 1000Hz (downsampled to 200Hz for RViz), representing the manipulator's theoretical kinematic state under zero-disturbance conditions. The shadow robot depicts pure kinematic targets, while the closed-loop CTC controller forces the physical manipulator to overcome physical disturbances (gravity, Coriolis, joint friction, etc.) to track this reference.

For detailed mathematical derivations of the analytical IK and Jacobian matrices, please refer to [Kinematics.pdf](Kinematics.pdf).

For detailed mathematical derivations of the CTC controller and the SysID process, please refer to [Theory.pdf](Theory.pdf).

> ⚠️ **Notice:** The robot model used in this simulation is the **UFactory Lite6**, sourced from the official [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie). This repository is an independent open-source project designed for research and educational purposes.

<p align="center">
  <img src="media/demo_MoveJ.gif" width="1080" alt="Joint PTP Motion and GUI Overview"/>
</p>


## 📑 Table of Contents
1. [System Architecture](#-system-architecture)
2. [Core Features & Demonstrations](#-core-features--demonstrations)
3. [Quick Start & Reproduction Guide](#-quick-start--reproduction-guide)
4. [Future Work](#-future-work)
5. [Acknowledgements](#-acknowledgements)
6. [Disclaimer](#-disclaimer)
7. [Contact](#-contact)


## 🧠 System Architecture
The system's architecture is illustrated in the diagram below:

```mermaid
graph TB
    %% Style Definitions
    classDef PyQt fill:#34495e,stroke:#2c3e50,stroke-width:2px,color:#fff;
    classDef Planner fill:#c0392b,stroke:#922b21,stroke-width:2px,color:#fff;
    classDef R2C fill:#8e44ad,stroke:#5b2c6f,stroke-width:2px,color:#fff;
    classDef Custom fill:#d35400,stroke:#a04000,stroke-width:2px,color:#fff;
    classDef Physics fill:#27ae60,stroke:#1e8449,stroke-width:2px,color:#fff;
    classDef Util fill:#7f8c8d,stroke:#5d6d7e,stroke-width:2px,color:#fff;

    %% Subgraph: PyQt5 HMI Layer (Asynchronous / Non-Real-Time)
    subgraph Sub_GUI [PyQt5 HMI Layer — Async / ~30Hz]
        UI[PyQt5 GUI Window]:::PyQt
        Worker[ROS2Worker Thread]:::PyQt
        SysID[SysIdEngine]:::PyQt
        UI <-->|Signals/Slots<br/>Bidirectional Sync| Worker
        Worker <-->|Least Squares Fitting| SysID
    end

    %% Subgraph: Custom Industrial Planning Engine
    subgraph Sub_Planner [Industrial Planning Engine — C++ Async]
        Action[IndustrialMotion Action Server]:::Planner
        KinEngine[KinematicsEngine<br/>Analytical IK & Singularity Checker]:::Planner
        Coal[Coal HPP-FCL<br/>3D Collision Avoidance]:::Planner
        Ruckig[Ruckig OTG<br/>Trajectory Generation]:::Planner
        
        Action <-->|Validates Target & Interpolation| KinEngine
        Action <-->|Checks Mid-air Collision| Coal
        Action <-->|Generates Time-optimal Profile| Ruckig
    end

    %% Subgraph: Hard Real-Time ROS 2 Control Loop (1000Hz)
    subgraph Sub_R2C [ROS 2 Control Manager — Hard Real-Time 1000Hz Loop]
        JTC[JointTrajectoryController<br/>Reference Forwarder & Fine Interpolator]:::R2C
        CTC[Lite6CTCController<br/>Downstream CTC Controller]:::Custom
        JSB[JointStateBroadcaster]:::R2C
        KF[1D Kalman Filter<br/>State Observer]:::Custom
        Pinocchio[Pinocchio RBD Engine<br/>RNEA Solver]:::Custom
        
        %% Chaining Mechanism
        JTC ==>|Memory Pointer Sharing<br/>q_d, dq_d, ddq_d| CTC
        CTC <-->|computes M, C, G| Pinocchio
        KF -->|Filtered q, dq| CTC
    end

    %% Subgraph: MuJoCo Simulation Environment (1000Hz)
    subgraph Sub_MuJoCo [MuJoCo Simulation Environment — 1000Hz]
        HW[Lite6MujocoSystem<br/>Hardware Interface]:::Custom
        Engine[MuJoCo<br/>Effort Mode]:::Physics
        HW <-->|mj_step / C API| Engine
    end

    %% Subgraph: Shadow Robot Twin (Visualization Helper)
    subgraph Sub_Display [Robot Graphical Display — 200Hz]
        ShadowRSP[Shadow Robot State Publisher]:::Util
        RViz[RViz2 Visualization]:::Util
    end

    %% System Connections (Data Flow)
    Worker -->|Action Goal:<br/>IndustrialMotion.action| Action
    Action ==>|Action Goal:<br/>FollowJointTrajectory| JTC
    Worker -.->|Software E-Stop<br/>Injects zero-velocity pt| JTC
    
    %% CTC Inputs & Outputs
    HW -->|Hardware State Interface<br/>Raw q_measured| KF
    HW -->|Hardware State Interface| JSB
    CTC ==>|Hardware Command Interface<br/>Output Torques tau_cmd| HW
    
    %% GUI State Feedback & SysID
    JSB -.->|Topic /joint_states| Worker
    Worker -.->|SysID Mode Commands:<br/>Int32 /system_cmd| CTC
    
    %% Shadow Robot Flow (Direct from CTC)
    CTC -.->|"RT Publisher (1000Hz -> 200Hz)"<br/>Topic /shadow/joint_states| ShadowRSP
    ShadowRSP -.->|Shadow TFs| RViz
    JSB -.->|Main Robot TFs| RViz

    %% Frequencies & Details annotations
    style Sub_GUI fill:#f4f6f9,stroke:#bdc3c7,stroke-width:2px;
    style Sub_Planner fill:#ebf5fb,stroke:#a9cce3,stroke-width:2px;
    style Sub_R2C fill:#f5eef8,stroke:#d7bde2,stroke-width:2px;
    style Sub_MuJoCo fill:#eaf2f8,stroke:#a9dfbf,stroke-width:2px;
    style Sub_Display fill:#f2f4f4,stroke:#ccd1d1,stroke-width:2px;
```

## 🎥 Core Features & Demonstrations

### 1. Industrial Motion Planning (PTP, MoveL, MoveC)

*   **PTP (Point-to-Point):** Joint space planning (MoveJ) or Cartesian target planning (MoveP) utilizing the optimal inverse kinematic configuration.
<p align="center">
  <img src="media/demo_MoveJ.gif" width="1080" alt="Joint PTP Motion Planning"/>
</p>

<p align="center">
  <img src="media/demo_MoveP.gif" width="1080" alt="Cartesian PTP Motion Planning"/>
</p>

*   **MoveL:** Deterministic linear Cartesian interpolation using SLERP for quaternion orientation and Jacobian pseudoinverse mapping.

<p align="center">
  <img src="media/demo_MoveL.gif" width="1080" alt="Linear Interpolation and Planning"/>
</p>

*   **MoveC:** Circular Cartesian interpolation using an auxiliary midpoint frame.

<p align="center">
  <img src="media/demo_MoveC.gif" width="1080" alt="Circular Interpolation and Planning"/>
</p>

### 2. Automated System Identification
*   The automated system identification executes bounded Fourier excitation trajectories, records $q, \dot{q}, \tau$, and utilizes **Least Squares Optimization** to extract exact **Armature, Viscous Friction, and Coulomb Friction** matrices.

<p align="center">
  <img src="media/demo_SYSID.gif" width="1080" alt="Automated System Identification"/>
</p>

### 3. The "Shadow Robot" Debugging Twin
*   This is a collision-free, cyan-colored digital twin that runs alongside the main robot. It reflects the controller's target commands, allowing instant visual verification of tracking error and dynamic deviations.

<p align="center">
  <img src="media/demo_shadow_robot.gif" width="1080" alt="Shadow Robot"/>
</p>

### 4. Software E-Stop and Recovery
*   The emergency stop overrides the current trajectory and generates a safe deceleration trajectory based on physical kinematic limits.

<p align="center">
  <img src="media/demo_estop.gif" width="1080" alt="E-Stop"/>
</p>

## 🚀 Quick Start & Reproduction Guide

You can run this project using either a containerized Docker environment (recommended for avoiding dependency conflicts) or natively on Ubuntu 24.04.

### 📋 Prerequisites

Download the repository and organize your workspace as follows:
```text
~/lite6_ws/
├── mujoco-3.9.0/          # MuJoCo binaries (provided in this repository)
├── src/                   # Source code (lite6_bringup, lite6_planner, etc.)
├── Dockerfile             # Docker configuration
├── run_docker.sh          # Container boot script
└── README.md
```
### 🐳 Option A: Docker Deployment
This method isolates the environment and requires no local ROS 2 installation. Ensure you have a Linux host (Ubuntu 22.04 or higher), [Docker](https://docs.docker.com/engine/install/) and the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) (recommended for GUI rendering and GPU acceleration).

**1. Start the Docker Environment:**
```bash
cd ~/lite6_ws
chmod +x ./run_docker.sh
./run_docker.sh
```
*(Note: The script automatically detects your GPU and chooses suitable drivers if you are using an Intel/AMD GPU or no GPU. It also automatically mounts the `src` folder, so you do not need to rebuild the image when modifying code).*

**2. Build and Launch (Inside Docker):**
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch lite6_bringup system_bringup.launch.py
```
### 💻 Option B: Native Deployment (Ubuntu 24.04)
If you prefer running natively, ensure you have **ROS2 Jazzy** installed on your host machine.

**1. Setup MuJoCo Binaries:**
Move the provided MuJoCo binaries to your home directory and link them to your `.bashrc`.
```bash
mkdir -p ~/.mujoco
cp -r ~/lite6_ws/mujoco-3.9.0 ~/.mujoco/

# Add MuJoCo paths to your bash profile
echo 'export MUJOCO_DIR=~/.mujoco/mujoco-3.9.0' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MUJOCO_DIR/lib' >> ~/.bashrc
source ~/.bashrc
```

**2. Install ROS 2 Dependencies (`rosdep`):**
Use `rosdep` to automatically install all required packages (Pinocchio, Coal, Ruckig, etc.).
```bash
cd ~/lite6_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

**3. Compile and Launch:**
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch lite6_bringup system_bringup.launch.py
```

## 🔮 Future Work
This framework is actively evolving. Upcoming features include:
- **End-Effector Integration:** Adding URDF and controller support for parallel jaw grippers to facilitate pick-and-place tasks.
- **Friction Modeling Improvement:** Replacing the current model with the LuGre friction model.
- **Advanced Control Algorithms:** Impedance control, admittance control, etc.

## 🙏 Acknowledgements

I would like to sincerely thank the creators and maintainers of the following open-source projects and organizations:

*   **[MuJoCo](https://mujoco.org/)**
*   **[MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)**
*   **[ros2_control](https://control.ros.org/)**
*   **[Pinocchio](https://stack-of-tasks.github.io/pinocchio/)**
*   **[Coal / HPP-FCL](https://github.com/coal-library/coal)**
*   **[Ruckig](https://ruckig.com/)**

## 📌 Disclaimer

This is a personal, open-source project. I am not affiliated with, sponsored by, or endorsed by any commercial entities mentioned in this repository. All trademarks and registered trademarks are the property of their respective owners. The software is provided "as is", without warranty of any kind.

## 📬 Contact

If you have any questions about this repository, you can raise an issue or contact [![Email](https://img.shields.io/badge/Email-Contact_Me-informational?style=flat&logo=gmail&logoColor=white)](mailto:wtianyi947@gmail.com) directly.
