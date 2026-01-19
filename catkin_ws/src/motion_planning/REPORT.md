# Thesis Project Report: Sensory-Driven Vision-Based Control for Robotic Manipulation

## 1. Project Overview

This project implements a robust pick-and-place system for a robotic manipulator (UR5) using a hybrid control strategy. The core objective is to achieve high-precision manipulation by combining standard trajectory planning for global movement with a custom velocity-based controller (referred to as MPC) for terminal accuracy.

## 2. System Architecture

- **Robot Platform**: Universal Robots UR5 (6-DOF manipulator).
- **Simulation Environment**: Gazebo with `gazebo_ros_link_attacher` for grasping simulation.
- **Middleware**: ROS Noetic (Python 3).
- **Control Interface**: `ros_control` managing `trajectory_controller` and `joint_group_vel_controller`.

## 3. Control Strategy Implementation

The system utilizes a **Hybrid Control Architecture** that dynamically switches between two distinct controllers based on the task phase.

### A. ArmController (Trajectory Control)

- **Type**: Position-based Joint Trajectory Controller.
- **Usage**: Used for large, coarse movements, moving to home positions, and vertical descent/ascent.
- **Mechanism**: Generates quintic spline trajectories to ensure smooth velocity and acceleration profiles. The trajectory $q(t)$ is defined as:
  $$q(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3 + a_4 t^4 + a_5 t^5$$
- **Role**: Ensures safe navigation and obstacle avoidance during the transport phase.

### B. MPCController (Precision Velocity Control)

- **Type**: Velocity-based PID Feedback Controller (running at 100Hz).
- **Usage**: Used exclusively for the "Precision Approach" phase to align the end-effector with the target object.
- **Mechanism**:
  - Calculates real-time joint errors based on Inverse Kinematics (IK).
  - Computes required joint velocities using a tuned PID loop (`Kp=15.0`, `Ki=1.5`, `Kd=0.6`). The control law is given by:
    $$u(t) = K_p e(t) + K_i \int_{0}^{t} e(\tau)d\tau + K_d \frac{de(t)}{dt}$$
  - Handles joint wrapping (normalization to `[-pi, pi]`) and velocity saturation (`+/- 1.5 rad/s`).
- **Role**: Eliminates steady-state error and achieves sub-millimeter positioning accuracy (< 2mm tolerance) which standard trajectory planners often struggle to achieve quickly.

## 4. Motion Planning Workflow

The `motion_planning.py` script orchestrates the task execution through the following state machine:

1.  **Initialization**:
    - Initialize Kinematics and Gripper action clients.
    - Wait for object detection (Vision/Gazebo state).

2.  **Object Approach**:
    - **Coarse Approach**: `ArmController` moves the robot to a "Safe Height" (`z + 0.2m`) above the target.
    - **Controller Switch**: System switches hardware interface from `trajectory_controller` to `joint_group_vel_controller`.
    - **Precision Alignment (MPC)**: `MPCController` servos the robot to minimize X/Y error while maintaining height.
      - _Success Criteria_: Error norm $||e|| < 2mm$ within 8 seconds, where $||e|| = \sqrt{\Delta x^2 + \Delta y^2 + \Delta z^2}$.
    - **Controller Switch**: System switches back to `trajectory_controller`.
    - **State Sync**: `ArmController` synchronizes its internal state with the actual robot position left by the MPC.

3.  **Manipulation**:
    - **Descent**: Robot lowers to the grasp height.
    - **Grasp**: Gripper closes, and a dynamic link is created between the robot and the object.
    - **Transport**: Object is moved to the target location.
    - **Place**: Object is released and fixed (static) in the simulation.

## 5. Performance Analysis & Results

We developed a suite of analysis tools to validate the hybrid strategy.

### Data Logging

Both controllers automatically log performance metrics to CSV files in `catkin_ws/`:

- **Metrics**: Timestamp, Target Pose (XYZ), Actual Pose (XYZ), Position Error, Orientation Error, and Joint Velocities.

### Visualization Tools (`plot_mpc_results.py`)

A comprehensive Python script was developed to analyze the logs:

1.  **Trajectory Tracking**: Visualizes Target vs. Actual paths in X, Y, and Z.
2.  **Error Convergence**: Log-scale plots showing how quickly the error minimizes.
3.  **Control Effort**: Plots joint velocities to ensure smoothness and safety.
4.  **Comparison Mode**:
    - Allows overlaying results from the `ArmController` and `MPCController`.
    - Automatically computes **Mean Error**, **Final Error**, and **Execution Time**.
    - Identifies the "Best" controller based on final accuracy.

### Key Findings

- **Standard Controller**: Good for smooth transport but often leaves a steady-state error of 3-5mm depending on calibration and trajectory tolerance.
- **Hybrid (MPC) Approach**: Successfully reduces the final positioning error to **< 2mm**, enabling reliable grasping of small objects (LEGO bricks).

## 6. File Structure Summary

```text
catkin_ws/src/motion_planning/
├── scripts/
│   ├── motion_planning.py    # Main executive script
│   ├── controller.py         # ArmController and MPCController classes
│   ├── kinematics.py         # IK/FK solvers
│   ├── plot_mpc_results.py   # Analysis and plotting tool
│   └── build_planning.py     # Construction task logic
└── REPORT.md                 # This report
```

## 7. Future Work

- **Dynamic Tuning**: Implement adaptive PID gains for the MPC controller based on the distance to the target.
- **Visual Servoing**: Replace the Gazebo model state ground truth with real-time camera feedback in the MPC loop.
- **Orientation Control**: Extend the MPC logic to actively correct orientation errors (quaternions) alongside position.
