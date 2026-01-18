# MPC (Model Predictive Control) Implementation Guide

## Overview

This document describes the production-ready Model Predictive Control (MPC) implementation for the UR5 robot in Gazebo. The MPC controller replaces the traditional trajectory-based approach with a real-time optimization framework that continuously solves a quadratic program to minimize tracking error while respecting joint velocity constraints.

## Key Features

### 1. **Optimized Controller Design**

- **Fast Jacobian Computation**: Uses analytical finite-difference based Jacobian instead of inefficient numerical differentiation
- **Real-time QP Solver**: SLSQP solver with tight tolerances (ftol=1e-2) and limited iterations (maxiter=15) for <50ms computation time
- **Velocity Ramping**: Smooth acceleration profile with 3 rad/s² limit to avoid jerky motions
- **Stuck Motion Detection**: Automatic detection and recovery when robot motion plateaus
- **Timeout Handling**: 45-second timeout with graceful shutdown and logging

### 2. **Robust State Management**

- Continuous joint state feedback from `/joint_states` topic
- Automatic FK updates for gripper pose
- Forward/inverse kinematics using UR5 DH parameters
- Velocity history for acceleration limiting

### 3. **Performance Tuning Parameters**

```python
self.dt = 0.05        # 50ms control cycle (20 Hz)
self.v_max = 2.5      # Joint velocity limit (rad/s)
self.max_cart_speed = 0.8  # m/s Cartesian speed
self.tracking_gain = 0.8   # Position error P-gain
self.rotation_gain = 0.6   # Rotation error P-gain
self.error_threshold = 0.003  # 3mm target tolerance
```

### 4. **Logging & Analysis**

- Real-time CSV logging with timestamps, actual/target positions, computation times
- Automatic log saving on node shutdown to `/root/catkin_ws/mpc_results_TIMESTAMP.csv`

## Implementation Details

### A. Control Law

The MPC solves the following optimization problem at each 50ms cycle:

```
minimize: ||J(q) * dq - v_ref||² + λ * ||dq||²
subject to: |dq_i| ≤ v_max, i = 1...6
```

Where:

- `J(q)` is the 6×6 Jacobian matrix (position + rotation)
- `dq` are the optimal joint velocities
- `v_ref` is the reference 6D velocity (Cartesian + angular)
- `λ` is regularization weight (0.00001)

### B. Velocity Reference Computation

```python
# Position error -> Cartesian velocity (proportional control)
v_cart = pos_error * self.tracking_gain
v_cart = v_cart / norm(v_cart) * min(norm(v_cart), max_cart_speed)

# Orientation error -> Angular velocity (quaternion-based)
q_err = target_quat * current_quat^(-1)
rot_error = q_err.vector * self.rotation_gain

v_ref = [v_cart, rot_error]
```

### C. Jacobian Computation

Analytical Jacobian computed using finite-differences on forward kinematics:

```
J = [ ∂p/∂q ]  where p is end-effector position (3×6)
    [ ∂ω/∂q ]  where ω is orientation rate (3×6)
```

Position Jacobian: Standard finite-difference
Orientation Jacobian: Quaternion-based (2 \* q_diff.vector)

### D. QP Solver

Uses scipy's SLSQP method with:

- Quadratic cost function (Hessian pre-computed)
- Linear bound constraints on joint velocities
- Warm-start from previous solution for better convergence
- Tight tolerance (1e-2) and limited iterations (15 max)

### E. Velocity Integration

```python
q_next = q_current + ramp_velocity(dq_optimal) * dt
```

Commands are sent as position setpoints to `JointGroupPositionController`, which integrates velocities internally.

## File Structure

```
catkin_ws/src/motion_planning/scripts/
├── controller.py           # MPC and ArmController classes
├── motion_planning.py      # Main node with controller switching
├── kinematics.py          # UR5 FK/IK functions
└── ...
```

## Setup & Configuration

### 1. Robot Configuration (ur5gripper_controllers.yaml)

```yaml
joint_group_pos_controller:
  type: position_controllers/JointGroupPositionController
  joints:
    - shoulder_pan_joint
    - shoulder_lift_joint
    - elbow_joint
    - wrist_1_joint
    - wrist_2_joint
    - wrist_3_joint
```

### 2. Gazebo Setup

Ensure these are loaded in your launch file:

```xml
<rosparam file="$(find ur5_gazebo)/controller/ur5gripper_controllers.yaml" command="load"/>
<node name="controller_spawner" pkg="controller_manager" type="spawner" args="joint_state_controller trajectory_controller gripper_controller joint_group_pos_controller"/>
```

### 3. Disable Ignition Fuel Downloads (Optional)

To avoid libcurl timeout errors:

```bash
mkdir -p ~/.ignition/fuel
echo "---" > ~/.ignition/fuel/config.yaml
echo "servers: []" >> ~/.ignition/fuel/config.yaml
```

## Usage

### Running the Full System

**Terminal 1: Gazebo Simulation**

```bash
source /root/catkin_ws/devel/setup.bash
roslaunch levelManager lego_world.launch
```

**Terminal 2: Level Manager (spawn bricks)**

```bash
source /root/catkin_ws/devel/setup.bash
rosservice call /gazebo/unpause_physics
rosrun levelManager levelManager.py -l 2  # Spawn 2 levels of bricks
```

**Terminal 3: Vision System (optional, for real-time detection)**

```bash
source /root/catkin_ws/devel/setup.bash
rosrun vision lego-vision.py -show
```

**Terminal 4: Motion Planning with MPC**

```bash
source /root/catkin_ws/devel/setup.bash
rosrun motion_planning motion_planning.py
```

If vision is not running, the system will automatically fallback to Gazebo ground truth after 30 seconds.

### Running Individual Movements

```python
import rospy
from motion_planning.controller import MPCController

rospy.init_node('mpc_test')
controller = MPCController()

# Absolute movement
controller.move_to(x=0.3, y=-0.2, z=0.8)

# Relative movement
controller.move(dx=0.1, dy=0, dz=-0.2)

# With specific orientation
from pyquaternion import Quaternion
quat = Quaternion(axis=[0,1,0], angle=3.14159/2)
controller.move_to(x=0.3, y=-0.2, z=0.8, target_quat=quat)
```

## Performance Metrics

Expected performance on UR5 in Gazebo:

| Metric                   | Value                            |
| ------------------------ | -------------------------------- |
| Control Cycle            | 50 ms (20 Hz)                    |
| Computation Time         | 20-40 ms (including FK/Jacobian) |
| Max Joint Velocity       | 2.5 rad/s                        |
| Max Cartesian Speed      | 0.8 m/s                          |
| Position Error Threshold | 3 mm                             |
| Movement Time (0.3m)     | 0.5-1.0 s                        |
| Accuracy                 | ±2-3 mm steady-state             |

## Troubleshooting

### Issue: Robot doesn't move

**Check:**

1. Verify controller is running: `rosservice call /controller_manager/list_controllers`
2. Ensure `joint_group_pos_controller` is in the "running" state
3. Check `/joint_states` topic is being published: `rostopic echo /joint_states`
4. Verify kinematics module is correctly imported

### Issue: Very slow movement

**Solutions:**

- Increase `tracking_gain` (default 0.8, try 1.2)
- Increase `max_cart_speed` (default 0.8, try 1.2)
- Reduce `error_threshold` (default 0.003, try 0.005)

### Issue: Jerky/oscillatory motion

**Solutions:**

- Decrease `tracking_gain` (try 0.5)
- Increase `dt` to 0.1 (slower control cycle)
- Increase regularization `lambda_vel` (try 0.001)

### Issue: Timeout errors

**Increase timeout:**

```python
# In controller.py move_to() function
timeout_duration = rospy.Duration(90.0)  # Increase from 45s
```

### Issue: libcurl connection errors

These are Gazebo trying to connect to the model database and don't affect control. Can be safely ignored or disabled (see Setup section).

## Advanced Tuning

### For Faster Movement

```python
self.dt = 0.05              # Keep or decrease
self.v_max = 3.5            # Increase from 2.5
self.tracking_gain = 1.2    # Increase from 0.8
self.max_cart_speed = 1.2   # Increase from 0.8
self.error_threshold = 0.005  # Relax from 0.003
```

### For More Precise Movement

```python
self.dt = 0.033             # Increase control frequency
self.v_max = 1.5            # Reduce from 2.5
self.tracking_gain = 0.5    # Reduce from 0.8
self.max_cart_speed = 0.4   # Reduce from 0.8
self.error_threshold = 0.001  # Tighten from 0.003
self.lambda_vel = 0.0001    # Increase regularization
```

### For Smooth Motion (Minimal Jerks)

```python
self.dt = 0.1               # Slower control cycle
# In ramp_velocity():
max_accel = 1.5             # Reduce from 3.0
```

## Output Analysis

Performance data is saved to `/root/catkin_ws/mpc_results_YYYYMMDD_HHMMSS.csv`

Columns:

- `time`: ROS timestamp
- `target_x, target_y, target_z`: Target position
- `actual_x, actual_y, actual_z`: Achieved position
- `error_dist`: Euclidean distance error (m)
- `computation_time`: MPC solver time (s)
- `v1-v6`: Joint velocities (rad/s)

### Analyze Results

```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('mpc_results_20260118_051927.csv')

# Plot error over time
plt.figure()
plt.plot(df['time'], df['error_dist'] * 1000)  # Convert to mm
plt.xlabel('Time (s)')
plt.ylabel('Position Error (mm)')
plt.title('MPC Tracking Error')
plt.grid()
plt.show()

# Plot joint velocities
plt.figure()
for i in range(1, 7):
    plt.plot(df['time'], df[f'v{i}'], label=f'Joint {i}')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (rad/s)')
plt.title('Joint Velocities')
plt.legend()
plt.grid()
plt.show()
```

## References

- UR5 Kinematics: DH parameters defined in `kinematics.py`
- MPC Theory: Real-time constrained optimization
- ROS Controllers: `position_controllers/JointGroupPositionController`

## Future Improvements

1. **Collision Avoidance**: Add obstacle constraints to QP formulation
2. **Multi-step Prediction**: Horizon > 1 for predictive control
3. **Adaptive Gains**: Self-tuning based on tracking error history
4. **Load Estimation**: Identify end-effector payload and adapt control
5. **GPU Acceleration**: Use CuPy for Jacobian computation on NVIDIA systems

---

**Author**: MPC Implementation for Robotic Manipulation  
**Date**: January 2026  
**Status**: Production-Ready
