# MPC Execution Task

This document outlines the steps to run the simulation with the Model Predictive Control (MPC) integrated into the motion planning loop.

## Prerequisites

Ensure you have run the setup script at least once to build the Docker container and workspace.

## Rebuild and Run

To apply changes (like the new velocity controller configuration) and start the environment:

1. Navigate to the scripts directory:

   ```bash
   cd catkin_ws/src/vision/scripts
   ```

2. Run the simulation script:
   ```bash
   ./run_simulation.sh
   ```
   _If prompted to restart the container, type `y` and press Enter._

## Execution Steps

Open 4 separate terminal windows. In each terminal, first connect to the container:

```bash
docker exec -it -e DISPLAY=$DISPLAY ur5_container bash
```

### Terminal 1: Gazebo Simulation

Starts the physics simulator and robot controllers.

```bash
source /root/catkin_ws/devel/setup.bash
roslaunch levelManager lego_world.launch
```

### Terminal 2: Level Manager

Spawns the Lego bricks on the table.

```bash
source /root/catkin_ws/devel/setup.bash
rosservice call /gazebo/unpause_physics
rosrun levelManager levelManager.py -l 1
```

### Terminal 3: Spawn Obstacle

Adds a static obstacle (red box) to the Gazebo world.

```bash
source /root/catkin_ws/devel/setup.bash
rosrun motion_planning spawn_obstacle.py
```

### Terminal 4: Motion Planning (MPC/MoveIt)

Runs the robot controller. It uses MoveIt for path planning around the obstacle.
_Note: Ensure `joint_group_vel_controller` is loaded if testing MPC logic._

```bash
source /root/catkin_ws/devel/setup.bash
rosrun motion_planning motion_planning.py
```

### Terminal 5: Vision Node

Starts the object detection system.

```bash
source /root/catkin_ws/devel/setup.bash
rosrun vision lego-vision.py -show
```
