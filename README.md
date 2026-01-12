# Sensory-Driven Vision-Based Control for Robotic Manipulation

This project implements an advanced manipulation system using a UR5 robot arm, simulated in Gazebo. It integrates computer vision (YOLOv5) for object detection and pose estimation, a level manager for spawning Lego bricks, and a motion planning node for pick-and-place operations.

## Project Overview

The system consists of four main components:

1.  **Gazebo Simulation**: Simulates the UR5 robot, a table, and Lego bricks.
2.  **Level Manager**: Spawns Lego bricks in random or predefined configurations on the table.
3.  **Vision Node**: Uses a camera sensor and YOLOv5 to detect Lego bricks, classify them, and estimate their pose (position and orientation).
4.  **Motion Planning**: Controls the UR5 arm to pick up detected bricks and stack them according to a build plan.
5.  **Obstacles**: Optional spawning of obstacles to test dynamic path planning.

## Prerequisites

- **OS**: Ubuntu 20.04 (Focal Fossa) or compatible Linux distribution.
- **Docker**: For running the simulation in an isolated environment.
- **NVIDIA Drivers (Optional)**: For GPU acceleration (though the current setup uses CPU for PyTorch).

## Installation & Setup

The entire environment is containerized using Docker. A helper script is provided to build the image, install dependencies, and launch the container.

1.  **Clone the Repository** (if not already done):

    ```bash
    git clone <repository_url>
    cd "Sensory-Driven Vision-Based Control for Robotic Manipulation"
    ```

2.  **Run the Simulation Script**:
    Navigate to the scripts directory and run the setup script. This script will:

    - Build the Docker image.
    - Start the container.
    - Install system dependencies (ROS Noetic, Gazebo, etc.).
    - Build the Catkin workspace.
    - Install Python dependencies (PyTorch, YOLOv5, Pandas, etc.).

    ```bash
    cd catkin_ws/src/vision/scripts
    ./run_simulation.sh
    ```

    _Note: The first run will take some time to download and install all dependencies._

## Running the Simulation

Once the setup script completes, it will output instructions on how to run the individual components. You will need **up to five separate terminal windows**.

For each terminal, first connect to the running container:

```bash
docker start ur5_container
 xhost +local:root
```

```bash
docker exec -it -e DISPLAY=$DISPLAY ur5_container bash
```

Then run the specific command for that component.

### Terminal 1: Gazebo World

Launches the simulation environment with the robot and table.

```bash
source /root/catkin_ws/devel/setup.bash
roslaunch levelManager lego_world.launch
```

_Wait for Gazebo to fully load before proceeding._

### Terminal 2: Level Manager

Spawns the Lego bricks. You can choose a level (1-4).

```bash
source /root/catkin_ws/devel/setup.bash
rosrun levelManager levelManager.py -l 1
```

- `-l 1`: Spawns a random brick.
- `-l 2`: Spawns all brick types.
- `-l 4`: Spawns bricks for a specific construction.

### Terminal 3: Motion Planning

Starts the robot controller to pick and place bricks. Note: Run this **before** the vision node so it is ready to receive detections.

```bash
source /root/catkin_ws/devel/setup.bash
rosservice call /gazebo/unpause_physics
rosrun motion_planning motion_planning.py
```

### Terminal 4: Vision Node

Starts the computer vision processing.

```bash
source /root/catkin_ws/devel/setup.bash
rosrun vision lego-vision.py -show
```

    `-show`: Displays a window with the camera feed and detection overlays.

### Terminal 5: Spawn Obstacles (Optional)

To test path planning capabilities, spawn a static obstacle in the workspace.

```bash
source /root/catkin_ws/devel/setup.bash
rosrun motion_planning spawn_obstacle.py
```

## Troubleshooting

- **"No module named ..."**: If you encounter missing Python modules, ensure the `run_simulation.sh` script has finished completely. You can manually install packages inside the container using `pip3 install <package_name>`.
- **Gazebo GUI issues**: If Gazebo doesn't show up, ensure you have allowed X11 forwarding (`xhost +`) on your host machine, which the script attempts to do automatically.
- **Controller Spawner Errors**: Occasionally, Gazebo might fail to spawn controllers. Restarting the simulation (Terminal 1) usually fixes this.

## Work Log

- **Environment Setup**: Configured Docker container with ROS Noetic and Gazebo.
- **Dependency Management**: Automated installation of ROS packages and Python libraries (pandas, scipy, torch, etc.).
- **Vision Integration**: Integrated YOLOv5 for object detection and custom pose estimation logic.
- **Motion Planning**: Implemented pick-and-place logic using MoveIt/Forward Kinematics and a custom gripper controller.
- **Simulation**: Successfully ran full loop simulation of detecting and stacking Lego bricks.
