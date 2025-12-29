#!/bin/bash

# This script sets up and runs the UR5 simulation environment in a Docker container.
# It handles container creation, dependency installation, and launching the ROS application.

set -e

echo "--- Preparing Docker Environment ---"

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
CATKIN_WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

# Build the Docker image if it doesn't exist
if [[ "$(docker images -q ur5_sim 2> /dev/null)" == "" ]]; then
  echo "Image 'ur5_sim' not found. Building it..."
  PROJECT_ROOT="$SCRIPT_DIR/../../../../"
  docker build -t ur5_sim "$PROJECT_ROOT"
fi

# Allow Docker to connect to the X server
xhost +

# Check for existing container status
if docker container inspect ur5_container >/dev/null 2>&1; then
  STATUS=$(docker inspect -f '{{.State.Status}}' ur5_container)
  if [ "$STATUS" == "exited" ]; then
    echo "Container 'ur5_container' exists but is stopped."
    echo "WARNING: If you are having GUI issues, choose 'n' to recreate the container with correct settings."
    read -p "Do you want to restart it? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
      echo "Starting container..."
      docker start ur5_container
      echo "Container started."
      echo "You can now run your application components in separate terminals."
      echo "Here are the commands to run in separate terminals:"
      echo ""
      echo "Terminal 1 (Gazebo World):"
      echo "docker exec -it -e DISPLAY=\$DISPLAY ur5_container bash -c 'source /root/catkin_ws/devel/setup.bash && roslaunch levelManager lego_world.launch'"
      echo ""
      echo "Terminal 2 (Level Manager - run after Gazebo loads):"
      echo "docker exec -it -e DISPLAY=\$DISPLAY ur5_container bash -c 'source /root/catkin_ws/devel/setup.bash && rosservice call /gazebo/unpause_physics && rosrun levelManager levelManager.py -l 1'"
      echo ""
      echo "Terminal 3 (Vision):"
      echo "docker exec -it -e DISPLAY=\$DISPLAY ur5_container bash -c 'source /root/catkin_ws/devel/setup.bash && rosrun vision lego-vision.py -show'"
      echo ""
      echo "Terminal 4 (Motion Planning):"
      echo "docker exec -it -e DISPLAY=\$DISPLAY ur5_container bash -c 'source /root/catkin_ws/devel/setup.bash && rosrun motion_planning motion_planning.py'"
      # Exit to avoid rebuilding
      exit 0
    fi
  fi

  # Stop and remove any existing container with the same name
  echo "Removing old container if it exists..."
  docker rm -f ur5_container 2>/dev/null || true
fi

echo "--- Starting UR5 Simulation Container ---"

# Run the Docker container. The -d flag runs it in detached mode.
# We will use 'docker exec' to run commands inside it.

docker run -d -it --name ur5_container \
  --net=host \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="LIBGL_ALWAYS_SOFTWARE=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="/run/user/$(id -u)/pulse:/run/user/1000/pulse" \
  --volume="$CATKIN_WS_DIR:/root/catkin_ws" \
  --privileged \
  ur5_sim

echo "--- Installing Dependencies inside the Container ---"

# Execute the setup script inside the container
docker exec -it ur5_container /bin/bash -c '
  set -e

  echo "Updating APT and installing system packages..."
  apt update && apt install -y \
      ros-noetic-gazebo-ros \
      ros-noetic-gazebo-ros-pkgs \
      ros-noetic-gazebo-ros-control \
      ros-noetic-ros-control \
      ros-noetic-ros-controllers \
      python3-pip \
      libgazebo11-dev \
      ros-noetic-rqt-image-view \
      pulseaudio-utils \
      python3-catkin-tools

  echo "Building Catkin workspace..."
  cd /root/catkin_ws
  source /opt/ros/noetic/setup.bash
  catkin clean -y
  catkin config --extend /opt/ros/noetic
  catkin build
  source devel/setup.bash
  echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

  echo "Installing Python packages..."
  python3 -m pip install --upgrade pip
  python3 -m pip install --upgrade importlib-metadata setuptools
  python3 -m pip install --ignore-installed psutil
  python3 -m pip install \
      pandas \
      pyquaternion \
      requests \
      seaborn \
      "networkx<3.0" \
      ultralytics \
      torch==2.2.2+cpu \
      torchvision==0.17.2+cpu \
      scipy \
      matplotlib \
      opencv-python \
      gitpython \
      tqdm \
      pillow \
      thop \
      -f https://download.pytorch.org/whl/torch_stable.html

  echo "Cloning YOLOv5 and installing its dependencies..."
  cd ~
  if [ ! -d "yolov5" ]; then
    git clone https://github.com/ultralytics/yolov5.git
  fi
  cd yolov5
  python3 -m pip install -r requirements.txt
'

echo "--- Setup Complete ---"
echo "You can now run your application components in separate terminals using 'docker exec'."
echo "Here are the commands to run in separate terminals:"
echo ""
echo "Terminal 1 (Gazebo World):"
echo "docker exec -it -e DISPLAY=\$DISPLAY ur5_container bash -c 'source /root/catkin_ws/devel/setup.bash && roslaunch levelManager lego_world.launch'"
echo ""
echo "Terminal 2 (Level Manager - run after Gazebo loads):"
echo "docker exec -it -e DISPLAY=\$DISPLAY ur5_container bash -c 'source /root/catkin_ws/devel/setup.bash && rosservice call /gazebo/unpause_physics && rosrun levelManager levelManager.py -l 1'"
echo ""
echo "Terminal 3 (Vision):"
echo "docker exec -it -e DISPLAY=\$DISPLAY ur5_container bash -c 'source /root/catkin_ws/devel/setup.bash && rosrun vision lego-vision.py -show'"
echo ""
echo "Terminal 4 (Motion Planning):"
echo "docker exec -it -e DISPLAY=\$DISPLAY ur5_container bash -c 'source /root/catkin_ws/devel/setup.bash && rosrun motion_planning motion_planning.py'"