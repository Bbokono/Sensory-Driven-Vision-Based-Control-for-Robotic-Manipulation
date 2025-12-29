FROM osrf/ros:noetic-desktop-full

# Install basic tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    && rm -rf /var/lib/apt/lists/*