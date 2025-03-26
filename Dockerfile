# Base ROS image
FROM ros:noetic-ros-core

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

# Install dependencies and Pinocchio
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    ros-noetic-dynamic-reconfigure \
    libeigen3-dev \
    ros-$ROS_DISTRO-plotjuggler-ros \
    ros-${ROS_DISTRO}-pinocchio \
    cmake \
    && rm -rf /var/lib/apt/lists/*
    
    
# Optional: Create a workspace directory
WORKDIR /home/dev_ws

# (Optional) Source ROS environment automatically
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

