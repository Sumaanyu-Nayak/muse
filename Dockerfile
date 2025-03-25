# Use official ROS Noetic base image
FROM ros:noetic-ros-core

ENV DEBIAN_FRONTEND=noninteractive

# Install git
RUN apt-get update && apt-get install -y \
    git \
    libeigen3-dev \
    ros-noetic-dynamic-reconfigure \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*


ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
