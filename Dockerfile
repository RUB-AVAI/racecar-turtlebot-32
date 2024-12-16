# Use the official ROS2 humble base image
FROM ros:humble-ros-base-jammy

# Install necessary dependencies for ROS2 workspace
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop=0.10.0-1* \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/* \
    apt install python3-pip

# Set up workspace and set environment variables
ENV ROS_WS=/workspace

# Copy any setup files (if necessary)
# COPY ./src /workspace/src

# Ensure ROS2 setup is sourced automatically on container start
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /workspace/src/install/local_setup.bash" >> /root/.bashrc

# Default working directory (it can be overwritten by volume)
WORKDIR /workspace

# Default command to start a bash shell
CMD ["bash"]
