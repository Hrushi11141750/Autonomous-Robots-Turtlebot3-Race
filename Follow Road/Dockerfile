FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive


# Install dependencies
RUN apt update && apt install -y \
    ros-humble-turtlebot3* \
    ros-humble-gazebo-ros-pkgs \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
RUN mkdir -p /root/ros2_ws/src

# Copy package to workspace (relative to Dockerfile location)
COPY ./autorace_real /root/ros2_ws/src/autorace_real

# Build the package
RUN ["/bin/bash", "-c", "\
    source /opt/ros/humble/setup.bash && \
    cd /root/ros2_ws && \
    colcon build --symlink-install --packages-select autorace_real"]

# Setup environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc
