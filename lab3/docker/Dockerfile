FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-turtlebot3-gazebo \
    ros-humble-rviz2 \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Convenience: always source ROS
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
