########################################################################
#### Ubuntu 
########################################################################
FROM ubuntu:24.04 AS base

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=jazzy

ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=${ROS_DISTRO}

# Base OS deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

########################################################################
#### NVIDIA graphics drivers
########################################################################
RUN apt update && apt install -y --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
    libgles2 \
    && rm -rf /var/lib/apt/lists/*

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV TURTLEBOT3_MODEL=burger

########################################################################
#### BUild tools and common dependencies
########################################################################
RUN apt update && apt install -q -y --no-install-recommends \
    build-essential \
    bash-completion \
    cmake \
    dirmngr \
    git \
    gnupg2 \
    lsb-release \
    wget \
    curl \
    clang \
    ca-certificates \
    iputils-ping \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

########################################################################
#### ROS2 installation + Gazebo
########################################################################

RUN apt update && apt install -y --no-install-recommends curl \
    && export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') \
    && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" \
    && dpkg -i /tmp/ros2-apt-source.deb \
    && rm /tmp/ros2-apt-source.deb

RUN apt update && apt install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-turtlesim \
    ros-${ROS_DISTRO}-demo-nodes-cpp \
    ros-${ROS_DISTRO}-demo-nodes-py \
    ros-${ROS_DISTRO}-turtlebot3 \
    ros-${ROS_DISTRO}-turtlebot3-simulations \
    ros-${ROS_DISTRO}-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

# install packages
RUN apt update && apt install -q -y --no-install-recommends \
    python3-apt \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-pip \
    python3-rosdep \
    python3-testresources \
    python3-vcstool \
    python3-colcon-common-extensions \
    python3-pytest \
    python3-pytest-cov \
    python3-setuptools \
    python3-flake8 \
    python3-flake8-blind-except \
    python3-flake8-builtins \
    python3-flake8-class-newline \
    python3-flake8-comprehensions \
    python3-flake8-deprecated \
    python3-flake8-docstrings \
    python3-flake8-import-order \
    python3-flake8-quotes \
    python3-pytest-repeat \
    python3-pytest-rerunfailures \
    python3-requests \
    nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*


# Initialize rosdep
RUN rosdep init && \
    rosdep update

COPY . /opt/ws/src/code

########################################################################
#### Setup entrypoint
########################################################################
COPY docker/entrypoint.bash /entrypoint.bash
RUN chmod +x /entrypoint.bash
ENTRYPOINT ["/entrypoint.bash"]

CMD ["bash"]
