FROM ubuntu:22.04

SHELL ["/bin/bash", "-c"]

ARG ROS_DISTRO=humble

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV PYTHONUNBUFFERED=1
ENV OMNI_KIT_ACCEPT_EULA=YES
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV ROS_DISTRO=${ROS_DISTRO}

# Base system + build tools + Isaac Sim runtime libs
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    tzdata \
    curl \
    gnupg2 \
    lsb-release \
    ca-certificates \
    software-properties-common \
    build-essential \
    cmake \
    git \
    pkg-config \
    python3 \
    python3-pip \
    python3-dev \
    python3-setuptools \
    python3-wheel \
    python3-numpy \
    libgl1 \
    libegl1 \
    libglu1-mesa \
    libglvnd0 \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender1 \
    libx11-6 \
    libxrandr2 \
    libxinerama1 \
    libxcursor1 \
    libxi6 \
    libxkbcommon0 \
    libxtst6 \
    libnss3 \
    libasound2 \
    libgtk-3-0 && \
    locale-gen en_US.UTF-8 && \
    update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*

# ROS 2 apt repo
RUN add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
      > /etc/apt/sources.list.d/ros2.list

# ROS + Gazebo bridge + common tooling
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-gripper-controllers \
    ros-${ROS_DISTRO}-pinocchio \
    ros-${ROS_DISTRO}-generate-parameter-library \
    ros-${ROS_DISTRO}-generate-parameter-library-py \
    ros-${ROS_DISTRO}-parameter-traits \
    ros-${ROS_DISTRO}-rsl \
    ros-${ROS_DISTRO}-tcb-span \
    ros-${ROS_DISTRO}-tl-expected \
    ros-${ROS_DISTRO}-gtest-vendor \
    ros-${ROS_DISTRO}-gmock-vendor \
    ros-${ROS_DISTRO}-ament-cmake-gtest \
    ros-${ROS_DISTRO}-ament-cmake-gmock \
    ros-${ROS_DISTRO}-ament-cmake-clang-format \
    ros-${ROS_DISTRO}-ros2controlcli \
    ros-${ROS_DISTRO}-urdfdom-py \
    libgtest-dev \
    libgmock-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

COPY ./workspace/ /workspace/
COPY ./scripts/run_inside_docker.sh /usr/local/bin/run_inside_docker.sh

RUN test -e /workspace/storm && \
    test -e /workspace/IsaacLab && \
    test -e /workspace/ros2_ws/src/storm_agimus_bridge && \
    test -e /workspace/ros2_ws/src/storm_pick_and_place_demo

RUN chmod +x /usr/local/bin/run_inside_docker.sh

ENV STORM_ROOT=/workspace/storm
ENV PYTHONPATH=/workspace/storm

CMD ["bash"]
