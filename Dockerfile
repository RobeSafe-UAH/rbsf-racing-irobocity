FROM nvidia/cuda:12.4.1-devel-ubuntu22.04 AS base
ENV FORCE_CUDA="1"

# UID/GID matching avoids volume permission issues.
ARG USER
ARG UID
ARG GID

# Timezone — avoid interactive prompts.
ENV TZ=Europe/Madrid
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Create user with sudo access.
RUN groupadd -g $GID $USER \
    && useradd --uid $UID --gid $GID -m $USER \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USER ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USER \
    && chmod 0440 /etc/sudoers.d/$USER

# Base system packages.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-dev \
    python-is-python3 \
    python3-pip \
    python3-setuptools \
    libgl1-mesa-glx \
    mesa-utils \
    libglapi-mesa \
    libqt5gui5 \
    curl \
    wget \
    git \
    build-essential \
    ca-certificates \
    cmake \
    figlet

USER $USER
WORKDIR /.cache

# Terminal prompt.
ENV TERM=xterm-256color
RUN echo "PS1='\[\e[1;34m\]\u\[\e[1;37m\]@\[\e[1;33m\]\h\[\e[1;37m\]:\[\e[0;37m\]\w\[\e[1;37m\] → \[\e[0m\]'" >> ~/.bashrc \
    && echo "source ~/.bashrc" >> ~/.bash_profile

# Locale.
RUN sudo apt-get update \
    && sudo apt-get install -y locales software-properties-common \
    && sudo locale-gen en_US en_US.UTF-8 \
    && sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && sudo add-apt-repository universe
ENV LANG=en_US.UTF-8

# ROS 2 Humble.
RUN ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F 'tag_name' | awk -F\" '{print $4}') \
    && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" \
    && sudo dpkg -i /tmp/ros2-apt-source.deb \
    && sudo rm /tmp/ros2-apt-source.deb \
    && sudo apt-get update \
    && sudo apt-get install -y \
        ros-humble-desktop \
        python3-colcon-common-extensions \
        ros-humble-cv-bridge \
        ros-humble-mvsim \
        ros-humble-ackermann-msgs \
        ros-humble-diagnostic-updater \
        ros-humble-serial-driver \
        ros-humble-asio-cmake-module \
        ros-humble-laser-proc \
        ros-humble-urg-node \
        ros-humble-slam-toolbox \
    && echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "alias set_ros='source /ros2_ws/install/setup.bash'" >> ~/.bashrc

# Create ROS 2 workspace and run initial build.
WORKDIR /ros2_ws/src
WORKDIR /ros2_ws
RUN colcon build
RUN . install/setup.bash

# RealSense drivers.
USER root
RUN apt-get install -y curl gpg apt-transport-https lsb-release \
    && mkdir -p /etc/apt/keyrings \
    && curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc | \
        gpg --dearmor | tee /etc/apt/keyrings/librealsenseai.gpg > /dev/null \
    && echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo $(lsb_release -cs) main" | \
        tee /etc/apt/sources.list.d/librealsense.list \
    && apt-get update \
    && apt-get install -y librealsense2-utils librealsense2-dev \
    && rm -rf /var/lib/apt/lists/*
USER $USER

# Claude Code.
RUN curl -fsSL https://claude.ai/install.sh | bash
ENV PATH="/home/$USER/.local/bin:$PATH"

# UV — Python package manager.
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/
ENV UV_LINK_MODE=copy
ENV PYTHONPATH=/ros2_ws/.venv/lib/python3.10/site-packages

CMD [ "/bin/bash" ]
