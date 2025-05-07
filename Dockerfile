# Base image with ROS 2 ${ROS_DISTRO} Desktop
FROM osrf/ros:jazzy-desktop

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ARG UID=1000
# ENV GID=1000
# RUN awk -F: '/\/home/ {printf "%s:%s\n",$1,$3}' /etc/passwd

# RUN usermod -l bmstu ubuntu \
#     -d /bmstu -m \
#     -s /usr/bin/bash \
#     -G dialout \
#     -c "BMSTU YRC" bmstu && \
#     echo "bmstu ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers
# RUN whoami
# USER ubuntu
# RUN whoami
RUN usermod -l  bmstu ubuntu && \
    groupmod -n bmstu ubuntu && \
    usermod -d /bmstu -m  bmstu  && \
    usermod -c "BMSTU YRC" bmstu
    # RUN usermod -G dialout
RUN echo "bmstu ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# Update and install required tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    build-essential \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-common-plugins \
    ros-${ROS_DISTRO}-realsense2-camera \
    # ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    git \
    nano \
    tmux \
    curl \
    wget \
    locales \
    && apt-get clean

# Set locale
RUN locale-gen en_US en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8

RUN sudo apt-get update \
    && sudo apt-get install -y ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-joint-state-publisher-gui \
    && sudo apt-get install -y ros-${ROS_DISTRO}-tf-transformations  ros-${ROS_DISTRO}-rqt-tf-tree \
    && sudo rm -rf /var/lib/apt/lists/*

RUN sudo apt update \
    && sudo apt install -y ros-${ROS_DISTRO}-navigation2 \
    && sudo apt install -y ros-${ROS_DISTRO}-nav2-bringup \
    && sudo apt install -y ros-${ROS_DISTRO}-slam-toolbox \
    && sudo rm -rf /var/lib/apt/lists/*

RUN pip install opencv-python --no-cache-dir --break-system-packages


# Install gazebo python3-gz-transport13
RUN sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
  && sudo apt-get update && sudo apt-get install -q -y --no-install-recommends \
  ros-${ROS_DISTRO}-ros-gz   python3-gz-transport13 \
  && sudo apt-get install -y libgz-sim8-dev libgz-math8-dev libgz-common6-dev libgz-plugin3-dev \
  && sudo rm -rf /var/lib/apt/lists/*

RUN sudo apt update && \
    sudo apt install -y python3-serial && \
    sudo rm -rf /var/lib/apt/lists/*

RUN pip install pymavlink --no-cache-dir --break-system-packages

# Source ROS 2 on container start
# RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /bmstu/.bashrc
# RUN echo "export PS1='${debian_chroot:+($debian_chroot)}\[\033[01;31m\]\u@\h\[\033[00m\]:\[\033[01;32m\]\w\[\033[00m\]\$ '" >> /bmstu/.bashrc
COPY ./bashrc  /tmp/bashrc
RUN cat /tmp/bashrc >> /bmstu/.bashrc &&\
    rm -f /tmp/bashrc &&\
    chown -R bmstu:bmstu /bmstu

USER bmstu
# Set working directory
WORKDIR /bmstu/ros2_ws

# Create entrypoint
COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN sudo chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
