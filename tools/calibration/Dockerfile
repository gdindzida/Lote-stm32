FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

# -----------------------------
# Basic system tools
# -----------------------------
RUN apt update && apt install -y \
    curl gnupg2 lsb-release \
    software-properties-common \
    build-essential cmake git \
    python3 python3-pip python3-dev \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# Add ROS Noetic repo
# -----------------------------
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1.list'

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# IMPORTANT: update AFTER adding repo
RUN apt update

# -----------------------------
# Install ROS Noetic
# -----------------------------
RUN apt install -y ros-noetic-desktop-full

# -----------------------------
# ROS build tools
# -----------------------------
RUN apt install -y \
    python3-catkin-tools \
    python3-osrf-pycommon \
    ros-noetic-catkin

# -----------------------------
# Kalibr dependencies
# -----------------------------
RUN apt install -y \
    libopencv-dev \
    libeigen3-dev \
    libboost-all-dev \
    libsuitesparse-dev \
    libblas-dev \
    liblapack-dev \
    libv4l-dev \
    libgl1-mesa-dev \
    libglib2.0-dev \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# Python deps
# -----------------------------
RUN pip3 install numpy scipy matplotlib

# -----------------------------
# Workspace
# -----------------------------
RUN mkdir -p /kalibr_ws/src
WORKDIR /kalibr_ws/src

# -----------------------------
# Kalibr source
# -----------------------------
RUN git clone https://github.com/ethz-asl/kalibr.git

WORKDIR /kalibr_ws

# -----------------------------
# Build Kalibr (stable)
# -----------------------------
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make -j1"

# -----------------------------
# Auto source
# -----------------------------
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /kalibr_ws/devel/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]