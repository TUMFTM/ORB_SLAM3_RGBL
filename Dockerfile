FROM ubuntu:20.04

# set environment variables
ENV DEBIAN_FRONTEND=noninteractive 
ENV ROS_DISTRO=galactic

ENV TZ=${TZ:-Europe/Berlin}
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_VERSION=2
ENV ROS_DISTRO=${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3
ENV RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}]: {message}"

# Install dependencies
RUN apt-get update && apt-get install -y -q --no-install-recommends \
    build-essential \
    ca-certificates \
    curl \
    gnupg2 \
    locales \
    lsb-release \
    python3-pip \
    git \
    vim \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y -q cmake libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev \
    python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev \
    libglew-dev libboost-all-dev libssl-dev \
    libeigen3-dev mesa-utils nano
    # libjasper-dev

# Generate and set locales
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add, install and source ROS2 distro
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y -q --no-install-recommends \
    ros-$ROS_DISTRO-ros-base \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-example-interfaces \
    ros-$ROS_DISTRO-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

RUN pip install -U colcon-common-extensions
RUN pip install pytest==5.3

# Install OpenCV 3.4.13
RUN cd ~ && \
    mkdir Dev && cd Dev && \
    git clone --branch 4.x --depth 1 https://github.com/opencv/opencv.git && \
    cd opencv && \
    mkdir build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release  -D CMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j 4 && \
    make install

# Install Pangolin
RUN cd ~/Dev && \
    git clone --depth 1 https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    mkdir build && \
    cd build && \
    cmake .. -D CMAKE_BUILD_TYPE=Release && \
    make -j 4 && \
    make install

RUN ldconfig

COPY . /ORBSLAM3

RUN cd /ORBSLAM3 && rm -rf build && ./scripts/build.sh
