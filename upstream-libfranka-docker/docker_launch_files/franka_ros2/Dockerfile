FROM ros:humble

### Make installation non interactive so that docker doesnt crash during build
ENV DEBIAN_FRONTEND noninteractive

### Basic OS setup
RUN apt-get update -y &&\
    apt-get install -y \
        git \
        cmake \
        python3-pip \
        build-essential \
        libeigen3-dev \
        libpoco-dev

### ROS 2 Setup
RUN apt-get update -y &&\
    apt-get install -y --allow-unauthenticated \
        ros-humble-ament-cmake \
        ros-humble-ament-cmake-clang-format \
        ros-humble-angles \
        ros-humble-ros2-controllers \
        ros-humble-ros2-control \
        ros-humble-ros2-control-test-assets \
        ros-humble-controller-manager \
        ros-humble-control-msgs \
        ros-humble-control-toolbox \
        ros-humble-generate-parameter-library \
        ros-humble-joint-state-publisher \
        ros-humble-joint-state-publisher-gui \
        ros-humble-moveit \
        ros-humble-pinocchio \
        ros-humble-realtime-tools \
        ros-humble-xacro \
        ros-humble-hardware-interface \
        ros-humble-ros-gz &&\
    rm -rf /var/lib/apt/lists/*

### Install python extensions for display mounting and building
RUN apt-get update -y &&\
    apt-get install -y  \
        python3-tk \
        python3-gi-cairo \
        python3-colcon-common-extensions
    
WORKDIR /docker_volume

### Install Libfranka
COPY ./install_franka_ros2.sh /install_franka_ros2.sh
RUN chmod +x /install_franka_ros2.sh
ENTRYPOINT [ "/install_franka_ros2.sh" ]

CMD ["/bin/bash"]

### Important: for every reboot: run xhost +local:docker
