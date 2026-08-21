FROM ros:noetic

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
        libpoco-dev \
        sudo

### ROS setup
RUN apt-get update -y &&\
    apt-get install -y \
        ros-noetic-rviz

### Install display mounting tools
RUN apt-get update -y &&\
    apt-get install -y  \
        python3-tk \
        python3-gi-cairo

### Create a non-root user and add to sudoers without password
RUN useradd -m dockeruser && \
    echo "dockeruser ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    mkdir -p /docker_volume && \
    chown dockeruser:dockeruser /docker_volume

### Set Working Directory    
WORKDIR /docker_volume

### Install Libfranka
COPY ./install_franka_ros.sh /install_franka_ros.sh
RUN chmod +x /install_franka_ros.sh && \
    chown dockeruser:dockeruser /install_franka_ros.sh

### Switch to non-root user
USER dockeruser

# Pre-create directories that will be needed
RUN mkdir -p /docker_volume/libfranka /docker_volume/catkin_ws/src

ENTRYPOINT [ "/install_franka_ros.sh" ]

CMD ["/bin/bash"]