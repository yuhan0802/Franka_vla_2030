FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    WORKSPACE=/ws_franka_vr

RUN apt-get update && apt-get install -y --no-install-recommends \
    adb \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libfmt-dev \
    libignition-gazebo6-dev \
    libignition-plugin-dev \
    libpoco-dev \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    ros-humble-generate-parameter-library \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    ros-humble-moveit \
    ros-humble-moveit-servo \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-tf-transformations \
    && rm -rf /var/lib/apt/lists/*

COPY vendor/libfranka_0.19.0_jammy_amd64.deb /tmp/libfranka.deb
RUN echo "3f8f8acc53311aab0bab05d1bb6d245005e67bfd1e11928ec760baf56c2b1a32  /tmp/libfranka.deb" \
      | sha256sum --check - \
    && apt-get update \
    && apt-get install -y --no-install-recommends /tmp/libfranka.deb \
    && rm -f /tmp/libfranka.deb \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir numpy pure-python-adb pyyaml

WORKDIR ${WORKSPACE}
COPY workspace/src ./src
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && rosdep update \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} --skip-keys "libfranka olv_module_descriptions realsense2_camera realsense2_description sick_safetyscanners2 hardware_interface_testing" \
    && colcon build --symlink-install --packages-up-to franka_vr franka_gazebo_bringup franka_example_controllers --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
