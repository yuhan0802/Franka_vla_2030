#!/bin/bash

### LIBFRANKA
cd /docker_volume/libfranka_v0.13.2

# make sure that libfranka is checked out to the compatible branch (required 0.13.3 as of 08/2024)
# see https://frankaemika.github.io/docs/compatibility.html
if [ ! -d "build" ]
then 
    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF  ..
    cmake --build . -j$(nproc)
    cpack -G DEB
    sudo dpkg -i libfranka-*.deb
fi


### FRANKA ROS2
source /opt/ros/humble/setup.sh
cd /docker_volume/ros2_ws
if [ ! -d "install" ]
then 
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/docker_volume/libfranka_v0.13.2/build
fi
source install/setup.sh


cd /docker_volume

echo "Attaching to container"
exec "$@"