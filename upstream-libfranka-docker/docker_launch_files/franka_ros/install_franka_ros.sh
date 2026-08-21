#!/bin/bash

### LIBFRANKA
cd /docker_volume/libfranka

# make sure that libfranka is checked out to the compatible branch
# see https://frankaemika.github.io/docs/compatibility.html
# if [ ! -d "build" ]
# then 
#     mkdir build && cd build
#     cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF  ..
#     cmake --build . -j$(nproc)
#     cpack -G DEB
#     sudo dpkg -i libfranka-*.deb
# fi


### FRANKA ROS
cd /docker_volume/catkin_ws
source /opt/ros/noetic/setup.sh
if [ ! -d "build" ]
then 
    catkin_init_workspace src
    rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka
    catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/docker_volume/libfranka/build
    source devel/setup.sh
fi
source devel/setup.sh

cd /docker_volume

echo "Attaching to container"
exec "$@"

