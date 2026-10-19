cmake_minimum_required(VERSION 3.8)
project(franka_vr)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(THIS_PACKAGE_INCLUDE_DEPENDS
    control_msgs
    controller_interface
    diagnostic_msgs
    geometry_msgs
    hardware_interface
    moveit_core
    moveit_msgs
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    rclcpp
    rclcpp_components
    realtime_tools
    sensor_msgs
    std_msgs
    std_srvs
    tf2_geometry_msgs
    tf2_eigen
    tf2_ros
    trajectory_msgs
    visualization_msgs)
# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)  # 添加geometry_msgs依赖
find_package(moveit_servo REQUIRED)
foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()
# 生成服务接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetStepSize.srv"
  "srv/SetTargetPose.srv"  # 添加新服务
  DEPENDENCIES std_msgs geometry_msgs
)
# 获取生成接口的目标
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# Executable vr controller (velocity controller)
add_executable(demo_franka_vr_vel src/franka_vr_vel.cpp)
target_include_directories(demo_franka_vr_vel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(demo_franka_vr_vel 
  moveit_servo::moveit_servo_lib_cpp
  ${cpp_typesupport_target}  # 链接生成的服务接口
)
ament_target_dependencies(demo_franka_vr_vel ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(moveit_cartesian_offset src/moveit_cartesian_offset.cpp)
ament_target_dependencies(moveit_cartesian_offset
  geometry_msgs
  moveit_ros_planning_interface
  rclcpp
)

add_executable(rviz_pose_goal_executor src/rviz_pose_goal_executor.cpp)
ament_target_dependencies(rviz_pose_goal_executor
  moveit_ros_planning_interface
  rclcpp
  visualization_msgs
)

add_executable(pi05_direct_velocity_executor src/pi05_direct_velocity_executor.cpp)
ament_target_dependencies(pi05_direct_velocity_executor
  control_msgs
  diagnostic_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

add_library(pi05_watchdog_velocity_controller SHARED src/pi05_watchdog_velocity_controller.cpp)
ament_target_dependencies(pi05_watchdog_velocity_controller
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  realtime_tools
  std_msgs
)
pluginlib_export_plugin_description_file(controller_interface pi05_direct_controller_plugins.xml)

add_executable(pi05_collision_monitor src/pi05_collision_monitor.cpp)
ament_target_dependencies(pi05_collision_monitor
  moveit_core
  moveit_ros_planning
  rclcpp
  std_msgs
)

install(TARGETS
demo_franka_vr_vel
moveit_cartesian_offset
rviz_pose_goal_executor
pi05_direct_velocity_executor
pi05_collision_monitor
    DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS pi05_watchdog_velocity_controller DESTINATION lib)
install(DIRECTORY 
    config/
    DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
    models/
    DESTINATION share/${PROJECT_NAME}/models
)

install(DIRECTORY
    launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

install(PROGRAMS
    scripts/gazebo_gripper_marker.py
  scripts/keyboard_manual_control.py
  scripts/lerobot_episode_recorder.py
  scripts/merge_lerobot_datasets.py
  scripts/random_low_speed_trajectory.py
  scripts/vla_policy_bridge.py
  scripts/franka_vr_vla_bridge.py
  scripts/calibrate_camera_extrinsics.py
  scripts/camera_info_frame_relay.py
  scripts/pi05_direct_policy_adapter.py
  scripts/pi05_direct_velocity_safety.py
    DESTINATION lib/${PROJECT_NAME}
)
# 声明这是一个ROS 2包
ament_package()
