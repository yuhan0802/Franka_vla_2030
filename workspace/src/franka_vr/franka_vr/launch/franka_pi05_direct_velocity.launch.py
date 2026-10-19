"""Independent FR3 direct-velocity stack. MoveIt Servo is intentionally absent."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import xacro


def setup(context: LaunchContext, robot_ip, arm_id, use_fake_hardware, fake_sensor_commands,
          load_gripper, use_rviz, arm_prefix, armed, max_velocity, max_acceleration,
          csv_log_path):
    value = context.perform_substitution
    robot_ip_v, arm_id_v = value(robot_ip), value(arm_id)
    fake_v, fake_sensor_v = value(use_fake_hardware), value(fake_sensor_commands)
    gripper_v, prefix_v = value(load_gripper), value(arm_prefix)
    xacro_path = os.path.join(get_package_share_directory("franka_description"), "robots", arm_id_v,
                              f"{arm_id_v}.urdf.xacro")
    mappings = {"ros2_control": "true", "arm_id": arm_id_v, "robot_ip": robot_ip_v,
                "hand": gripper_v, "use_fake_hardware": fake_v,
                "fake_sensor_commands": fake_sensor_v, "arm_prefix": prefix_v}
    robot_description = {"robot_description": xacro.process_file(xacro_path, mappings=mappings).toprettyxml(indent="  ")}
    moveit = (MoveItConfigsBuilder("franka_fr3", package_name="franka_fr3_moveit_config")
              .robot_description(file_path=xacro_path, mappings=mappings)
              .robot_description_semantic(
                  file_path=os.path.join(get_package_share_directory("franka_description"), "robots", "fr3", "fr3.srdf.xacro"),
                  mappings={"hand": gripper_v, "arm_prefix": prefix_v})
              .robot_description_kinematics(
                  file_path=os.path.join(get_package_share_directory("franka_vr"), "config", "kinematics.yaml"))
              .joint_limits(file_path=os.path.join(get_package_share_directory("franka_vr"), "config", "joint_limits.yaml"))
              .trajectory_execution(
                  file_path=os.path.join(get_package_share_directory("franka_vr"), "config", "fr3_controllers.yaml"))
              .planning_pipelines(pipelines=["ompl"])
              .to_moveit_configs())
    controller_config = PathJoinSubstitution([FindPackageShare("franka_vr"), "config", "fr3_pi05_direct_velocity_controllers.yaml"])
    state_config = PathJoinSubstitution([FindPackageShare("franka_vr"), "config", "pi05_state_publisher.yaml"])
    safety_config = PathJoinSubstitution([FindPackageShare("franka_vr"), "config", "pi05_direct_velocity.yaml"])
    # Loading another controller while the real arm controller owns the FCI
    # loop has caused communication_constraints_violation on this host. Spawn
    # every state broadcaster first, then activate the arm controller only
    # after that spawner has exited successfully.
    state_controllers = ["joint_state_broadcaster"]
    if fake_v.lower() != "true":
        state_controllers.append("franka_robot_state_broadcaster")
    state_spawner = Node(
        package="controller_manager", executable="spawner", arguments=state_controllers,
        parameters=[{"arm_id": arm_id_v}], output="screen")

    nodes = [
        Node(package="tf2_ros", executable="static_transform_publisher", name="world_to_robot_base_broadcaster",
             arguments=["--x", "0", "--y", "0", "--z", "0", "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                        "--frame-id", "world", "--child-frame-id", "base"], output="screen"),
        Node(package="robot_state_publisher", executable="robot_state_publisher", name="robot_state_publisher",
             parameters=[robot_description], output="screen"),
        Node(package="joint_state_publisher", executable="joint_state_publisher", name="joint_state_publisher",
             parameters=[robot_description, {"source_list": ["franka/joint_states", "/fr3_gripper/franka_gripper/joint_states"], "rate": 100}]),
        Node(package="controller_manager", executable="ros2_control_node", prefix=["taskset -c 4"],
             parameters=[controller_config, state_config, robot_description, {"arm_id": arm_id_v, "load_gripper": gripper_v}],
             remappings=[("joint_states", "franka/joint_states")], output="screen", on_exit=Shutdown()),
        state_spawner,
        Node(package="franka_vr", executable="pi05_collision_monitor", name="pi05_collision_monitor",
             parameters=[safety_config, moveit.robot_description, moveit.robot_description_semantic,
                         moveit.robot_description_kinematics], output="screen"),
        Node(package="franka_vr", executable="pi05_direct_velocity_executor", name="pi05_direct_velocity_executor",
             parameters=[safety_config, {"armed": value(armed).lower() == "true",
                                        "max_joint_velocity_rad_s": float(value(max_velocity)),
                                        "max_joint_acceleration_rad_s2": float(value(max_acceleration)),
                                        "csv_log_path": value(csv_log_path)}], output="screen",
             on_exit=Shutdown(reason="direct velocity executor exited")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare("franka_gripper"), "launch", "gripper.launch.py"])]),
            launch_arguments={"robot_ip": robot_ip_v, "use_fake_hardware": fake_v, "robot_type": arm_id_v,
                              "namespace": "fr3_gripper"}.items(), condition=IfCondition(load_gripper)),
        Node(package="rviz2", executable="rviz2", name="rviz2",
             arguments=["-d", os.path.join(get_package_share_directory("franka_description"), "rviz", "visualize_franka.rviz")],
             parameters=[moveit.robot_description, moveit.robot_description_semantic], condition=IfCondition(use_rviz), output="log"),
    ]
    if value(armed).lower() == "true":
        arm_spawner = Node(
            package="controller_manager", executable="spawner",
            arguments=["fr3_direct_velocity_controller"], output="screen")

        def start_arm_after_state_spawners(event, _context):
            if event.returncode != 0:
                return [Shutdown(reason="state controller spawner failed; arm controller will not be started")]
            return [arm_spawner]

        nodes.append(RegisterEventHandler(
            OnProcessExit(target_action=state_spawner, on_exit=start_arm_after_state_spawners)))
    return nodes


def generate_launch_description():
    names = {
        "robot_ip": "172.16.0.2", "arm_id": "fr3", "use_rviz": "false",
        "use_fake_hardware": "false", "fake_sensor_commands": "false", "load_gripper": "false",
        "arm_prefix": "", "armed": "false", "max_joint_velocity": "0.6",
        "max_joint_acceleration": "2.0", "csv_log_path": "",
    }
    configs = {name: LaunchConfiguration(name) for name in names}
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default) for name, default in names.items()] +
        [OpaqueFunction(function=setup, args=[
            configs["robot_ip"], configs["arm_id"], configs["use_fake_hardware"],
            configs["fake_sensor_commands"], configs["load_gripper"], configs["use_rviz"],
            configs["arm_prefix"], configs["armed"], configs["max_joint_velocity"],
            configs["max_joint_acceleration"], configs["csv_log_path"],
        ])]
    )
