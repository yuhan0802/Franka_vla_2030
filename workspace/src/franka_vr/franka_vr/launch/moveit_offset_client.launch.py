import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_client(context):
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    dx = LaunchConfiguration("dx").perform(context)
    dy = LaunchConfiguration("dy").perform(context)
    dz = LaunchConfiguration("dz").perform(context)
    execute = LaunchConfiguration("execute").perform(context)
    description_share = get_package_share_directory("franka_description")
    config = (
        MoveItConfigsBuilder("franka_fr3", package_name="franka_fr3_moveit_config")
        .robot_description(
            file_path=os.path.join(description_share, "robots", "fr3", "fr3.urdf.xacro"),
            mappings={
                "hand": "false",
                "robot_ip": robot_ip,
                "use_fake_hardware": "false",
                "fake_sensor_commands": "false",
                "ros2_control": "true",
            },
        )
        .robot_description_semantic(
            file_path=os.path.join(description_share, "robots", "fr3", "fr3.srdf.xacro"),
            mappings={"hand": "false", "arm_prefix": ""},
        )
        .joint_limits(
            file_path=os.path.join(description_share, "robots", "fr3", "joint_limits.yaml")
        )
        .robot_description_kinematics(
            file_path=os.path.join(
                get_package_share_directory("franka_vr"), "config", "kinematics.yaml"
            )
        )
        .trajectory_execution(
            file_path=os.path.join(
                get_package_share_directory("franka_fr3_moveit_config"),
                "config",
                "fr3_controllers.yaml",
            )
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    return [
        Node(
            package="franka_vr",
            executable="moveit_cartesian_offset",
            output="screen",
            arguments=[dx, dy, dz, f"execute={execute}"],
            parameters=[
                config.robot_description,
                config.robot_description_semantic,
                config.robot_description_kinematics,
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip", default_value="172.16.0.2"),
            DeclareLaunchArgument("dx", default_value="0"),
            DeclareLaunchArgument("dy", default_value="0"),
            DeclareLaunchArgument("dz", default_value="0"),
            DeclareLaunchArgument("execute", default_value="false"),
            OpaqueFunction(function=launch_client),
        ]
    )
