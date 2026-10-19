"""Real/fake FR3 stack with standard MoveIt Servo JointJog input.

This launch deliberately does not start demo_franka_vr_vel or expose the old
Cartesian execution service. MoveIt Servo is the sole command publisher to
the arm trajectory controller.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import xacro


def setup_nodes(
    context: LaunchContext,
    robot_ip,
    arm_id,
    use_fake_hardware,
    fake_sensor_commands,
    load_gripper,
    use_rviz,
    arm_prefix,
):
    robot_ip_value = context.perform_substitution(robot_ip)
    arm_id_value = context.perform_substitution(arm_id)
    fake_hardware_value = context.perform_substitution(use_fake_hardware)
    fake_sensor_value = context.perform_substitution(fake_sensor_commands)
    load_gripper_value = context.perform_substitution(load_gripper)
    arm_prefix_value = context.perform_substitution(arm_prefix)

    xacro_path = os.path.join(
        get_package_share_directory("franka_description"), "robots", arm_id_value, f"{arm_id_value}.urdf.xacro"
    )
    robot_description_xml = xacro.process_file(
        xacro_path,
        mappings={
            "ros2_control": "true",
            "arm_id": arm_id_value,
            "robot_ip": robot_ip_value,
            "hand": load_gripper_value,
            "use_fake_hardware": fake_hardware_value,
            "fake_sensor_commands": fake_sensor_value,
            "arm_prefix": arm_prefix_value,
        },
    ).toprettyxml(indent="  ")
    robot_description = {"robot_description": robot_description_xml}

    moveit_config = (
        MoveItConfigsBuilder("franka_fr3", package_name="franka_fr3_moveit_config")
        .robot_description(
            file_path=xacro_path,
            mappings={
                "robot_ip": robot_ip_value,
                "use_fake_hardware": fake_hardware_value,
                "fake_sensor_commands": fake_sensor_value,
                "hand": load_gripper_value,
            },
        )
        .robot_description_semantic(
            file_path=os.path.join(
                get_package_share_directory("franka_description"), "robots", "fr3", "fr3.srdf.xacro"
            ),
            mappings={"hand": load_gripper_value, "arm_prefix": arm_prefix_value},
        )
        .robot_description_kinematics(
            file_path=os.path.join(get_package_share_directory("franka_vr"), "config", "kinematics.yaml")
        )
        .joint_limits(
            file_path=os.path.join(get_package_share_directory("franka_vr"), "config", "joint_limits.yaml")
        )
        .trajectory_execution(
            file_path=os.path.join(get_package_share_directory("franka_vr"), "config", "fr3_controllers.yaml")
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    servo_params = {
        "moveit_servo": ParameterBuilder("franka_vr").yaml("config/fr3_pi05_servo.yaml").to_dict()
    }
    controller_config = PathJoinSubstitution([FindPackageShare("franka_vr"), "config", "fr3_ros_controllers.yaml"])
    state_publisher_config = PathJoinSubstitution(
        [FindPackageShare("franka_vr"), "config", "pi05_state_publisher.yaml"]
    )
    rviz_config = os.path.join(get_package_share_directory("franka_description"), "rviz", "visualize_franka.rviz")

    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_robot_base_broadcaster",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                "--frame-id", "world", "--child-frame-id", "base",
            ],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            parameters=[
                robot_description,
                {
                    "source_list": ["franka/joint_states", "/fr3_gripper/franka_gripper/joint_states"],
                    "rate": 100,
                },
            ],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            prefix=["taskset -c 4"],
            parameters=[
                controller_config,
                state_publisher_config,
                robot_description,
                {"arm_id": arm_id_value, "load_gripper": load_gripper_value},
            ],
            remappings=[("joint_states", "franka/joint_states")],
            output="screen",
            on_exit=Shutdown(),
        ),
        Node(
            package="moveit_servo",
            executable="servo_node",
            name="servo_node",
            parameters=[
                servo_params,
                {"update_period": 0.01, "planning_group_name": "fr3_arm"},
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "fr3_arm_controller"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["franka_robot_state_broadcaster"],
            parameters=[{"arm_id": arm_id_value}],
            condition=UnlessCondition(use_fake_hardware),
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare("franka_gripper"), "launch", "gripper.launch.py"])
            ]),
            launch_arguments={
                "robot_ip": robot_ip_value,
                "use_fake_hardware": fake_hardware_value,
                "robot_type": arm_id_value,
                "namespace": "fr3_gripper",
            }.items(),
            condition=IfCondition(load_gripper),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            parameters=[moveit_config.robot_description, moveit_config.robot_description_semantic],
            condition=IfCondition(use_rviz),
            output="log",
        ),
    ]
    return nodes


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    arm_id = LaunchConfiguration("arm_id")
    use_rviz = LaunchConfiguration("use_rviz")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    load_gripper = LaunchConfiguration("load_gripper")
    arm_prefix = LaunchConfiguration("arm_prefix")
    return LaunchDescription([
        DeclareLaunchArgument("robot_ip", default_value="172.16.0.2"),
        DeclareLaunchArgument("arm_id", default_value="fr3"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("fake_sensor_commands", default_value="false"),
        DeclareLaunchArgument("load_gripper", default_value="true"),
        DeclareLaunchArgument("arm_prefix", default_value=""),
        OpaqueFunction(
            function=setup_nodes,
            args=[robot_ip, arm_id, use_fake_hardware, fake_sensor_commands, load_gripper, use_rviz, arm_prefix],
        ),
    ])
