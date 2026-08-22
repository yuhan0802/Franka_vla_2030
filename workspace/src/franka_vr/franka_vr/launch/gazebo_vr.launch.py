"""Run the Franka VR Servo stack against the Gazebo Sim FR3 model."""

import os
import re

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder


def build_moveit_config(load_gripper: str):
    """Build the MoveIt parameters for the exact URDF spawned in Gazebo."""
    description_share = get_package_share_directory("franka_description")
    return (
        MoveItConfigsBuilder("franka_fr3", package_name="franka_fr3_moveit_config")
        .robot_description(
            file_path=os.path.join(description_share, "robots", "fr3", "fr3.urdf.xacro"),
            mappings={
                "robot_ip": "",
                "use_fake_hardware": "false",
                "fake_sensor_commands": "false",
                "ros2_control": "true",
                "gazebo": "true",
                "gazebo_effort": "false",
                "hand": load_gripper,
            },
        )
        .robot_description_semantic(
            file_path=os.path.join(description_share, "robots", "fr3", "fr3.srdf.xacro"),
            mappings={"hand": load_gripper, "arm_prefix": ""},
        )
        .joint_limits(file_path=os.path.join(get_package_share_directory("franka_vr"), "config", "joint_limits.yaml"))
        .robot_description_kinematics(
            file_path=os.path.join(get_package_share_directory("franka_vr"), "config", "kinematics.yaml")
        )
        .trajectory_execution(
            file_path=os.path.join(get_package_share_directory("franka_vr"), "config", "fr3_controllers.yaml")
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )


def launch_setup(context: LaunchContext):
    load_gripper = LaunchConfiguration("load_gripper").perform(context)
    use_rviz = LaunchConfiguration("use_rviz")
    enable_vr = LaunchConfiguration("enable_vr")
    description_share = get_package_share_directory("franka_description")
    xacro_path = os.path.join(description_share, "robots", "fr3", "fr3.urdf.xacro")
    robot_description_xml = xacro.process_file(
        xacro_path,
        mappings={
            "robot_ip": "",
            "use_fake_hardware": "false",
            "fake_sensor_commands": "false",
            "ros2_control": "true",
            "gazebo": "true",
            "gazebo_effort": "false",
            "hand": load_gripper,
        },
    ).toprettyxml(indent="  ")
    moveit_config = build_moveit_config(load_gripper)
    semantic_xacro_path = os.path.join(description_share, "robots", "fr3", "fr3.srdf.xacro")
    semantic_xml = xacro.process_file(
        semantic_xacro_path, mappings={"hand": load_gripper, "arm_prefix": ""}
    ).toprettyxml(indent="  ")
    # In Gazebo the URDF root link is `world`, whereas the FR3 SRDF's virtual
    # joint is for the normal hardware URDF root (`base`).  Remove that
    # hardware-only virtual joint so MoveIt uses the Gazebo world root.
    semantic_xml = re.sub(r"\s*<virtual_joint\b[^>]*/>\s*", "\n", semantic_xml)
    moveit_params = moveit_config.to_dict()
    moveit_params["robot_description"] = robot_description_xml
    moveit_params["robot_description_semantic"] = semantic_xml
    with open(
        os.path.join(
            get_package_share_directory("franka_fr3_moveit_config"), "config", "ompl_planning.yaml"
        ),
        encoding="utf-8",
    ) as ompl_file:
        ompl_planning_config = {
            "default_planning_pipeline": "ompl",
            "planning_pipelines": ["ompl"],
            "ompl": {
                "planning_plugins": ["ompl_interface/OMPLPlanner"],
                "request_adapters": [
                    "default_planning_request_adapters/ResolveConstraintFrames",
                    "default_planning_request_adapters/ValidateWorkspaceBounds",
                    "default_planning_request_adapters/CheckStartStateBounds",
                    "default_planning_request_adapters/CheckStartStateCollision",
                ],
                "response_adapters": [
                    "default_planning_response_adapters/AddTimeOptimalParameterization",
                    "default_planning_response_adapters/ValidateSolution",
                ],
                "start_state_max_bounds_error": 0.1,
            },
        }
        ompl_planning_config["ompl"].update(yaml.safe_load(ompl_file))
    with open(
        os.path.join(get_package_share_directory("franka_vr"), "config", "fr3_controllers.yaml"),
        encoding="utf-8",
    ) as controllers_file:
        moveit_controllers = {
            "moveit_controller_manager": (
                "moveit_simple_controller_manager/MoveItSimpleControllerManager"
            ),
            "moveit_simple_controller_manager": yaml.safe_load(controllers_file),
            "trajectory_execution.allowed_execution_duration_scaling": 1.2,
            "trajectory_execution.allowed_goal_duration_margin": 0.5,
            "trajectory_execution.allowed_start_tolerance": 0.01,
        }
    servo_params = {
        "moveit_servo": ParameterBuilder("franka_vr").yaml("config/fr3_real_config.yaml").to_dict()
    }
    # Gazebo's joint_state_broadcaster publishes /joint_states, unlike the
    # real robot launch which remaps it to /franka/joint_states.
    servo_params["moveit_servo"]["joint_topic"] = "/joint_states"

    os.environ["GZ_SIM_RESOURCE_PATH"] = os.path.dirname(description_share)
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": "empty.sdf -r"}.items(),
    )
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "/robot_description"],
        output="screen",
    )
    block = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-file",
            os.path.join(get_package_share_directory("franka_vr"), "models", "target_block.sdf"),
            "-name",
            "target_block",
            "-x",
            "0.45",
            "-y",
            "0.0",
            "-z",
            "0.04",
        ],
        output="screen",
    )
    joint_state_broadcaster = ExecuteProcess(
        # `spawner` waits for Gazebo's controller-manager service instead of
        # racing it just after the robot entity has appeared in the world.
        cmd=[
            "ros2", "run", "controller_manager", "spawner", "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
        output="screen",
    )
    arm_controller = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner", "fr3_arm_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
        output="screen",
    )
    gripper_controller = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner", "fr3_gripper",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
        output="screen",
    )
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[
            moveit_params,
            ompl_planning_config,
            moveit_controllers,
            {
                # Gazebo already owns the controller manager.  MoveIt must
                # send goals to the active trajectory controller, not try to
                # start another controller manager.
                "moveit_manage_controllers": False,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
                "use_sim_time": True,
            },
        ],
        output="screen",
    )
    rviz_config = os.path.join(
        get_package_share_directory("franka_vr"), "config", "gazebo_moveit.rviz"
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_params,
            ompl_planning_config,
            moveit_controllers,
            {"use_sim_time": True},
        ],
        condition=IfCondition(use_rviz),
        output="log",
    )

    return [
        gazebo,
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description_xml, "use_sim_time": True}],
            output="screen",
        ),
        spawn,
        block,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[joint_state_broadcaster])),
        RegisterEventHandler(
            OnProcessExit(target_action=joint_state_broadcaster, on_exit=[arm_controller])
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=arm_controller, on_exit=[gripper_controller])
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=gripper_controller, on_exit=[move_group])
        ),
        # RViz requests /get_planning_scene as it starts.  Starting it with
        # Gazebo used to make that first request race move_group, leaving the
        # MotionPlanning display permanently in "Requesting initial scene
        # failed" state and preventing reliable end-effector interaction.
        RegisterEventHandler(
            OnProcessStart(
                target_action=move_group,
                on_start=[TimerAction(period=3.0, actions=[rviz])],
            )
        ),
        Node(
            package="franka_vr",
            executable="gazebo_gripper_marker.py",
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
        Node(
            package="franka_vr",
            executable="demo_franka_vr_vel",
            # joint_state_broadcaster stamps states with Gazebo's /clock.  Servo
            # must use the same clock or it treats every simulated state as stale.
            parameters=[servo_params, moveit_params, {"use_sim_time": True}],
            condition=IfCondition(enable_vr),
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("load_gripper", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("enable_vr", default_value="false"),
            OpaqueFunction(function=launch_setup),
        ]
    )
