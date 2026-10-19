from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    recorder = Node(package="franka_vr", executable="lerobot_episode_recorder.py", output="screen",
                    arguments=["--duration", LaunchConfiguration("duration"), "--dataset-name", LaunchConfiguration("dataset_name"),
                               "--task", LaunchConfiguration("task"), "--fps", LaunchConfiguration("fps"),
                               "--action-fps", LaunchConfiguration("action_fps")])
    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("franka_vr"), "launch", "vla_cameras.launch.py"]
            )
        )
    )
    return LaunchDescription([
        DeclareLaunchArgument("duration", default_value="30"),
        DeclareLaunchArgument("fps", default_value="30"),
        DeclareLaunchArgument("action_fps", default_value="80"),
        DeclareLaunchArgument("dataset_name", default_value=""),
        DeclareLaunchArgument(
            "task",
            default_value="Move the FR3 end effector along a small smooth random trajectory",
        ),
        cameras,
        recorder,
        RegisterEventHandler(OnProcessExit(
            target_action=recorder,
            on_exit=[EmitEvent(event=Shutdown(reason="episode recorder finished"))],
        )),
    ])
