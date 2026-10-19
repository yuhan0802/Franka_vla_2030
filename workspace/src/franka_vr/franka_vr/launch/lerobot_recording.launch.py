from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    recorder = Node(package="franka_vr", executable="lerobot_episode_recorder.py", output="screen",
                    arguments=["--duration", LaunchConfiguration("duration"), "--dataset-name", LaunchConfiguration("dataset_name"),
                               "--task", LaunchConfiguration("task"), "--fps", LaunchConfiguration("fps"),
                               "--action-fps", LaunchConfiguration("action_fps")])
    return LaunchDescription([
        DeclareLaunchArgument("duration", default_value="30"),
        DeclareLaunchArgument("fps", default_value="30"),
        DeclareLaunchArgument("action_fps", default_value="80"),
        DeclareLaunchArgument("dataset_name", default_value=""),
        DeclareLaunchArgument(
            "task",
            default_value="Move the FR3 end effector along a small smooth random trajectory",
        ),
        Node(package="realsense2_camera", executable="realsense2_camera_node", name="camera_front",
             parameters=[{"camera_name": "camera_front", "enable_color": True, "enable_depth": False,
                          "rgb_camera.color_profile": "640x480x30"}]),
        # /dev/videoN numbers change when a USB camera reconnects. Use the
        # persistent udev links for the two dedicated C922 webcams instead.
        Node(package="v4l2_camera", executable="v4l2_camera_node", name="left_camera", namespace="camera_left",
             parameters=[{"video_device": "/dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_43DDCA6F-video-index0",
                          "image_size": [640, 480], "time_per_frame": [1, 30]}]),
        Node(package="v4l2_camera", executable="v4l2_camera_node", name="right_camera", namespace="camera_right",
             parameters=[{"video_device": "/dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_CD7DCA6F-video-index0",
                          "image_size": [640, 480], "time_per_frame": [1, 30]}]),
        recorder,
        RegisterEventHandler(OnProcessExit(
            target_action=recorder,
            on_exit=[EmitEvent(event=Shutdown(reason="episode recorder finished"))],
        )),
    ])
