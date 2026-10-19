"""Publish the three calibrated RGB streams required by the FrankaVR fine-tune.

The RealSense D435i is ``front`` and the two C922 webcams are ``left`` and
``right``. This launch intentionally does not record data or control the robot.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml


def static_transform_node(name, transform):
    translation = transform["translation_xyz_m"]
    rotation = transform["rotation_xyzw"]
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        arguments=[
            "--x", str(translation[0]),
            "--y", str(translation[1]),
            "--z", str(translation[2]),
            "--qx", str(rotation[0]),
            "--qy", str(rotation[1]),
            "--qz", str(rotation[2]),
            "--qw", str(rotation[3]),
            "--frame-id", transform["parent_frame"],
            "--child-frame-id", transform["child_frame"],
        ],
        output="screen",
    )


def generate_launch_description():
    calibration_dir = Path(get_package_share_directory("franka_vr")) / "config" / "cameras"
    left_calibration_url = f"file://{calibration_dir / 'left_c922.yaml'}"
    right_calibration_url = f"file://{calibration_dir / 'right_c922.yaml'}"
    with (calibration_dir / "extrinsics.yaml").open(encoding="utf-8") as stream:
        extrinsics = yaml.safe_load(stream)["transforms"]

    return LaunchDescription([
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            name="camera_front",
            parameters=[{
                "camera_name": "camera_front",
                "enable_color": True,
                "enable_depth": False,
                "enable_infra1": False,
                "enable_infra2": False,
                "enable_gyro": False,
                "enable_accel": False,
                "rgb_camera.color_profile": "640x480x30",
            }],
            output="screen",
        ),
        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name="right_camera",
            namespace="camera_right",
            parameters=[{
                "video_device": "/dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_CD7DCA6F-video-index0",
                # Two uncompressed 640x480@30 streams plus the D435i exhaust
                # this host's shared USB controller and make the second C922
                # fail with EPROTO.  This v4l2_camera build cannot decode the
                # C922's MJPEG stream, so retain YUYV and halve the frame rate.
                "pixel_format": "YUYV",
                "output_encoding": "yuv422_yuy2",
                "image_size": [640, 480],
                "time_per_frame": [1, 15],
                "camera_info_url": right_calibration_url,
                "camera_frame_id": "camera_right_optical_frame",
                # Lock the focus at the value read from this C922 after its
                # autofocus had settled.  Re-calibrate if this value changes.
                "focus_automatic_continuous": False,
                "focus_absolute": 0,
            }],
            remappings=[("camera_info", "camera_info_driver")],
            output="screen",
        ),
        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name="left_camera",
            namespace="camera_left",
            parameters=[{
                "video_device": "/dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_43DDCA6F-video-index0",
                "pixel_format": "YUYV",
                "output_encoding": "yuv422_yuy2",
                "image_size": [640, 480],
                "time_per_frame": [1, 15],
                "camera_info_url": left_calibration_url,
                "camera_frame_id": "camera_left_optical_frame",
                # Lock the focus at the value read from this C922 after its
                # autofocus had settled.  Re-calibrate if this value changes.
                "focus_automatic_continuous": False,
                "focus_absolute": 0,
            }],
            remappings=[("camera_info", "camera_info_driver")],
            output="screen",
        ),
        Node(
            package="franka_vr",
            executable="camera_info_frame_relay.py",
            name="camera_info_frame_relay",
            namespace="camera_right",
            parameters=[{"frame_id": "camera_right_optical_frame"}],
            output="screen",
        ),
        Node(
            package="franka_vr",
            executable="camera_info_frame_relay.py",
            name="camera_info_frame_relay",
            namespace="camera_left",
            parameters=[{"frame_id": "camera_left_optical_frame"}],
            output="screen",
        ),
        Node(
            package="image_proc",
            executable="rectify_node",
            name="rectify",
            namespace="camera_right",
            remappings=[("image", "image_raw")],
            output="screen",
        ),
        Node(
            package="image_proc",
            executable="rectify_node",
            name="rectify",
            namespace="camera_left",
            remappings=[("image", "image_raw")],
            output="screen",
        ),
        static_transform_node(
            "hand_to_camera_front_broadcaster", extrinsics["hand_to_front_link"]
        ),
        static_transform_node(
            "world_to_camera_left_broadcaster", extrinsics["world_to_left"]
        ),
        static_transform_node(
            "world_to_camera_right_broadcaster", extrinsics["world_to_right"]
        ),
    ])
