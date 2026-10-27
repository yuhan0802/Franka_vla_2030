"""Publish the three calibrated RGB streams required by the FrankaVR fine-tune.

The RealSense D435i is ``front`` and the two C922 webcams are ``left`` and
``right``. This launch intentionally does not record data or control the robot.
"""

import os
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


def raw_camera_nodes():
    """Legacy unrectified exterior streams; restore autofocus after calibrated runs."""
    exterior_camera = os.environ.get("VLA_EXTERIOR_CAMERA", "all")
    if exterior_camera not in ("all", "left", "right"):
        raise ValueError("VLA_EXTERIOR_CAMERA must be all, left or right")
    nodes = [Node(
        package="realsense2_camera", executable="realsense2_camera_node",
        name="camera_front",
        parameters=[{"camera_name": "camera_front", "enable_color": True,
                     "enable_depth": False, "rgb_camera.color_profile": "640x480x30"}],
        output="screen",
    )]
    for side, serial in (("right", "CD7DCA6F"), ("left", "43DDCA6F")):
        if exterior_camera != "all" and side != exterior_camera:
            continue
        nodes.append(Node(
            package="v4l2_camera", executable="v4l2_camera_node",
            name=f"{side}_camera", namespace=f"camera_{side}",
            parameters=[{
                "video_device": f"/dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_{serial}-video-index0",
                "pixel_format": "YUYV", "output_encoding": "rgb8",
                "image_size": [640, 480], "time_per_frame": [1, 15],
                "focus_automatic_continuous": True,
            }], output="screen",
        ))
    return nodes


def generate_launch_description():
    mode = os.environ.get("VLA_CAMERA_MODE", "calibrated")
    if mode == "raw":
        return LaunchDescription(raw_camera_nodes())
    if mode != "calibrated":
        raise ValueError("VLA_CAMERA_MODE must be raw or calibrated")
    return calibrated_launch_description()


def calibrated_launch_description():
    exterior_camera = os.environ.get("VLA_EXTERIOR_CAMERA", "all")
    if exterior_camera not in ("all", "left", "right"):
        raise ValueError("VLA_EXTERIOR_CAMERA must be all, left or right")
    calibration_dir = Path(get_package_share_directory("franka_vr")) / "config" / "cameras"
    left_calibration_url = f"file://{calibration_dir / 'left_c922.yaml'}"
    right_calibration_url = f"file://{calibration_dir / 'right_c922.yaml'}"
    with (calibration_dir / "extrinsics.yaml").open(encoding="utf-8") as stream:
        extrinsics = yaml.safe_load(stream)["transforms"]

    nodes = [
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
        )
    ]
    camera_specs = {
        "right": ("CD7DCA6F", right_calibration_url),
        "left": ("43DDCA6F", left_calibration_url),
    }
    selected = ("right", "left") if exterior_camera == "all" else (exterior_camera,)
    for side in selected:
        serial, calibration_url = camera_specs[side]
        nodes.extend([Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name=f"{side}_camera",
            namespace=f"camera_{side}",
            parameters=[{
                "video_device": f"/dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_{serial}-video-index0",
                "pixel_format": "YUYV",
                "output_encoding": "rgb8",
                "image_size": [640, 480],
                "time_per_frame": [1, 15],
                "camera_info_url": calibration_url,
                "camera_frame_id": f"camera_{side}_optical_frame",
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
            namespace=f"camera_{side}",
            parameters=[{"frame_id": f"camera_{side}_optical_frame"}],
            output="screen",
        ),
        Node(
            package="image_proc",
            executable="rectify_node",
            name="rectify",
            namespace=f"camera_{side}",
            remappings=[("image", "image_raw")],
            output="screen",
        )])
    nodes.extend([
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
    return LaunchDescription(nodes)
