"""Publish the three RGB streams required by the FrankaVR fine-tune.

The RealSense D435i is ``front`` and the two C922 webcams are ``left`` and
``right``. This launch intentionally does not record data or control the robot.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            name="camera_front",
            parameters=[{
                "camera_name": "camera_front",
                "enable_color": True,
                "enable_depth": False,
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
                "output_encoding": "rgb8",
                "image_size": [640, 480],
                "time_per_frame": [1, 15],
            }],
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
                "output_encoding": "rgb8",
                "image_size": [640, 480],
                "time_per_frame": [1, 15],
            }],
            output="screen",
        ),
    ])
