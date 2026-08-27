#!/usr/bin/env python3
"""Smooth terminal teleoperation for the Gazebo FR3 end effector.

Keys update a Cartesian target that MoveIt Servo continuously follows.  This
avoids the plan-and-execute pause that made the original keyboard control feel
jerky, while Servo keeps the normal singularity and collision safeguards.
"""

import math
import os
import select
import sys
import termios
import time
import tty

from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose, Quaternion
from rclpy.action import ActionClient
from rclpy.duration import Duration
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from franka_vr.srv import SetTargetPose


CONTROL_FRAME = "world"
TIP_FRAME = "fr3_link8"
# Small target increments let normal terminal key-repeat produce an even,
# continuous motion.  Servo performs the interpolation between targets.
LINEAR_STEP_METERS = 0.008
ANGULAR_STEP_RADIANS = math.radians(3.0)
GRIPPER_OPEN_POSITION = 0.04
GRIPPER_MAX_EFFORT = 100.0
STARTUP_TIMEOUT_SECONDS = 30.0


def multiply_quaternions(left: Quaternion, right: Quaternion) -> Quaternion:
    """Return the normalized composition left * right."""
    result = Quaternion()
    result.w = left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z
    result.x = left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y
    result.y = left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x
    result.z = left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w
    norm = math.sqrt(result.x**2 + result.y**2 + result.z**2 + result.w**2)
    result.x /= norm
    result.y /= norm
    result.z /= norm
    result.w /= norm
    return result


def axis_angle(axis: str, angle: float) -> Quaternion:
    """Create a world-frame rotation quaternion around one Cartesian axis."""
    half_angle = angle / 2.0
    sine = math.sin(half_angle)
    rotation = Quaternion()
    rotation.w = math.cos(half_angle)
    setattr(rotation, axis, sine)
    return rotation


class KeyboardManualControl(Node):
    def __init__(self):
        super().__init__("keyboard_manual_control")
        self._target_client = self.create_client(SetTargetPose, "/set_target_pose")
        self._gripper_client = ActionClient(self, GripperCommand, "/fr3_gripper/gripper_cmd")
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._gripper_open = False
        self._gripper_goal_in_flight = False
        self._target_pose = None
        # Do not let terminal auto-repeat build an unbounded ROS service
        # queue.  One request may be in flight; newer key presses are merged
        # into the latest target and sent as soon as that request completes.
        self._target_request_in_flight = False
        self._target_dirty = False
        self.create_subscription(JointState, "/joint_states", self._joint_state_callback, 10)

    def _joint_state_callback(self, message: JointState):
        try:
            index = message.name.index("fr3_finger_joint1")
        except ValueError:
            return
        if index < len(message.position):
            self._gripper_open = message.position[index] > GRIPPER_OPEN_POSITION / 2.0

    def _current_pose(self):
        transform = self._tf_buffer.lookup_transform(
            CONTROL_FRAME, TIP_FRAME, Time(), timeout=Duration(seconds=1.0)
        )
        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        return pose

    def wait_until_ready(self) -> bool:
        """Wait for the TF cache and MoveIt Servo before accepting key presses.

        A newly started command-line node has an empty TF cache.  Accepting a
        key immediately made the first command disappear with a misleading
        "world does not exist" warning, which looked like a broken controller.
        """
        deadline = time.monotonic() + STARTUP_TIMEOUT_SECONDS
        print("Waiting for Gazebo pose and MoveIt Servo...", flush=True)
        while rclpy.ok() and time.monotonic() < deadline:
            # TransformListener receives /tf_static only while the node spins.
            rclpy.spin_once(self, timeout_sec=0.1)
            has_pose = self._tf_buffer.can_transform(CONTROL_FRAME, TIP_FRAME, Time())
            if has_pose and self._target_client.service_is_ready():
                self._target_pose = self._current_pose()
                print("Manual control is ready.", flush=True)
                return True

        if not self._tf_buffer.can_transform(CONTROL_FRAME, TIP_FRAME, Time()):
            self.get_logger().error(
                f"Timed out waiting for TF transform {CONTROL_FRAME} -> {TIP_FRAME}. "
                "Wait for Gazebo to finish starting, then run this command again."
            )
        else:
            self.get_logger().error(
                "Timed out waiting for MoveIt Servo. Start ./run_gazebo.sh "
                "or ./run_robot.sh, then wait for the Servo stack to finish starting."
            )
        return False

    def _send_target_increment(self, linear_axis=None, linear_sign=0.0, angular_axis=None, angular_sign=0.0):
        if self._target_pose is None:
            self.get_logger().warning("Manual control is not ready yet")
            return
        if linear_axis:
            setattr(
                self._target_pose.position,
                linear_axis,
                getattr(self._target_pose.position, linear_axis) + linear_sign * LINEAR_STEP_METERS,
            )
        if angular_axis:
            self._target_pose.orientation = multiply_quaternions(
                axis_angle(angular_axis, angular_sign * ANGULAR_STEP_RADIANS), self._target_pose.orientation
            )

        self._target_dirty = True
        self._dispatch_latest_target()
        self.get_logger().info(
            "Updated Servo target: "
            f"[{self._target_pose.position.x:.3f}, {self._target_pose.position.y:.3f}, "
            f"{self._target_pose.position.z:.3f}]"
        )

    def _dispatch_latest_target(self):
        """Send only the newest target; terminal key repeat never backlogs ROS."""
        if (not self._target_dirty or self._target_request_in_flight
                or not self._target_client.service_is_ready()):
            return
        request = SetTargetPose.Request()
        request.target_pose.header.stamp = self.get_clock().now().to_msg()
        request.target_pose.header.frame_id = CONTROL_FRAME
        request.target_pose.pose = self._target_pose
        request.stop = False
        self._target_dirty = False
        self._target_request_in_flight = True
        future = self._target_client.call_async(request)
        future.add_done_callback(self._target_request_done)

    def _target_request_done(self, future):
        self._target_request_in_flight = False
        try:
            response = future.result()
            if not response.success:
                self.get_logger().warning(f"Servo target rejected: {response.message}")
        except Exception as error:  # Service disappearance must not strand input.
            self.get_logger().warning(f"Servo target request failed: {error}")
        self._dispatch_latest_target()

    def _toggle_gripper(self):
        if self._gripper_goal_in_flight:
            return
        if not self._gripper_client.server_is_ready():
            self.get_logger().error("Gripper action server is not ready")
            return
        goal = GripperCommand.Goal()
        goal.command.position = 0.0 if self._gripper_open else GRIPPER_OPEN_POSITION
        goal.command.max_effort = GRIPPER_MAX_EFFORT
        self._gripper_goal_in_flight = True
        goal_future = self._gripper_client.send_goal_async(goal)
        goal_future.add_done_callback(self._gripper_goal_response)
        action = "close" if self._gripper_open else "open"
        self.get_logger().info(f"Requested gripper {action}")

    def _gripper_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as error:
            self._gripper_goal_in_flight = False
            self.get_logger().warning(f"Gripper goal failed: {error}")
            return
        if not goal_handle.accepted:
            self._gripper_goal_in_flight = False
            self.get_logger().warning("Gripper goal was rejected")
            return
        goal_handle.get_result_async().add_done_callback(self._gripper_goal_result)

    def _gripper_goal_result(self, future):
        self._gripper_goal_in_flight = False
        try:
            result = future.result().result
            self._gripper_open = result.position > GRIPPER_OPEN_POSITION / 2.0
        except Exception as error:
            self.get_logger().warning(f"Gripper result unavailable: {error}")

    def handle_key(self, key: str) -> bool:
        # All translations and rotations are expressed in the world frame.
        linear_keys = {
            "w": ("x", 1.0), "s": ("x", -1.0),
            "a": ("y", 1.0), "d": ("y", -1.0),
            "r": ("z", 1.0), "f": ("z", -1.0),
        }
        angular_keys = {
            "up": ("x", 1.0), "down": ("x", -1.0),
            "right": ("y", 1.0), "left": ("y", -1.0),
            "z": ("z", 1.0), "x": ("z", -1.0),
        }
        if key in linear_keys:
            axis, direction = linear_keys[key]
            self._send_target_increment(linear_axis=axis, linear_sign=direction)
        elif key in angular_keys:
            axis, direction = angular_keys[key]
            self._send_target_increment(angular_axis=axis, angular_sign=direction)
        elif key == " ":
            self._toggle_gripper()
        elif key in ("q", "\x03"):
            return False
        elif key == "?":
            print_controls()
        return True


def print_controls():
    print(
        "\nSmooth manual control (world frame; each key press is 8 mm or 3 degrees)\n"
        "  Hold a key for continuous motion; release it to stop.\n"
        "  W/S  front/back (+X/-X)\n"
        "  A/D  left/right (+Y/-Y)\n"
        "  R/F  up/down (+Z/-Z)\n"
        "  Up/Down     rotate around X (+/-)\n"
        "  Right/Left  rotate around Y (+/-)\n"
        "  Z/X         rotate around Z (+/-)\n"
        "  Space toggle gripper open/close\n"
        "  Q or Ctrl-C quit\n"
    )


def main():
    if not sys.stdin.isatty():
        raise RuntimeError("keyboard_manual_control.py must be run in an interactive terminal")

    rclpy.init()
    node = KeyboardManualControl()
    terminal_fd = sys.stdin.fileno()
    original_terminal = termios.tcgetattr(terminal_fd)
    try:
        if not node.wait_until_ready():
            return
        print_controls()
        tty.setraw(terminal_fd)
        running = True
        while rclpy.ok() and running:
            rclpy.spin_once(node, timeout_sec=0.0)
            ready, _, _ = select.select([sys.stdin], [], [], 0.05)
            if ready:
                key = os.read(terminal_fd, 1)
                # Terminal arrow keys arrive as ESC [ A/B/C/D.  Consume the
                # full sequence so its individual bytes are not mistaken for
                # regular controls.
                if key == b"\x1b":
                    sequence_ready, _, _ = select.select([sys.stdin], [], [], 0.02)
                    if sequence_ready and os.read(terminal_fd, 1) == b"[":
                        sequence_ready, _, _ = select.select([sys.stdin], [], [], 0.02)
                        if sequence_ready:
                            arrow = os.read(terminal_fd, 1)
                            key = {
                                b"A": b"up",
                                b"B": b"down",
                                b"C": b"right",
                                b"D": b"left",
                            }.get(arrow, b"")
                running = node.handle_key(key.decode(errors="ignore").lower())
    finally:
        termios.tcsetattr(terminal_fd, termios.TCSADRAIN, original_terminal)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
