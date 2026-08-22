#!/usr/bin/env python3
"""RViz menu control for the Gazebo Franka Hand gripper."""

from control_msgs.action import GripperCommand
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker


class GazeboGripperMarker(Node):
    def __init__(self):
        super().__init__("gazebo_gripper_marker")
        self._client = ActionClient(self, GripperCommand, "/fr3_gripper/gripper_cmd")
        self._server = InteractiveMarkerServer(self, "franka_gripper_control")
        self._menu = MenuHandler()
        self._open = self._menu.insert("Open gripper", callback=self._open_gripper)
        self._close = self._menu.insert("Close gripper", callback=self._close_gripper)
        self._add_marker()

    def _add_marker(self):
        marker = InteractiveMarker()
        marker.header.frame_id = "world"
        marker.name = "gripper_control"
        marker.description = "Gripper: right-click for open / close"
        marker.pose.position.x = 0.20
        marker.pose.position.y = -0.45
        marker.pose.position.z = 0.12
        marker.scale = 0.22

        control = InteractiveMarkerControl()
        control.name = "gripper_menu"
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True
        cube = Marker()
        cube.type = Marker.CUBE
        cube.scale.x = 0.14
        cube.scale.y = 0.08
        cube.scale.z = 0.08
        cube.color.r = 0.10
        cube.color.g = 0.65
        cube.color.b = 0.95
        cube.color.a = 0.9
        control.markers.append(cube)
        marker.controls.append(control)

        self._server.insert(marker, feedback_callback=self._process_feedback)
        self._menu.apply(self._server, marker.name)
        self._server.applyChanges()

    def _process_feedback(self, feedback):
        self._menu.processFeedback(feedback)

    def _open_gripper(self, _feedback):
        self._send_goal(0.035, "open")

    def _close_gripper(self, _feedback):
        self._send_goal(0.0, "close")

    def _send_goal(self, position, label):
        if not self._client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Gripper action server is not ready")
            return
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 20.0
        self._client.send_goal_async(goal)
        self.get_logger().info(f"Requested gripper {label}")


def main():
    rclpy.init()
    node = GazeboGripperMarker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
