"""Read-only joint/Servo telemetry; never publishes or calls actions."""
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import JointJog
from moveit_msgs.msg import ServoStatus


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=15)
    args = parser.parse_args()
    rclpy.init()
    node = Node('pi05_read_only_tracking_audit')
    start = time.monotonic()

    def record(kind, msg):
        row = {'t': time.monotonic() - start, 'kind': kind}
        if kind in ('joint', 'gripper'):
            row.update(names=list(msg.name), position=list(msg.position), velocity=list(msg.velocity))
        elif kind == 'jog':
            row.update(names=list(msg.joint_names), velocity=list(msg.velocities))
        elif kind == 'trajectory':
            row.update(names=list(msg.joint_names), points=[
                {'position': list(p.positions), 'velocity': list(p.velocities),
                 'time': p.time_from_start.sec + p.time_from_start.nanosec / 1e9}
                for p in msg.points])
        else:
            row.update(code=msg.code, message=msg.message)
        print(json.dumps(row), flush=True)

    for kind, cls, topic in [
        ('joint', JointState, '/franka/joint_states'),
        ('gripper', JointState, '/fr3_gripper/franka_gripper/joint_states'),
        ('trajectory', JointTrajectory, '/fr3_arm_controller/joint_trajectory'),
        ('status', ServoStatus, '/servo_node/status'),
    ]:
        node.create_subscription(cls, topic, lambda msg, kind=kind: record(kind, msg), qos_profile_sensor_data)
    try:
        while time.monotonic() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
