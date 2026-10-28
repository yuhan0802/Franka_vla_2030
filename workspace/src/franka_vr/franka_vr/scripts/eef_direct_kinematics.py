#!/usr/bin/env python3
"""FR3 kinematics and the Servo-equivalent Cartesian tracker for the direct EEF path.

ROS-independent (numpy only) so it is unit-testable off the robot.

Kinematics: franka_description/robots/fr3/kinematics.yaml (joint origins xyz/rpy,
every joint rotates about its local z) plus the fixed joint8 to fr3_link8. The
world -> base -> fr3_link0 transforms of both robot stacks are identities.

Tracking law: the same proportional pose tracker MoveIt Servo ran during VR data
collection (src/franka_vr_vel.cpp, tip fr3_link8, frame world):
  linear  = e_p / |e_p| * min(0.30 m/s, 2.0 * |e_p|)
  angular = axis(q_t * q^-1) * min(0.975 rad/s, 2.0 * angle(q_t * q^-1))
The twist goes through damped least-squares IK instead of Servo's IK; the result
is scaled uniformly (direction kept) to the joint speed / acceleration envelope.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from eef_pose_safety import normalize_quaternion, quaternion_multiply, quaternion_to_rotvec, rotation_angle_between

# (xyz, roll) of fr3_joint1..7 origins; pitch and yaw are zero for every FR3 joint.
FR3_JOINT_ORIGINS = (
    ((0.0, 0.0, 0.333), 0.0),
    ((0.0, 0.0, 0.0), -np.pi / 2),
    ((0.0, -0.316, 0.0), np.pi / 2),
    ((0.0825, 0.0, 0.0), np.pi / 2),
    ((-0.0825, 0.384, 0.0), -np.pi / 2),
    ((0.0, 0.0, 0.0), np.pi / 2),
    ((0.088, 0.0, 0.0), np.pi / 2),
)
FR3_FLANGE_Z = 0.107  # fr3_joint8 (fixed) -> fr3_link8
JOINT_NAMES = tuple(f"fr3_joint{index}" for index in range(1, 8))

# Servo tracker constants used for the Task0 recordings (franka_vr_vel.cpp).
SERVO_POSITION_GAIN = 2.0
SERVO_ORIENTATION_GAIN = 2.0
SERVO_MAX_LINEAR_SPEED = 0.30
SERVO_MAX_ANGULAR_SPEED = 0.975


def _rot_x(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rot_z(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def matrix_to_quaternion(matrix: np.ndarray) -> np.ndarray:
    """Rotation matrix -> xyzw quaternion with w >= 0."""
    m = np.asarray(matrix, dtype=np.float64)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = 2.0 * np.sqrt(trace + 1.0)
        q = np.array([(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s])
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        q = np.array([0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s])
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        q = np.array([(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s])
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        q = np.array([(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s])
    q /= np.linalg.norm(q)
    return -q if q[3] < 0 else q


def forward_kinematics(joints: object) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """fr3_joint1..7 -> (fr3_link8 position, xyzw quaternion, 6x7 world-frame Jacobian at fr3_link8)."""
    q = np.asarray(joints, dtype=np.float64)
    if q.shape != (7,) or not np.isfinite(q).all():
        raise ValueError("joints must be 7 finite values")
    rotation, position = np.eye(3), np.zeros(3)
    axes, origins = [], []
    for (xyz, roll), angle in zip(FR3_JOINT_ORIGINS, q):
        position = position + rotation @ np.asarray(xyz)
        rotation = rotation @ _rot_x(roll)
        axes.append(rotation[:, 2].copy()); origins.append(position.copy())
        rotation = rotation @ _rot_z(angle)
    position = position + rotation @ np.array([0.0, 0.0, FR3_FLANGE_Z])
    jacobian = np.zeros((6, 7))
    for index, (axis, origin) in enumerate(zip(axes, origins)):
        jacobian[:3, index] = np.cross(axis, position - origin)
        jacobian[3:, index] = axis
    return position, matrix_to_quaternion(rotation), jacobian


def servo_twist(position: object, quaternion: object, target_position: object, target_quaternion: object,
                position_gain: float = SERVO_POSITION_GAIN, orientation_gain: float = SERVO_ORIENTATION_GAIN,
                max_linear: float = SERVO_MAX_LINEAR_SPEED, max_angular: float = SERVO_MAX_ANGULAR_SPEED) -> np.ndarray:
    """World-frame [v, w] that franka_vr_vel.cpp commands for this pose error."""
    error = np.asarray(target_position, dtype=np.float64) - np.asarray(position, dtype=np.float64)
    distance = float(np.linalg.norm(error))
    linear = np.zeros(3) if distance <= 1e-6 else error / distance * min(max_linear, position_gain * distance)
    current = normalize_quaternion(quaternion)
    difference = quaternion_multiply(normalize_quaternion(target_quaternion), current * [-1.0, -1.0, -1.0, 1.0])
    rotvec = quaternion_to_rotvec(difference)  # w >= 0: the short way round, like the C++ tracker
    angle = float(np.linalg.norm(rotvec))
    angular = np.zeros(3) if angle <= 1e-9 else rotvec / angle * min(max_angular, orientation_gain * angle)
    return np.concatenate([linear, angular])


@dataclass(frozen=True)
class JointCommand:
    velocity: np.ndarray
    twist: np.ndarray
    min_singular_value: float
    speed_scaled: bool
    acceleration_scaled: bool


class DirectPoseTracker:
    """Servo-equivalent pose tracker producing fr3_joint1..7 velocities.

    Uniform scaling keeps the Cartesian direction when a joint would exceed the
    speed or acceleration envelope; the direct executor clamps per joint, which
    would bend the path.
    """

    def __init__(self, max_joint_velocity: float = 0.5, max_joint_acceleration: float = 1.8,
                 damping: float = 0.02, min_singular_value: float = 0.02) -> None:
        if not 0 < max_joint_velocity <= 0.6 or not 0 < max_joint_acceleration <= 3.75:
            raise ValueError("tracker envelope must stay inside the configured 0.6 rad/s / 3.75 rad/s^2 safety envelope")
        self.max_joint_velocity, self.max_joint_acceleration = max_joint_velocity, max_joint_acceleration
        self.damping, self.min_singular_value = damping, min_singular_value
        self.previous = np.zeros(7)

    def reset(self) -> None:
        self.previous = np.zeros(7)

    def command(self, joints: object, target_position: object, target_quaternion: object, dt: float) -> JointCommand:
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError(f"invalid tracker period {dt}")
        position, quaternion, jacobian = forward_kinematics(joints)
        twist = servo_twist(position, quaternion, target_position, target_quaternion)
        sigma = float(np.linalg.svd(jacobian, compute_uv=False)[-1])
        if sigma < self.min_singular_value:
            raise ValueError(f"near a kinematic singularity (sigma_min={sigma:.4f})")
        velocity = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + self.damping ** 2 * np.eye(6), twist)
        peak = float(np.max(np.abs(velocity)))
        speed_scaled = peak > self.max_joint_velocity
        if speed_scaled:
            velocity *= self.max_joint_velocity / peak
        change = velocity - self.previous
        step = float(np.max(np.abs(change)))
        acceleration_scaled = step > self.max_joint_acceleration * dt
        if acceleration_scaled:
            velocity = self.previous + change * (self.max_joint_acceleration * dt / step)
        self.previous = velocity.copy()
        return JointCommand(velocity, twist, sigma, speed_scaled, acceleration_scaled)


def pose_discrepancy(position_a: object, quaternion_a: object, position_b: object,
                     quaternion_b: object) -> tuple[float, float]:
    """(metres, radians) between two poses; used to cross-check FK against TF."""
    distance = float(np.linalg.norm(np.asarray(position_a, float) - np.asarray(position_b, float)))
    return distance, rotation_angle_between(quaternion_a, quaternion_b)


def joints_vector(joints: dict) -> Optional[np.ndarray]:
    if any(name not in joints for name in JOINT_NAMES):
        return None
    return np.array([joints[name] for name in JOINT_NAMES], dtype=np.float64)
