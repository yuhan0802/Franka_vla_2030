#!/usr/bin/env python3
"""Solve wrist and fixed-camera extrinsics from a ROS 2 calibration bag."""

import argparse
import math
import sqlite3
from dataclasses import dataclass
from pathlib import Path

import cv2
from geometry_msgs.msg import PoseStamped
import numpy as np
from rclpy.serialization import deserialize_message
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, CompressedImage
from tf2_msgs.msg import TFMessage
import yaml


CAMERA_TOPICS = {
    "front": (
        "/camera/camera_front/color/image_raw/compressed",
        "/camera/camera_front/color/camera_info",
    ),
    "left": ("/camera_left/image_raw/compressed", "/camera_left/camera_info"),
    "right": ("/camera_right/image_raw/compressed", "/camera_right/camera_info"),
}
POSE_TOPIC = "/franka_robot_state_broadcaster/current_pose"


@dataclass
class CameraModel:
    matrix: np.ndarray
    distortion: np.ndarray


@dataclass
class Observation:
    timestamp_ns: int
    base_to_ee: np.ndarray
    camera_to_board: np.ndarray
    reprojection_rms_px: float


def message_stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def quaternion_matrix(values) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(values).as_matrix()
    return transform


def pose_matrix(position, quaternion) -> np.ndarray:
    transform = quaternion_matrix(quaternion)
    transform[:3, 3] = position
    return transform


def pose_message_matrix(message: PoseStamped) -> np.ndarray:
    position = message.pose.position
    orientation = message.pose.orientation
    return pose_matrix(
        [position.x, position.y, position.z],
        [orientation.x, orientation.y, orientation.z, orientation.w],
    )


def transform_message_matrix(message) -> np.ndarray:
    translation = message.transform.translation
    rotation = message.transform.rotation
    return pose_matrix(
        [translation.x, translation.y, translation.z],
        [rotation.x, rotation.y, rotation.z, rotation.w],
    )


def transform_from_vector(values: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_rotvec(values[:3]).as_matrix()
    transform[:3, 3] = values[3:]
    return transform


def transform_to_vector(transform: np.ndarray) -> np.ndarray:
    return np.concatenate(
        [Rotation.from_matrix(transform[:3, :3]).as_rotvec(), transform[:3, 3]]
    )


def rotation_distance_radians(left: np.ndarray, right: np.ndarray) -> float:
    relative = left[:3, :3].T @ right[:3, :3]
    return float(np.linalg.norm(Rotation.from_matrix(relative).as_rotvec()))


def transform_distance(left: np.ndarray, right: np.ndarray) -> tuple[float, float]:
    translation = float(np.linalg.norm(left[:3, 3] - right[:3, 3]))
    return translation, rotation_distance_radians(left, right)


def average_transform(transforms: list[np.ndarray]) -> np.ndarray:
    if not transforms:
        raise ValueError("Cannot average an empty transform list")
    result = np.eye(4)
    result[:3, 3] = np.median([item[:3, 3] for item in transforms], axis=0)
    quaternions = np.asarray(
        [Rotation.from_matrix(item[:3, :3]).as_quat() for item in transforms]
    )
    reference = quaternions[0]
    quaternions[np.sum(quaternions * reference, axis=1) < 0.0] *= -1.0
    accumulator = sum(np.outer(item, item) for item in quaternions)
    _, vectors = np.linalg.eigh(accumulator)
    quaternion = vectors[:, -1]
    if np.dot(quaternion, reference) < 0.0:
        quaternion *= -1.0
    result[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    return result


def residual_metrics(reference: np.ndarray, transforms: list[np.ndarray]) -> dict:
    translation_mm = []
    rotation_deg = []
    for transform in transforms:
        translation, rotation = transform_distance(reference, transform)
        translation_mm.append(translation * 1000.0)
        rotation_deg.append(math.degrees(rotation))
    return {
        "translation_rms_mm": float(np.sqrt(np.mean(np.square(translation_mm)))),
        "translation_median_mm": float(np.median(translation_mm)),
        "translation_max_mm": float(np.max(translation_mm)),
        "rotation_rms_deg": float(np.sqrt(np.mean(np.square(rotation_deg)))),
        "rotation_median_deg": float(np.median(rotation_deg)),
        "rotation_max_deg": float(np.max(rotation_deg)),
    }


class BagDatabase:
    def __init__(self, bag_directory: Path):
        database_files = sorted(bag_directory.glob("*.db3"))
        if len(database_files) != 1:
            raise RuntimeError(
                f"Expected one .db3 file in {bag_directory}, found {len(database_files)}"
            )
        self.connection = sqlite3.connect(str(database_files[0]))
        self.topic_ids = {
            name: topic_id
            for topic_id, name in self.connection.execute("SELECT id, name FROM topics")
        }

    def topic_id(self, name: str) -> int:
        if name not in self.topic_ids:
            raise RuntimeError(f"Required topic is missing from bag: {name}")
        return self.topic_ids[name]

    def first_message(self, topic: str, message_type):
        row = self.connection.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = ? "
            "ORDER BY timestamp LIMIT 1",
            (self.topic_id(topic),),
        ).fetchone()
        if row is None:
            raise RuntimeError(f"Topic has no messages: {topic}")
        return row[0], deserialize_message(row[1], message_type)

    def nearest_message(self, topic: str, timestamp_ns: int, message_type):
        topic_id = self.topic_id(topic)
        before = self.connection.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = ? AND timestamp <= ? "
            "ORDER BY timestamp DESC LIMIT 1",
            (topic_id, timestamp_ns),
        ).fetchone()
        after = self.connection.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = ? AND timestamp >= ? "
            "ORDER BY timestamp ASC LIMIT 1",
            (topic_id, timestamp_ns),
        ).fetchone()
        rows = [row for row in (before, after) if row is not None]
        if not rows:
            raise RuntimeError(f"Could not find a message on {topic}")
        row = min(rows, key=lambda item: abs(item[0] - timestamp_ns))
        return row[0], deserialize_message(row[1], message_type)

    def topic_time_bounds(self, topic: str) -> tuple[int, int]:
        row = self.connection.execute(
            "SELECT MIN(timestamp), MAX(timestamp) FROM messages WHERE topic_id = ?",
            (self.topic_id(topic),),
        ).fetchone()
        return int(row[0]), int(row[1])

    def load_poses(self, minimum_interval_ns: int = 10_000_000):
        timestamps = []
        transforms = []
        last_stamp = -minimum_interval_ns
        cursor = self.connection.execute(
            "SELECT data FROM messages WHERE topic_id = ? ORDER BY timestamp",
            (self.topic_id(POSE_TOPIC),),
        )
        for (data,) in cursor:
            message = deserialize_message(data, PoseStamped)
            stamp = message_stamp_ns(message)
            if stamp - last_stamp < minimum_interval_ns:
                continue
            timestamps.append(stamp)
            transforms.append(pose_message_matrix(message))
            last_stamp = stamp
        if not timestamps:
            raise RuntimeError(f"No poses found on {POSE_TOPIC}")
        return np.asarray(timestamps, dtype=np.int64), transforms

    def static_transform(self, parent_frame: str, child_frame: str) -> np.ndarray:
        topic_id = self.topic_id("/tf_static")
        adjacency = {}
        for (data,) in self.connection.execute(
            "SELECT data FROM messages WHERE topic_id = ? ORDER BY timestamp", (topic_id,)
        ):
            message = deserialize_message(data, TFMessage)
            for stamped in message.transforms:
                parent = stamped.header.frame_id
                child = stamped.child_frame_id
                transform = transform_message_matrix(stamped)
                adjacency.setdefault(parent, []).append((child, transform))
                adjacency.setdefault(child, []).append((parent, np.linalg.inv(transform)))
        queue = [(parent_frame, np.eye(4))]
        visited = set()
        while queue:
            frame, parent_to_frame = queue.pop(0)
            if frame == child_frame:
                return parent_to_frame
            if frame in visited:
                continue
            visited.add(frame)
            for next_frame, frame_to_next in adjacency.get(frame, []):
                if next_frame not in visited:
                    queue.append((next_frame, parent_to_frame @ frame_to_next))
        raise RuntimeError(
            f"No /tf_static path from {parent_frame} to {child_frame} was found"
        )


def camera_model(database: BagDatabase, info_topic: str) -> CameraModel:
    _, message = database.first_message(info_topic, CameraInfo)
    return CameraModel(
        matrix=np.asarray(message.k, dtype=np.float64).reshape(3, 3),
        distortion=np.asarray(message.d, dtype=np.float64),
    )


def object_points(pattern: tuple[int, int], square_size: float) -> np.ndarray:
    points = np.zeros((pattern[0] * pattern[1], 3), dtype=np.float32)
    points[:, :2] = np.mgrid[0 : pattern[0], 0 : pattern[1]].T.reshape(-1, 2)
    points[:, :2] *= square_size
    return points


def detect_board(
    message: CompressedImage,
    model: CameraModel,
    board_points: np.ndarray,
    pattern: tuple[int, int],
):
    image = cv2.imdecode(np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        return None
    grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    found, corners = cv2.findChessboardCornersSB(grayscale, pattern, flags)
    if not found:
        return None
    success, rotation_vector, translation_vector = cv2.solvePnP(
        board_points,
        corners,
        model.matrix,
        model.distortion,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not success:
        return None
    if hasattr(cv2, "solvePnPRefineLM"):
        rotation_vector, translation_vector = cv2.solvePnPRefineLM(
            board_points,
            corners,
            model.matrix,
            model.distortion,
            rotation_vector,
            translation_vector,
        )
    projected, _ = cv2.projectPoints(
        board_points, rotation_vector, translation_vector, model.matrix, model.distortion
    )
    error = projected.reshape(-1, 2) - corners.reshape(-1, 2)
    reprojection_rms = float(np.sqrt(np.mean(np.sum(np.square(error), axis=1))))
    rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
    camera_to_board = np.eye(4)
    camera_to_board[:3, :3] = rotation_matrix
    camera_to_board[:3, 3] = translation_vector.reshape(3)
    return camera_to_board, reprojection_rms, image, corners


def pose_index(timestamps: np.ndarray, timestamp_ns: int) -> int:
    index = int(np.searchsorted(timestamps, timestamp_ns))
    if index <= 0:
        return 0
    if index >= len(timestamps):
        return len(timestamps) - 1
    if timestamp_ns - timestamps[index - 1] <= timestamps[index] - timestamp_ns:
        return index - 1
    return index


def stable_pose_candidates(
    timestamps: np.ndarray,
    transforms: list[np.ndarray],
    window_seconds: float = 0.3,
):
    interval = max(1, int(round(window_seconds * 1e9 / np.median(np.diff(timestamps)))))
    candidates = []
    step = max(1, interval // 2)
    for index in range(interval, len(timestamps) - interval, step):
        translation, rotation = transform_distance(
            transforms[index - interval], transforms[index + interval]
        )
        if translation > 0.0008 or rotation > math.radians(0.25):
            continue
        current = transforms[index]
        duplicate = False
        for _, accepted in candidates:
            accepted_translation, accepted_rotation = transform_distance(accepted, current)
            if accepted_translation < 0.003 and accepted_rotation < math.radians(0.9):
                duplicate = True
                break
        if not duplicate:
            candidates.append((int(timestamps[index]), current))
    return candidates


def collect_hand_eye_observations(
    database: BagDatabase,
    timestamps: np.ndarray,
    transforms: list[np.ndarray],
    model: CameraModel,
    board_points: np.ndarray,
    pattern: tuple[int, int],
    preview_directory: Path,
) -> list[Observation]:
    observations = []
    image_topic = CAMERA_TOPICS["front"][0]
    for candidate_stamp, base_to_ee in stable_pose_candidates(timestamps, transforms):
        bag_stamp, message = database.nearest_message(
            image_topic, candidate_stamp, CompressedImage
        )
        if abs(bag_stamp - candidate_stamp) > 150_000_000:
            continue
        detection = detect_board(message, model, board_points, pattern)
        if detection is None:
            continue
        camera_to_board, reprojection_rms, image, corners = detection
        if reprojection_rms > 1.0:
            continue
        observation = Observation(
            timestamp_ns=candidate_stamp,
            base_to_ee=base_to_ee,
            camera_to_board=camera_to_board,
            reprojection_rms_px=reprojection_rms,
        )
        observations.append(observation)
        preview = image.copy()
        cv2.drawChessboardCorners(preview, pattern, corners, True)
        cv2.imwrite(str(preview_directory / f"front_{len(observations):03d}.jpg"), preview)
    return observations


def hand_eye_candidates(observations: list[Observation]):
    rotations_gripper_to_base = [item.base_to_ee[:3, :3] for item in observations]
    translations_gripper_to_base = [item.base_to_ee[:3, 3] for item in observations]
    rotations_target_to_camera = [item.camera_to_board[:3, :3] for item in observations]
    translations_target_to_camera = [item.camera_to_board[:3, 3] for item in observations]
    methods = {
        "tsai": cv2.CALIB_HAND_EYE_TSAI,
        "park": cv2.CALIB_HAND_EYE_PARK,
        "horaud": cv2.CALIB_HAND_EYE_HORAUD,
        "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
        "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    results = []
    for name, method in methods.items():
        try:
            rotation, translation = cv2.calibrateHandEye(
                rotations_gripper_to_base,
                translations_gripper_to_base,
                rotations_target_to_camera,
                translations_target_to_camera,
                method=method,
            )
        except cv2.error:
            continue
        ee_to_camera = np.eye(4)
        ee_to_camera[:3, :3] = rotation
        ee_to_camera[:3, 3] = np.asarray(translation).reshape(3)
        if not np.all(np.isfinite(ee_to_camera)):
            continue
        base_to_board_values = [
            item.base_to_ee @ ee_to_camera @ item.camera_to_board
            for item in observations
        ]
        base_to_board = average_transform(base_to_board_values)
        metrics = residual_metrics(base_to_board, base_to_board_values)
        score = metrics["translation_rms_mm"] + 2.0 * metrics["rotation_rms_deg"]
        results.append((score, name, ee_to_camera, base_to_board, metrics))
    if not results:
        raise RuntimeError("Every OpenCV hand-eye method failed")
    return sorted(results, key=lambda item: item[0])


def refine_hand_eye(
    observations: list[Observation],
    ee_to_camera: np.ndarray,
    base_to_board: np.ndarray,
):
    initial = np.concatenate(
        [transform_to_vector(ee_to_camera), transform_to_vector(base_to_board)]
    )

    def residual(values):
        current_ee_to_camera = transform_from_vector(values[:6])
        current_base_to_board = transform_from_vector(values[6:])
        inverse_board = np.linalg.inv(current_base_to_board)
        components = []
        for item in observations:
            error = (
                inverse_board
                @ item.base_to_ee
                @ current_ee_to_camera
                @ item.camera_to_board
            )
            components.extend(error[:3, 3] / 0.002)
            components.extend(
                Rotation.from_matrix(error[:3, :3]).as_rotvec() / math.radians(0.5)
            )
        return np.asarray(components)

    result = least_squares(residual, initial, loss="soft_l1", f_scale=1.0, max_nfev=500)
    return transform_from_vector(result.x[:6]), transform_from_vector(result.x[6:])


def reject_hand_eye_outliers(
    observations: list[Observation], ee_to_camera: np.ndarray, base_to_board: np.ndarray
):
    translation = []
    rotation = []
    for item in observations:
        estimate = item.base_to_ee @ ee_to_camera @ item.camera_to_board
        current_translation, current_rotation = transform_distance(base_to_board, estimate)
        translation.append(current_translation)
        rotation.append(current_rotation)
    translation = np.asarray(translation)
    rotation = np.asarray(rotation)
    translation_limit = max(
        0.004,
        float(np.median(translation) + 3.0 * np.median(np.abs(translation - np.median(translation)))),
    )
    rotation_limit = max(
        math.radians(1.0),
        float(np.median(rotation) + 3.0 * np.median(np.abs(rotation - np.median(rotation)))),
    )
    return [
        item
        for item, translation_error, rotation_error in zip(
            observations, translation, rotation
        )
        if translation_error <= translation_limit and rotation_error <= rotation_limit
    ]


def solve_hand_eye(observations: list[Observation]):
    if len(observations) < 8:
        raise RuntimeError(f"Need at least 8 valid hand-eye observations, got {len(observations)}")
    method_results = hand_eye_candidates(observations)
    _, method, ee_to_camera, base_to_board, _ = method_results[0]
    ee_to_camera, base_to_board = refine_hand_eye(
        observations, ee_to_camera, base_to_board
    )
    inliers = reject_hand_eye_outliers(observations, ee_to_camera, base_to_board)
    if len(inliers) < 8:
        raise RuntimeError(f"Hand-eye outlier rejection left only {len(inliers)} samples")
    method_results = hand_eye_candidates(inliers)
    _, method, ee_to_camera, base_to_board, _ = method_results[0]
    ee_to_camera, base_to_board = refine_hand_eye(inliers, ee_to_camera, base_to_board)
    board_estimates = [
        item.base_to_ee @ ee_to_camera @ item.camera_to_board for item in inliers
    ]
    return {
        "method": method,
        "ee_to_camera": ee_to_camera,
        "base_to_board": base_to_board,
        "inliers": inliers,
        "metrics": residual_metrics(base_to_board, board_estimates),
        "method_metrics": {
            name: metrics for _, name, _, _, metrics in method_results
        },
    }


def collect_fixed_camera_board_poses(
    database: BagDatabase,
    camera: str,
    model: CameraModel,
    board_points: np.ndarray,
    pattern: tuple[int, int],
    preview_directory: Path,
):
    image_topic = CAMERA_TOPICS[camera][0]
    start, end = database.topic_time_bounds(image_topic)
    query_times = np.linspace(start + 2_000_000_000, end - 2_000_000_000, 40)
    transforms = []
    reprojection = []
    for timestamp in query_times.astype(np.int64):
        _, message = database.nearest_message(image_topic, int(timestamp), CompressedImage)
        detection = detect_board(message, model, board_points, pattern)
        if detection is None:
            continue
        camera_to_board, reprojection_rms, image, corners = detection
        if reprojection_rms > 1.0:
            continue
        transforms.append(camera_to_board)
        reprojection.append(reprojection_rms)
        if len(transforms) == 1:
            preview = image.copy()
            cv2.drawChessboardCorners(preview, pattern, corners, True)
            cv2.imwrite(str(preview_directory / f"{camera}_board.jpg"), preview)
    if len(transforms) < 8:
        raise RuntimeError(f"Only {len(transforms)} valid observations for {camera} camera")
    initial = average_transform(transforms)
    errors = np.asarray([transform_distance(initial, item)[0] for item in transforms])
    median = float(np.median(errors))
    limit = max(0.003, median + 3.0 * float(np.median(np.abs(errors - median))))
    keep = [item for item, error in zip(transforms, errors) if error <= limit]
    mean_transform = average_transform(keep)
    return mean_transform, {
        "samples": len(keep),
        "reprojection_rms_px": float(np.sqrt(np.mean(np.square(reprojection)))),
        **residual_metrics(mean_transform, keep),
    }


def transform_record(parent: str, child: str, transform: np.ndarray) -> dict:
    quaternion = Rotation.from_matrix(transform[:3, :3]).as_quat()
    return {
        "parent_frame": parent,
        "child_frame": child,
        "translation_xyz_m": [float(value) for value in transform[:3, 3]],
        "rotation_xyzw": [float(value) for value in quaternion],
    }


def parse_transform(value: str) -> np.ndarray:
    numbers = [float(item.strip()) for item in value.split(",")]
    if len(numbers) != 7:
        raise argparse.ArgumentTypeError("Expected x,y,z,qx,qy,qz,qw")
    return pose_matrix(numbers[:3], numbers[3:])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--pattern-columns", type=int, default=9)
    parser.add_argument("--pattern-rows", type=int, default=6)
    parser.add_argument("--square-size", type=float, default=0.025)
    parser.add_argument("--base-to-hand", type=parse_transform, required=True)
    parser.add_argument("--base-to-ee", type=parse_transform, required=True)
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)
    preview_directory = args.output / "previews"
    preview_directory.mkdir(parents=True, exist_ok=True)
    database = BagDatabase(args.bag)
    pattern = (args.pattern_columns, args.pattern_rows)
    board_points = object_points(pattern, args.square_size)
    models = {
        name: camera_model(database, topics[1]) for name, topics in CAMERA_TOPICS.items()
    }
    pose_timestamps, pose_transforms = database.load_poses()
    observations = collect_hand_eye_observations(
        database,
        pose_timestamps,
        pose_transforms,
        models["front"],
        board_points,
        pattern,
        preview_directory,
    )
    hand_eye = solve_hand_eye(observations)
    base_to_board = hand_eye["base_to_board"]

    fixed_results = {}
    for camera in ("left", "right"):
        camera_to_board, metrics = collect_fixed_camera_board_poses(
            database,
            camera,
            models[camera],
            board_points,
            pattern,
            preview_directory,
        )
        fixed_results[camera] = {
            "base_to_camera": base_to_board @ np.linalg.inv(camera_to_board),
            "metrics": metrics,
        }

    hand_to_ee = np.linalg.inv(args.base_to_hand) @ args.base_to_ee
    hand_to_front = hand_to_ee @ hand_eye["ee_to_camera"]
    front_link_to_optical = database.static_transform(
        "camera_front_link", "camera_front_color_optical_frame"
    )
    hand_to_front_link = hand_to_front @ np.linalg.inv(front_link_to_optical)
    result = {
        "source_bag": str(args.bag),
        "checkerboard": {
            "inner_corners": [args.pattern_columns, args.pattern_rows],
            "square_size_m": args.square_size,
        },
        "transforms": {
            "hand_to_front_link": transform_record(
                "fr3_hand", "camera_front_link", hand_to_front_link
            ),
            "hand_to_front": transform_record(
                "fr3_hand", "camera_front_color_optical_frame", hand_to_front
            ),
            "ee_to_front": transform_record(
                "fr3_EE", "camera_front_color_optical_frame", hand_eye["ee_to_camera"]
            ),
            "world_to_left": transform_record(
                "world", "camera_left_optical_frame", fixed_results["left"]["base_to_camera"]
            ),
            "world_to_right": transform_record(
                "world", "camera_right_optical_frame", fixed_results["right"]["base_to_camera"]
            ),
            "world_to_board": transform_record("world", "checkerboard", base_to_board),
        },
        "quality": {
            "hand_eye_method": hand_eye["method"],
            "hand_eye_candidates": len(observations),
            "hand_eye_inliers": len(hand_eye["inliers"]),
            "front_reprojection_rms_px": float(
                np.sqrt(
                    np.mean(
                        np.square(
                            [item.reprojection_rms_px for item in hand_eye["inliers"]]
                        )
                    )
                )
            ),
            "hand_eye_consistency": hand_eye["metrics"],
            "hand_eye_method_metrics": hand_eye["method_metrics"],
            "left_fixed_camera": fixed_results["left"]["metrics"],
            "right_fixed_camera": fixed_results["right"]["metrics"],
        },
    }
    output_file = args.output / "camera_extrinsics.yaml"
    output_file.write_text(yaml.safe_dump(result, sort_keys=False), encoding="utf-8")
    print(yaml.safe_dump(result["quality"], sort_keys=False))
    print(f"Wrote {output_file}")


if __name__ == "__main__":
    main()
