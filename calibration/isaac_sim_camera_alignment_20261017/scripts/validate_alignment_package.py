#!/usr/bin/env python3
"""Validate the camera alignment package without Isaac Sim dependencies."""

import argparse
import json
import math
from pathlib import Path


TOLERANCE = 1e-9


def quaternion_multiply(left, right):
    x1, y1, z1, w1 = left
    x2, y2, z2, w2 = right
    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


def equivalent_quaternions(left, right):
    direct = max(abs(a - b) for a, b in zip(left, right))
    negated = max(abs(a + b) for a, b in zip(left, right))
    return min(direct, negated) <= TOLERANCE


def validate_camera(name, camera, width, height):
    optical_q = camera["optical_pose_in_parent"]["rotation_xyzw"]
    usd_q = camera["usd_camera_pose_in_parent"]["rotation_xyzw"]
    expected_usd_q = quaternion_multiply(optical_q, [1.0, 0.0, 0.0, 0.0])
    assert equivalent_quaternions(expected_usd_q, usd_q), f"{name}: optical/USD rotation mismatch"

    for label, quaternion in (("optical", optical_q), ("usd", usd_q)):
        norm = math.sqrt(sum(value * value for value in quaternion))
        assert abs(norm - 1.0) <= TOLERANCE, f"{name}: {label} quaternion norm is {norm}"

    intrinsics = camera["render_intrinsics"]
    lens = camera["usd_lens"]
    focal = lens["focal_length_attr"]
    horizontal_aperture = lens["horizontal_aperture_attr"]
    vertical_aperture = lens["vertical_aperture_attr"]
    horizontal_offset = lens["horizontal_aperture_offset_attr"]
    vertical_offset = lens["vertical_aperture_offset_attr"]

    reconstructed = {
        "fx_px": focal * width / horizontal_aperture,
        "fy_px": focal * height / vertical_aperture,
        "cx_px": width / 2.0 - horizontal_offset * width / horizontal_aperture,
        "cy_px": height / 2.0 + vertical_offset * height / vertical_aperture,
    }
    for key, value in reconstructed.items():
        error = abs(value - intrinsics[key])
        assert error <= 1e-8, f"{name}: {key} reconstruction error is {error} px"

    print(
        f"OK {name:>5}: {width}x{height}, "
        f"fx/fy={intrinsics['fx_px']:.6f}/{intrinsics['fy_px']:.6f}, "
        f"cx/cy={intrinsics['cx_px']:.6f}/{intrinsics['cy_px']:.6f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "config",
        nargs="?",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "config" / "camera_alignment.json",
    )
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8"))

    contract = config["render_contract"]
    width = contract["width_px"]
    height = contract["height_px"]
    assert width == 640 and height == 480
    assert config["isaac_stage"]["required_up_axis"] == "Z"

    for name, camera in config["cameras"].items():
        validate_camera(name, camera, width, height)

    status = config["stage_alignment"]["verification_status"]
    print(f"OK package structure; stage alignment status: {status}")


if __name__ == "__main__":
    main()
