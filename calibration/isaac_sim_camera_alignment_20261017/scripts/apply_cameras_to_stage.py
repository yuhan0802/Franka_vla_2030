"""Create the three calibrated cameras in the currently open Isaac Sim stage.

Run this file from Isaac Sim's Script Editor. Open the scanned scene first.
Set CONFIG_PATH only when the Script Editor does not preserve __file__.
"""

import json
import math
import os
from pathlib import Path

import omni.usd
from pxr import Gf, Sdf, UsdGeom


CONFIG_PATH = os.environ.get("CAMERA_ALIGNMENT_CONFIG", "")


def load_config():
    if CONFIG_PATH:
        path = Path(CONFIG_PATH).expanduser().resolve()
    elif "__file__" in globals():
        path = Path(__file__).resolve().parents[1] / "config" / "camera_alignment.json"
    else:
        raise RuntimeError(
            "Set CONFIG_PATH to the absolute camera_alignment.json path at the top of this script."
        )
    print(f"Loading camera alignment: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def quaternion_multiply(left, right):
    x1, y1, z1, w1 = left
    x2, y2, z2, w2 = right
    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


def quaternion_conjugate(quaternion):
    x, y, z, w = quaternion
    return [-x, -y, -z, w]


def rotate_vector(quaternion, vector):
    pure = [vector[0], vector[1], vector[2], 0.0]
    rotated = quaternion_multiply(
        quaternion_multiply(quaternion, pure), quaternion_conjugate(quaternion)
    )
    return rotated[:3]


def compose_pose(parent_pose, child_pose):
    parent_translation, parent_rotation = parent_pose
    child_translation, child_rotation = child_pose
    translated = rotate_vector(parent_rotation, child_translation)
    return (
        [parent_translation[index] + translated[index] for index in range(3)],
        quaternion_multiply(parent_rotation, child_rotation),
    )


def normalize_quaternion(quaternion):
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1e-12:
        raise ValueError("Zero-length quaternion")
    return [value / norm for value in quaternion]


def discover_hand_prim(stage, configured_path):
    if configured_path.startswith("/"):
        prim = stage.GetPrimAtPath(configured_path)
        if prim.IsValid():
            return configured_path

    matches = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if prim.GetName() == "fr3_hand"
    ]
    if len(matches) != 1:
        raise RuntimeError(
            "Expected exactly one prim named fr3_hand; found "
            f"{len(matches)}: {matches}. Set isaac_stage.robot_hand_prim_path explicitly."
        )
    return matches[0]


def set_local_pose(prim, translation_m, rotation_xyzw, meters_per_unit):
    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    translate_op = xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
    orient_op = xformable.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
    translate_op.Set(Gf.Vec3d(*(value / meters_per_unit for value in translation_m)))
    x, y, z, w = normalize_quaternion(rotation_xyzw)
    orient_op.Set(Gf.Quatd(w, Gf.Vec3d(x, y, z)))


def configure_camera(stage, path, camera_data, pose, meters_per_unit, contract):
    camera = UsdGeom.Camera.Define(stage, Sdf.Path(path))
    set_local_pose(camera.GetPrim(), pose[0], pose[1], meters_per_unit)

    lens = camera_data["usd_lens"]
    camera.CreateProjectionAttr().Set(UsdGeom.Tokens.perspective)
    camera.CreateFocalLengthAttr().Set(float(lens["focal_length_attr"]))
    camera.CreateHorizontalApertureAttr().Set(float(lens["horizontal_aperture_attr"]))
    camera.CreateVerticalApertureAttr().Set(float(lens["vertical_aperture_attr"]))
    camera.CreateHorizontalApertureOffsetAttr().Set(
        float(lens["horizontal_aperture_offset_attr"])
    )
    camera.CreateVerticalApertureOffsetAttr().Set(
        float(lens["vertical_aperture_offset_attr"])
    )
    camera.CreateFStopAttr().Set(float(lens["f_stop"]))
    camera.CreateClippingRangeAttr().Set(
        Gf.Vec2f(contract["near_clip_m"] / meters_per_unit, contract["far_clip_m"] / meters_per_unit)
    )

    prim = camera.GetPrim()
    intrinsics = camera_data["render_intrinsics"]
    metadata = {
        "calibration:childFrame": camera_data["child_frame"],
        "calibration:resolution": f"{contract['width_px']}x{contract['height_px']}",
        "calibration:fxPx": str(intrinsics["fx_px"]),
        "calibration:fyPx": str(intrinsics["fy_px"]),
        "calibration:cxPx": str(intrinsics["cx_px"]),
        "calibration:cyPx": str(intrinsics["cy_px"]),
        "calibration:inputRepresentation": intrinsics["source"],
    }
    for name, value in metadata.items():
        prim.CreateAttribute(name, Sdf.ValueTypeNames.String, custom=True).Set(value)
    return path


def main():
    config = load_config()
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No USD stage is open. Open the scanned scene first.")

    up_axis = str(UsdGeom.GetStageUpAxis(stage)).upper()
    required_axis = config["isaac_stage"]["required_up_axis"]
    if up_axis != required_axis:
        raise RuntimeError(f"Stage up axis is {up_axis}; calibration requires {required_axis}-up.")

    meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)
    if not meters_per_unit or meters_per_unit <= 0.0:
        raise RuntimeError(f"Invalid stage metersPerUnit: {meters_per_unit}")

    world_prim = stage.GetPrimAtPath("/World")
    if world_prim.IsValid() and UsdGeom.Xformable(world_prim).GetOrderedXformOps():
        raise RuntimeError(
            "/World has authored transforms. Flatten or clear them before applying calibrated world poses."
        )

    stage_config = config["isaac_stage"]
    fixed_root_path = stage_config["fixed_camera_root_prim"]
    fixed_root = UsdGeom.Xform.Define(stage, Sdf.Path(fixed_root_path))
    UsdGeom.Xformable(fixed_root.GetPrim()).ClearXformOpOrder()
    hand_path = discover_hand_prim(stage, stage_config["robot_hand_prim_path"])

    stage_alignment = config["stage_alignment"]
    stage_from_world = (
        stage_alignment["translation_xyz_m"],
        normalize_quaternion(stage_alignment["rotation_xyzw"]),
    )
    optical_from_usd = config["coordinate_conventions"]["optical_from_usd_rotation_xyzw"]
    contract = config["render_contract"]

    created_paths = {}
    for name, camera_data in config["cameras"].items():
        optical = camera_data["optical_pose_in_parent"]
        optical_pose = (
            optical["translation_xyz_m"],
            normalize_quaternion(optical["rotation_xyzw"]),
        )
        usd_pose = (optical_pose[0], quaternion_multiply(optical_pose[1], optical_from_usd))

        if camera_data["mount"] == "fixed_world":
            pose = compose_pose(stage_from_world, usd_pose)
            path = f"{fixed_root_path}/{camera_data['prim_name']}"
        elif camera_data["mount"] == "robot_link":
            pose = usd_pose
            path = f"{hand_path}/{camera_data['prim_name']}"
        else:
            raise ValueError(f"Unsupported mount type for {name}: {camera_data['mount']}")

        created_paths[name] = configure_camera(
            stage, path, camera_data, pose, meters_per_unit, contract
        )

    print("Created calibrated Isaac Sim cameras:")
    for name, path in created_paths.items():
        print(f"  {name}: {path} ({contract['width_px']}x{contract['height_px']})")
    print(
        "Stage changes are not saved automatically. Inspect all three views, then use File > Save As."
    )
    print(f"Stage alignment status: {stage_alignment['verification_status']}")


main()
