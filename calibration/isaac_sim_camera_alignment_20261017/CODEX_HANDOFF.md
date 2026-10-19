# Codex Handoff: Isaac Sim Camera Alignment

Objective: add three simulated RGB cameras to the user's scanned Isaac Sim scene so their views match the calibrated real cameras.

## Authoritative input

Use `config/camera_alignment.json` as the machine-readable source of truth. Do not re-estimate or round camera poses from screenshots. Do not use the raw C922 camera matrices for rendering; the deployed model consumes rectified C922 images, so use each camera's `render_intrinsics` values.

## Required operations

1. Open the user's scanned USD stage and keep it Z-up.
2. Identify the single Franka hand prim corresponding to ROS `fr3_hand`.
3. Verify or solve `stage_alignment`, defined as `T_stage_ros_world`. Identity is only valid when the simulated Franka base frame exactly matches the physical ROS `world` frame.
4. Run or adapt `scripts/apply_cameras_to_stage.py`.
5. Keep the front camera parented to `fr3_hand`; keep left and right cameras fixed in stage world.
6. Create `640x480` render products for all cameras and preserve the names `front`, `left`, and `right` in downstream observations.
7. Compare views with `reference/cameras_3in1_20261017_1803.png`, then validate at least four corresponding static 3D scene points against measured real-image pixels.
8. Save to a new USD file; do not overwrite the original scan until validation passes.

## Coordinate warning

The source transforms use ROS optical axes (`+X right, +Y down, +Z forward`). A USD Camera looks along `-Z` with `+Y up`. The package represents this as a 180-degree X-axis rotation and the supplied script applies it exactly once. Do not apply a second ROS-to-USD camera rotation.

## Files that are evidence, not runtime dependencies

- `source/calibration/extrinsics_full_solution.yaml`
- `source/calibration/source_bag_metadata.yaml`
- `source/reproducibility/calibrate_camera_extrinsics.py`
- `evidence/*.jpg`

The 11.2 GB source rosbag is intentionally external to this package and is only needed to solve the calibration again.
