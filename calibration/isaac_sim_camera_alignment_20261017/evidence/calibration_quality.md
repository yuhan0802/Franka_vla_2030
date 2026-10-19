# Calibration Quality

Calibration date: 2026-10-17  
Checkerboard: 9 x 6 inner corners, 0.025 m square size  
Source bag: `camera_extrinsics_20261017_163011`

| Metric | Result |
|---|---:|
| Hand-eye candidates / inliers | 22 / 22 |
| Front reprojection RMS | 0.1417097 px |
| Hand-eye translation RMS | 0.7851814 mm |
| Hand-eye translation max | 1.9112960 mm |
| Hand-eye rotation RMS | 0.2704015 deg |
| Hand-eye rotation max | 0.6161810 deg |
| Left fixed-camera samples | 40 |
| Left reprojection RMS | 0.1473136 px |
| Left repeat translation RMS | 0.0796264 mm |
| Right fixed-camera samples | 40 |
| Right reprojection RMS | 0.1212715 px |
| Right repeat translation RMS | 0.0644134 mm |

The deployment transform for the front camera uses `fr3_hand -> camera_front_color_optical_frame` directly. The production ROS setup stores `fr3_hand -> camera_front_link` and obtains the final optical transform from the RealSense static TF tree; both representations are preserved in the source files.
