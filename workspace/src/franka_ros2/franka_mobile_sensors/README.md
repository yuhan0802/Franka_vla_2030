# franka_mobile_sensors

ROS 2 package for managing RealSense cameras and SICK safety scanners on Franka Tactile Mobile Robot.

## ⚠️ Note for Franka Tactile Mobile Robot Users

This package is **ignored by colcon by default** (via `COLCON_IGNORE` file). It is only relevant for users of a Franka Tactile Mobile Robot (TMR) or a Franka Mobile FR3 Duo. If you want to enable this package, remove the `COLCON_IGNORE` file:

```bash
rm src/franka_mobile_sensors/COLCON_IGNORE
colcon build --packages-select franka_mobile_sensors --symlink-install
```

## Launch

```bash
ros2 launch franka_mobile_sensors franka_mobile_sensors.launch.py \
  [start_cameras:=true|false] \
  [start_lidars:=true|false] \
  [start_rviz:=true|false] \
  [robot_type:=<robot_type>] \
  [config_file:=<config_name>] 
```

**Parameters:**
- `start_cameras` (default: `true`) - Start RealSense camera drivers
- `start_lidars` (default: `true`) - Start SICK safety scanner drivers  
- `start_rviz` (default: `true`) - Start RViz visualization
- `robot_type` (default: `tmrv0_2`) - ID of the robot type for visualization
- `config_file` (default: `default_sensor_suite`) - Sensor suite configuration (without .yaml extension)

## Configuration

### Sensor Suite Configuration

The main sensor suite is configured in:
- **`config/default_sensor_suite.yaml`** - Defines which cameras and lidars are used

### Device-Specific Parameters

Individual device parameters are configured in device profile files:

**Cameras:**
- `config/cameras/franka_mobile_d455.yaml` - RealSense D455 parameters

**Lidars:**
- `config/lidars/sick_nanoscan2.yaml` - SICK nanoScan2 parameters

To create a custom configuration:
1. Copy `config/default_sensor_suite.yaml` to `config/my_custom_suite.yaml`
2. Modify camera/lidar lists and reference existing or new device profiles
3. Launch with: `config_file:=my_custom_suite`
