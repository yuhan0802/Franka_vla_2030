#  Copyright (c) 2025 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

############################################################################
# Parameters:
# controller_name: Name of the controller to spawn (required, no default)
# robot_config_file: Path to the robot configuration file to load
#                   (default: franka.config.yaml in franka_bringup/config)
#
# The example.launch.py launch file provides a flexible and unified interface
# for launching Franka Robotics example controllers via the 'controller_name'
# parameter, such as 'elbow_example_controller'.
# Example:
# ros2 launch franka_bringup example.launch.py controller_name:=elbow_example_controller
#
# This script "includes" franka.launch.py to declare core component nodes,
# including: robot_state_publisher, ros2_control_node, joint_state_publisher,
# joint_state_broadcaster, franka_robot_state_broadcaster, and optionally
# franka_gripper and rviz, with support for namespaced and non-namespaced
# environments as defined in franka.config.yaml. RViz is launched if
# 'use_rviz' is set to true in the configuration file.
#
# The default robot_config_file is franka.config.yaml in the
# franka_bringup/config directory. See that file for its own documentation.
#
# This approach improves upon the earlier individual launch scripts, which
# varied in structure and lacked namespace support, offering a more consistent
# and maintainable solution. While some may favor the older scripts for their
# specific use cases, example.launch.py enhances scalability and ease of use
# for a wide range of Franka Robotics applications.
#
# Ensure the specified  controller_name matches a controller defined in
#  controllers.yaml to avoid runtime errors.
############################################################################

import importlib.util
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# constant for the controller name parameter
CONTROLLER_EXAMPLE = 'controller'

package_share = get_package_share_directory('franka_bringup')
utils_path = os.path.abspath(
    os.path.join(package_share, '..', '..', 'lib', 'franka_bringup', 'utils')
)
launch_utils_path = os.path.join(utils_path, 'launch_utils.py')

spec = importlib.util.spec_from_file_location('launch_utils', launch_utils_path)
launch_utils = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch_utils)

load_yaml = launch_utils.load_yaml

# Iterates over the uncommented lines in file specified by the robot_config_file parameter.
# 'Includes' franka.launch.py for each active (uncommented) Robot.
# That file is well documented.
# The function also checks if the 'use_rviz' parameter is set to true in the YAML file.
# If so, it includes a node for RViz to visualize the robot's state.
# The function returns a list of nodes to be launched.


def generate_robot_nodes(context):
    config_file = LaunchConfiguration('robot_config_file').perform(context)
    controller_names = LaunchConfiguration('controller_names').perform(context)
    controller_names_vector = controller_names.split(',')
    configs = load_yaml(config_file)
    nodes = []
    for index, (_, config) in enumerate(configs.items()):
        namespace = config['namespace']
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare('franka_bringup'),
                            'launch',
                            'franka.launch.py',
                        ]
                    )
                ),
                launch_arguments={
                    'robot_type': str(config['robot_type']),
                    'arm_prefix': str(config['arm_prefix']),
                    'namespace': str(namespace),
                    'robot_ip': str(config['robot_ip']),
                    'load_gripper': str(config['load_gripper']),
                    'use_fake_hardware': str(config['use_fake_hardware']),
                    'fake_sensor_commands': str(config['fake_sensor_commands']),
                    'joint_state_rate': str(config['joint_state_rate']),
                }.items(),
            )
        )
        if controller_names_vector:
            if len(controller_names_vector) == len(configs):
                controller_name = controller_names_vector[index]
            else:
                print(
                    'Warning: Number of controller names does not match number of robot configs.'
                    ' Using the first controller for all robots.'
                )
                controller_name = controller_names_vector[0]
        else:
            print(
                'Error: No controller names provided. Please provide at least one controller name.'
            )
            sys.exit(1)

        if CONTROLLER_EXAMPLE in controller_name:
            # Spawn the example as ros2_control controller
            controller_name = controller_names_vector[index]
            nodes.append(
                Node(
                    package='controller_manager',
                    executable='spawner',
                    namespace=namespace,
                    arguments=[controller_name, '--controller-manager-timeout', '30'],
                    parameters=[PathJoinSubstitution([
                        FindPackageShare('franka_bringup'), 'config', 'controllers.yaml',
                    ])],
                    output='screen',
                )
            )
        else:
            # Spawn the example as node
            nodes.append(
                Node(
                    package='franka_example_controllers',
                    executable=controller_name,
                    namespace=namespace,
                    output='screen',
                )
            )

    if any(
        str(config.get('use_rviz', 'false')).lower() == 'true'
        for config in configs.values()
    ):
        nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=[
                    '--display-config',
                    PathJoinSubstitution(
                        [
                            FindPackageShare('franka_description'),
                            'rviz',
                            'visualize_franka.rviz',
                        ]
                    ),
                ],
                output='screen',
            )
        )
    return nodes


# The generate_launch_description function is the entry point (like 'main')
# It is called by the ROS 2 launch system when the launch file is executed.
# via: ros2 launch franka_bringup example.launch.py ARGS...
# This function must return a LaunchDescription object containing nodes to be launched.
# it calls the generate_robot_nodes function to get the list of nodes to be launched.


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot_config_file',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('franka_bringup'), 'config', 'franka.config.yaml']
                ),
                description='Path to the robot configuration file to load',
            ),
            OpaqueFunction(function=generate_robot_nodes),
        ]
    )
