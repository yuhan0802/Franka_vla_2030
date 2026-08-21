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

import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def create_rviz_launch_arguments() -> List[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument(
            'robot_type',
            default_value='tmrv0_2',
            description='ID of the robot type for visualization'
        ),
    ]


def create_rviz_node() -> IncludeLaunchDescription:
    rviz_config = os.path.join(
        get_package_share_directory('franka_mobile_sensors'),
        'rviz',
        'tmr_sensors.rviz'
    )

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('franka_description'),
                'launch',
                'visualize_franka.launch.py'
            )
        ]),
        launch_arguments={
            'rviz_file': rviz_config,
            'robot_type': LaunchConfiguration('robot_type'),
        }.items()
    )


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        *create_rviz_launch_arguments(),
        create_rviz_node()
    ])


if __name__ == '__main__':
    generate_launch_description()