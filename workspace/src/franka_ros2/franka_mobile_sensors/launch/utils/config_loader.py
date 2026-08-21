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
import yaml
from typing import Dict, Optional
from ament_index_python.packages import get_package_share_directory


def load_yaml_config(config_name: str, subdirectory: Optional[str] = None) -> Dict:
    """
    This utility function handles the common pattern of loading YAML files from
    the franka_mobile_sensors package configuration directory.
    
    Args:
        config_name: Name of the config file (with or without .yaml extension)
        subdirectory: Optional subdirectory within config/ (e.g., 'cameras', 'lidars')
        
    Returns:
        Dictionary containing the parsed YAML data
    """
    if not config_name.endswith('.yaml'):
        config_name += '.yaml'
    
    package_dir = get_package_share_directory('franka_mobile_sensors')
    if subdirectory:
        config_path = os.path.join(package_dir, 'config', subdirectory, config_name)
    else:
        config_path = os.path.join(package_dir, 'config', config_name)

    try:
        with open(config_path, 'r') as file:
            config_data = yaml.safe_load(file)
    except yaml.YAMLError as e:
        raise yaml.YAMLError(f"Failed to parse YAML file {config_name}: {e}")
    
    return config_data if config_data else {}