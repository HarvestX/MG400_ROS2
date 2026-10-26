"""mg400_rviz.launch.py."""

# Copyright 2025 HarvestX Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch RViz with configurable config file."""
    # Declare launch arguments
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='mg400.rviz',
        description='RViz configuration file name (without path)',
    )

    package_name_arg = DeclareLaunchArgument(
        'package_name',
        default_value='mg400_bringup',
        description='Package name containing the RViz config',
    )

    config_dir_arg = DeclareLaunchArgument(
        'config_dir',
        default_value='rviz',
        description='Directory name within package containing RViz configs',
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='error',
        description='Log level for RViz (debug, info, warn, error, fatal)',
    )

    # Generate rviz config path
    rviz_config_path = PathJoinSubstitution(
        [
            FindPackageShare(LaunchConfiguration('package_name')),
            LaunchConfiguration('config_dir'),
            LaunchConfiguration('rviz_config'),
        ]
    )

    # Create nodes
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=[
            '-d',
            rviz_config_path,
            '--ros-args',
            '--log-level',
            LaunchConfiguration('log_level'),
        ],
    )

    # Create launch description
    ld = LaunchDescription()
    ld.add_action(rviz_config_arg)
    ld.add_action(package_name_arg)
    ld.add_action(config_dir_arg)
    ld.add_action(log_level_arg)
    ld.add_action(rviz_node)

    return ld
