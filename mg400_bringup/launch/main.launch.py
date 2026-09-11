"""Launch main system."""

# Copyright 2022 HarvestX Inc.
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
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import TextSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch rviz display."""
    this_package_path = FindPackageShare('mg400_bringup')

    # Declare launch arguments
    ns_arg = DeclareLaunchArgument(
        'namespace',
        default_value=TextSubstitution(text='mg400'),
        description='Set the robot resource namespace.',
    )

    joy_arg = DeclareLaunchArgument(
        'joy', default_value='false', description='Determines if joy.launch is called.'
    )

    ip_address_arg = DeclareLaunchArgument(
        'ip_address',
        default_value=TextSubstitution(text='192.168.1.6'),
        description='Set the ip address to connect',
    )

    enable_external_force_estimator_arg = DeclareLaunchArgument(
        'enable_external_force_estimator', default_value=TextSubstitution(text='false')
    )

    workspace_visible_arg = DeclareLaunchArgument(
        'workspace_visible',
        default_value=TextSubstitution(text='False'),
        description='true : MG400 workspace is visible in rviz',
    )

    # Set launch configurations
    ns = LaunchConfiguration('namespace')
    joy = LaunchConfiguration('joy')
    ip_address = LaunchConfiguration('ip_address')
    enable_external_force_estimator = LaunchConfiguration('enable_external_force_estimator')
    workspace_visible = LaunchConfiguration('workspace_visible')

    # Create nodes
    mg400_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([this_package_path, 'launch', 'mg400.launch.py'])]
        ),
        launch_arguments=[
            ('namespace', ns),
            ('ip_address', ip_address),
            ('enable_external_force_estimator', enable_external_force_estimator),
        ],
    )

    joy_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([this_package_path, 'launch', 'joy.launch.py'])]
        ),
        condition=IfCondition(joy),
        launch_arguments=[('namespace', ns)],
    )

    rsp_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([this_package_path, 'launch', 'rsp.launch.py'])]
        ),
        launch_arguments=[('namespace', ns), ('workspace_visible', workspace_visible)],
    )

    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([this_package_path, 'launch', 'rviz.launch.py'])]
        ),
        launch_arguments=[
            ('package_name', 'mg400_bringup'),
            ('config_dir', 'rviz'),
            ('rviz_config', 'mg400.rviz'),
        ],
    )

    # Create launch description
    ld = LaunchDescription()
    # Add arguments
    ld.add_action(ns_arg)
    ld.add_action(joy_arg)
    ld.add_action(ip_address_arg)
    ld.add_action(enable_external_force_estimator_arg)
    ld.add_action(workspace_visible_arg)
    # Add nodes
    ld.add_action(mg400_node)
    ld.add_action(rsp_node)
    ld.add_action(joy_node)
    ld.add_action(rviz_node)

    return ld
