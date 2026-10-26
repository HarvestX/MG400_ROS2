"""Display robot joint states."""

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
from launch.actions import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch rviz display."""
    this_package_path = FindPackageShare('mg400_bringup')

    # Declare launch arguments
    arg_workspace_visible = DeclareLaunchArgument(
        'workspace_visible',
        default_value=TextSubstitution(text='False'),
        description='true : MG400 workspace is visible in rviz',
    )

    # Set launch configuration
    workspace_visible = LaunchConfiguration('workspace_visible', default='False')

    # Create nodes
    rsp_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([this_package_path, 'launch', 'rsp.launch.py'])]
        ),
        launch_arguments=[('workspace_visible', workspace_visible)],
    )

    jsp_node = Node(
        package='mg400_node',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        on_exit=Shutdown(),
    )

    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([this_package_path, 'launch', 'rviz.launch.py'])]
        ),
        launch_arguments=[
            ('package_name', 'mg400_bringup'),
            ('config_dir', 'rviz'),
            ('rviz_config', 'display.rviz'),
        ],
    )

    # Create launch description
    ld = LaunchDescription()
    # Add arguments
    ld.add_action(arg_workspace_visible)
    # Add nodes
    ld.add_action(rsp_node)
    ld.add_action(jsp_node)
    ld.add_action(rviz_node)
    return ld
