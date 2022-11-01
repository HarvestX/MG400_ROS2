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

from pathlib import Path
from typing import Dict

from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import Shutdown
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def _load_robot_description(filepath: Path) -> Dict:
    """Load robot description."""
    if 'xacro' in filepath.suffix:
        robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name='xacro')]),
                ' ',
                str(filepath)
            ],
        )
    else:
        try:
            with open(str(filepath), 'r') as file:
                robot_description_content = file
        except EnvironmentError:
            exit(1)
    return {'robot_description': robot_description_content}


def generate_launch_description():
    """Launch rviz display."""
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='log',
        parameters=[
            _load_robot_description(
                get_package_share_path('mg400_description') /
                'urdf' / 'mg400.urdf.xacro')])
    jsp_node = Node(
        package='mg400_node',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        on_exit=Shutdown())
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=[
            '-d', str(get_package_share_path('mg400_bringup') /
                      'rviz' / 'display.rviz'),
            '--ros-args', '--log-level', 'error'
        ])

    ld = LaunchDescription()
    ld.add_action(rsp_node)
    ld.add_action(jsp_node)
    ld.add_action(rviz_node)
    return ld
