# Copyright 2025 HarvestX Inc. All Rights Reserved.
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

import os
import sys
from pathlib import Path
from typing import Dict
from typing import List
from typing import Tuple

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.substitutions import Command
from launch.actions import DeclareLaunchArgument
from launch.substitutions import FindExecutable
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.append(os.path.dirname(os.path.realpath(__file__)))


def load_robot_description(xacro_filepath: Path, xacro_options: List[Tuple] = None) -> Dict:
    """Load robot description."""
    if xacro_options is None:
        xacro_options = []

    if 'xacro' in str(xacro_filepath):
        params = []
        if xacro_options:
            for xacro_option in xacro_options:
                params.append(f' {xacro_option[0]}:=')
                params.append(xacro_option[1])
        command = [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            str(xacro_filepath),
        ]
        robot_description_content = Command(command + params)
    else:
        try:
            with open(str(xacro_filepath), 'r', encoding='utf-8') as file:
                robot_description_content = file.read()
        except EnvironmentError:
            exit(1)
    return {'robot_description': robot_description_content}


def launch_setup(context, *args, **kwargs):
    use_gui = LaunchConfiguration('use_gui')

    rviz_config_dir = os.path.join(
        get_package_share_directory('mg400_description'), 'rviz', 'display.rviz'
    )
    urdf_filepath = os.path.join(
        get_package_share_path('mg400_description'), 'urdf', 'mg400.urdf.xacro'
    )

    robot_description = load_robot_description(
        xacro_filepath=urdf_filepath,
        xacro_options={
            'prefix': '',
            'use_single_path_arm': 'true',
            'workspace_visible': 'false',
        }.items(),
    )

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': False}],
    )

    node_model = Node(
        name='robot_state_publisher_right',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='',
        output='screen',
        parameters=[robot_description],
    )

    node_jsp = Node(
        name='jsp_node',
        condition=IfCondition(use_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace='',
        parameters=[
            {'use_gui': True},
        ],
    )

    return [
        node_model,
        node_rviz,
        node_jsp,
    ]


def generate_launch_description():
    """generate_launch_description"""
    arg_use_gui = DeclareLaunchArgument(
        'use_gui', default_value='true', description="""Use GUI for Joint State Publisher"""
    )

    ld = LaunchDescription()
    ld.add_action(arg_use_gui)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
