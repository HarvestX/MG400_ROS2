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


from pathlib import Path
from typing import Dict
from typing import List
from typing import Tuple

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node


def _load_robot_description(
        xacro_filepath: Path, xacro_options: List[Tuple] = False) -> Dict:
    """Load robot description."""
    if 'xacro' in str(xacro_filepath):
        params = []
        if xacro_options:
            for xacro_option in xacro_options:
                params.append(' {}:='.format(
                    xacro_option[0]))
                params.append(xacro_option[1])
        command = [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            str(xacro_filepath)]
        robot_description_content = Command(command=(command + params))
    else:
        try:
            with open(str(xacro_filepath), 'r') as file:
                robot_description_content = file
        except EnvironmentError:
            exit(1)

    return {'robot_description': robot_description_content}


def generate_launch_description():
    """Launch rviz display."""
    this_package_path = get_package_share_path('mg400_bringup')
    ns_arg = DeclareLaunchArgument(
        'namespace', default_value=TextSubstitution(text='mg400'),
        description='Set the robot resource namespace.')
    ns = LaunchConfiguration('namespace')
    joy_arg = DeclareLaunchArgument(
        'joy',
        default_value='false',
        description='Determines if joy.launch is called.')
    joy = LaunchConfiguration('joy')
    ip_address_arg = DeclareLaunchArgument(
        'ip_address', default_value=TextSubstitution(text='192.168.1.6'),
        description='Set the ip address to connect')
    ip_address = LaunchConfiguration('ip_address')

    mg400_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(this_package_path
                / 'launch'
                / 'mg400.launch.py')),
        launch_arguments=[
            ('namespace', ns),
            ('ip_address', ip_address),
        ])

    workspace_visible_arg = DeclareLaunchArgument(
        'workspace_visible',
        default_value=TextSubstitution(text='False'),
        description='true : MG400 workspace is visible in rviz')

    workspace_visible = LaunchConfiguration(
        'workspace_visible', default='False')
    xacro_filepath_ = get_package_share_path('mg400_description') / 'urdf' / 'mg400.urdf.xacro'
    robot_description = _load_robot_description(
        xacro_filepath=xacro_filepath_,
        xacro_options={
            'workspace_visible': workspace_visible,
        }.items())

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=ns,
        output='log',
        parameters=[robot_description])

    joy_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(this_package_path
                / 'launch'
                / 'joy.launch.py')),
        condition=IfCondition(joy),
        launch_arguments=[
            ('namespace', ns)
        ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=[
            '-d', str(get_package_share_path('mg400_bringup') /
                      'rviz' / 'mg400.rviz'),
            '--ros-args', '--log-level', 'error'
        ])

    ld = LaunchDescription()

    ld.add_action(ns_arg)
    ld.add_action(joy_arg)
    ld.add_action(ip_address_arg)
    ld.add_action(workspace_visible_arg)

    ld.add_action(mg400_node)
    ld.add_action(rsp_node)
    ld.add_action(joy_node)
    ld.add_action(rviz_node)

    return ld
