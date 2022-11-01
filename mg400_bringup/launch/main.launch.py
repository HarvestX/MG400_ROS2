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

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node

from mg400_bringup.config_loader import loader as cl


def generate_launch_description():
    """Launch rviz display."""
    this_package_path = get_package_share_path('mg400_bringup')
    ns_arg = DeclareLaunchArgument(
        'namespace', default_value=TextSubstitution(text='mg400'))
    ns = LaunchConfiguration('namespace')
    joy_arg = DeclareLaunchArgument(
        'joy',
        default_value='false',
        description='Determines if joy.launch is called.')
    joy = LaunchConfiguration('joy')
    ip_address_arg = DeclareLaunchArgument(
        'ip_address', default_value=TextSubstitution(text='192.168.1.6'))
    ip_address = LaunchConfiguration('ip_address')
    service_level_arg = DeclareLaunchArgument(
        'service_level',
        default_value='1',
        description='Determine the command level that '
        'can be called from the service.')
    service_level = LaunchConfiguration('service_level')

    mg400_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(this_package_path
                / 'launch'
                / 'mg400.launch.py')),
        launch_arguments=[
            ('namespace', ns),
            ('ip_address', ip_address),
            ('service_level', service_level),
        ])

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
            '-d', cl.load_rviz2('mg400.rviz'),
            '--ros-args', '--log-level', 'error'
        ])

    ld = LaunchDescription()

    ld.add_action(ns_arg)
    ld.add_action(joy_arg)
    ld.add_action(ip_address_arg)
    ld.add_action(service_level_arg)

    ld.add_action(mg400_node)
    ld.add_action(joy_node)
    ld.add_action(rviz_node)

    return ld
