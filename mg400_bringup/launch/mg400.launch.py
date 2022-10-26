"""Launch robot controller."""
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
from launch.actions import Shutdown
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node

from mg400_bringup.config_loader import loader as cl


def generate_launch_description():
    """Launch rviz display."""
    namespace: str = 'mg400'
    this_pkg = 'mg400_bringup'
    ip_address_arg = DeclareLaunchArgument(
        'ip_address', default_value=TextSubstitution(text='192.168.1.6'))
    joy_arg = DeclareLaunchArgument(
        'joy',
        default_value='false',
        description='Determines if joy.launch is called.')
    service_level_arg = DeclareLaunchArgument(
        'service_level',
        default_value='1',
        description='Determine the command level that can be called from the service.')
    action_level_arg = DeclareLaunchArgument(
        'action_level',
        default_value='2',
        description='Determine the command level that can be called from the action.')
    ip_address = LaunchConfiguration('ip_address')
    joy = LaunchConfiguration('joy')
    service_level = LaunchConfiguration('service_level')
    action_level = LaunchConfiguration('action_level')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', cl.load_rviz2('mg400.rviz')])
    service_node = Node(
        package='mg400_node',
        executable='service_node_exec',
        namespace=namespace,
        name='mg400_service_node',
        on_exit=Shutdown(),
        parameters=[
            {'ip_address': ip_address,
             'service_level': service_level}])
    action_node = Node(
        package='mg400_node',
        executable='action_node_exec',
        namespace=namespace,
        name='mg400_action_node',
        on_exit=Shutdown(),
        parameters=[
            {'ip_address': ip_address,
             'action_level': action_level}])
    joy_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                get_package_share_path(this_pkg)
                / 'launch'
                / 'joy.launch.py'
            ),
        ),
        condition=IfCondition(joy))
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='log',
        parameters=[
            cl.load_robot_description('mg400.urdf.xacro')])
    timer_action = TimerAction(
        period=1.5,
        actions=[
            joy_node,
            rsp_node])

    ld = LaunchDescription()
    ld.add_action(ip_address_arg)
    ld.add_action(joy_arg)
    ld.add_action(service_level_arg)
    ld.add_action(action_level_arg)
    ld.add_action(rviz_node)
    ld.add_action(service_node)
    ld.add_action(action_node)
    ld.add_action(timer_action)

    return ld
