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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch rviz display."""
    ns_arg = DeclareLaunchArgument(
        'namespace', default_value=TextSubstitution(text=''))
    ns = LaunchConfiguration('namespace')
    ip_address_arg = DeclareLaunchArgument(
        'ip_address', default_value=TextSubstitution(text='192.168.1.6'))
    ip_address = LaunchConfiguration('ip_address')
    service_level_arg = DeclareLaunchArgument(
        'service_level',
        default_value='1',
        description='Determine the command level that '
        'can be called from the service.')
    service_level = LaunchConfiguration('service_level')

    mg400_container = ComposableNodeContainer(
        name='mg400_container',
        namespace=ns,
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='mg400_node',
                plugin='mg400_node::ServiceNode',
                name='mg400_service_node',
                namespace=ns,
                parameters=[{
                    'ip_address': ip_address,
                    'service_level': service_level,
                }],
            ),
            ComposableNode(
                package='mg400_node',
                plugin='mg400_node::ActionNode',
                name='mg400_action_node',
                namespace=ns,), ],
    )

    ld = LaunchDescription()

    ld.add_action(ns_arg)
    ld.add_action(ip_address_arg)
    ld.add_action(service_level_arg)

    ld.add_action(mg400_container)

    return ld
