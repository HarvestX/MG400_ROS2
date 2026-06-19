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
from launch.actions import Shutdown
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Launch rviz display."""
    # DeclareLaunchArguments
    ns_arg = DeclareLaunchArgument('namespace', default_value=TextSubstitution(text=''))
    enable_external_force_pub_arg = DeclareLaunchArgument(
        'enable_external_force_pub', default_value=TextSubstitution(text='false')
    )
    ip_address_arg = DeclareLaunchArgument(
        'ip_address', default_value=TextSubstitution(text='192.168.1.6')
    )

    # Set launch configurations
    ns = LaunchConfiguration('namespace')
    ip_address = LaunchConfiguration('ip_address')
    enable_external_force_pub = LaunchConfiguration('enable_external_force_pub')

    # Create nodes
    mg400_node = Node(
        package='mg400_node',
        executable='mg400_node_exec',
        name='mg400_node',
        namespace=ns,
        parameters=[
            {
                'ip_address': ip_address,
                'enable_external_force_pub': enable_external_force_pub,
            }
        ],
        on_exit=Shutdown(),
    )

    # Create launch description
    ld = LaunchDescription()
    # Add arguments
    ld.add_action(ns_arg)
    ld.add_action(ip_address_arg)
    ld.add_action(enable_external_force_pub_arg)
    # Add nodes
    ld.add_action(mg400_node)

    return ld
