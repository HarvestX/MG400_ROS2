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

from typing import List

from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node

from mg400_bringup.config_loader import loader as cl


def generate_launch_description():
    """Launch rviz display."""
    nodes: List[Node] = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='log',
            parameters=[
                cl.load_robot_description('mg400.urdf.xacro')
            ]
        ),
        Node(
            package='mg400_node',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            on_exit=Shutdown(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', cl.load_rviz2('display.rviz')]
        ),
    ]

    return LaunchDescription(nodes)
