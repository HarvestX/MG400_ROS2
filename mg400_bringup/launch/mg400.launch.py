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
from launch.actions import Shutdown

from launch_ros.actions import Node

from mg400_bringup.node_generator import robot_state_publisher as rsp
from mg400_bringup.node_generator import rviz2


def generate_launch_description():
    """Launch rviz display."""
    namespace: str = 'mg400'

    robot_state_publisher_node: Node = rsp.load_node(
        filename='mg400.urdf.xacro',
        namespace=namespace
    )

    rviz_node: Node = rviz2.load_node('mg400.rviz')

    mg400_interface_node = Node(
        package='mg400_control',
        executable='mg400_control',
        namespace=namespace,
        name='mg400_control',
        on_exit=Shutdown(),
    )

    return LaunchDescription([
        mg400_interface_node,
        robot_state_publisher_node,
        rviz_node,
    ])
