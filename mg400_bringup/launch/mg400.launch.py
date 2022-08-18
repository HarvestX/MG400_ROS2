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

from typing import List

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            Shutdown, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

from mg400_bringup.config_loader import loader as cl


def generate_launch_description():
    """Launch rviz display."""
    namespace: str = "mg400"
    this_pkg = "mg400_bringup"
    launch_args = [
        DeclareLaunchArgument(
            "ip_address", default_value=TextSubstitution(text="192.168.1.6")
        ),
        DeclareLaunchArgument(
            "joy",
            default_value="false",
            description="Determines if joy.launch is called.",
        ),
    ]
    ip_address = LaunchConfiguration("ip_address")
    joy = LaunchConfiguration("joy")

    nodes: List[Node] = [
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", cl.load_rviz2("mg400.rviz")],
        ),
        Node(
            package="mg400_node",
            executable="service_node_exec",
            namespace=namespace,
            name="mg400_service_node",
            on_exit=Shutdown(),
            parameters=[
                {
                    "ip_address": ip_address,
                }
            ],
        ),
        TimerAction(
            period=1.5,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        str(
                            get_package_share_path(this_pkg)
                            / "launch"
                            / "joy.launch.py"
                        ),
                    ),
                    condition=IfCondition(joy),
                ),
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    namespace=namespace,
                    output="log",
                    parameters=[
                        cl.load_robot_description("mg400.urdf.xacro"),
                    ],
                ),
            ],
        ),
    ]

    return LaunchDescription(launch_args + nodes)
