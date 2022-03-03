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


from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.substitutions.command import Command
from launch.substitutions.find_executable import FindExecutable
from launch.substitutions.path_join_substitution import PathJoinSubstitution

from launch_ros.actions.node import Node


def generate_launch_description():
    """Launch rviz display."""
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            str(
                get_package_share_path('mg400_description') /
                'urdf' /
                'mg400_description.urdf.xacro'
            )
        ],
    )
    robot_description = {'robot_description': robot_description_content}
    rviz_config_file = str(
        get_package_share_path('mg400_bringup') /
        'rviz' /
        'mg400.rviz'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
    )

    mg400_interface_node = Node(
        package='mg400_controller',
        executable='mg400_interface',
        name='mg400_controller',
    )

    return LaunchDescription([
        mg400_interface_node,
        robot_state_publisher_node,
        rviz_node,
    ])
