"""Launch robot simulator controller."""
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

from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description."""
    this_pkg_name = 'mg400_bringup'
    joy_arg = DeclareLaunchArgument(
        'joy',
        default_value='false',
        description='Determines if joy.launch is called.')
    service_level_arg = DeclareLaunchArgument(
        'service_level',
        default_value='1',
        description='Determine the command level that '
        'can be called from the service.')
    joy = LaunchConfiguration('joy')
    service_level = LaunchConfiguration('service_level')
    main_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(get_package_share_path(this_pkg_name) /
                'launch' / 'mg400.launch.py')
        ),
        launch_arguments=[
            ('ip_address', '127.0.0.1'),
            ('joy', joy),
            ('service_level', service_level),
        ])

    ld = LaunchDescription()

    ld.add_action(joy_arg)
    ld.add_action(service_level_arg)

    ld.add_action(main_node)

    return ld
