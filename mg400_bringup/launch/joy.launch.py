"""Launch joy."""
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
    """Launch joy."""
    hw_type_arg = DeclareLaunchArgument(
        'hw_type',
        default_value=TextSubstitution(text='DualSense'),
        description='Joy controller hardware type.')

    joy_container = ComposableNodeContainer(
        name='joy_container',
        namespace='mg400_joy',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
                ComposableNode(
                    package='joy',
                    plugin='joy::Joy',
                    name='joy',
                    namespace='mg400',
                ),
            ComposableNode(
                    package='mg400_joy',
                    plugin='mg400_joy::MG400JoyInterfaceNode',
                    name='mg400_joy_interface_node',
                    namespace='mg400',
                    parameters=[{
                        'hw_type': LaunchConfiguration('hw_type'),
                    }],
                )
        ])

    ld = LaunchDescription()
    ld.add_action(hw_type_arg)
    ld.add_action(joy_container)

    return ld
