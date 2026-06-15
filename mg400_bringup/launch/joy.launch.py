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
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launch joy."""
    ns_arg = DeclareLaunchArgument(
        'namespace',
        default_value=TextSubstitution(text='mg400'),
        description='Set the robot resource namespace.',
    )

    mapping_file_arg = DeclareLaunchArgument(
        'mapping_file',
        default_value=TextSubstitution(text=''),
        description=(
            'Optional YAML file with raw Joy mapping. '
            'Empty value learns mapping at startup.'
        ),
    )

    servo_control_type_arg = DeclareLaunchArgument(
        'servo_control_type',
        default_value=TextSubstitution(text='ServoJ'),
        description='Initial servo command type. Use ServoJ or ServoP.',
    )

    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed_mm_s',
        default_value=TextSubstitution(text='20.0'),
        description='ServoP linear speed in millimeters per second.',
    )

    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed_deg_s',
        default_value=TextSubstitution(text='15.0'),
        description='ServoP angular speed in degrees per second.',
    )

    joint_speed_arg = DeclareLaunchArgument(
        'joint_speed_deg_s',
        default_value=TextSubstitution(text='15.0'),
        description='ServoJ joint speed in degrees per second.',
    )

    controller_axis_threshold_arg = DeclareLaunchArgument(
        'controller_axis_threshold',
        default_value=TextSubstitution(text='0.5'),
        description='Raw Joy axis threshold for controller mapping check.',
    )

    controller_check_timeout_arg = DeclareLaunchArgument(
        'controller_check_timeout_sec',
        default_value=TextSubstitution(text='60.0'),
        description='Startup controller mapping check timeout in seconds.',
    )

    service_timeout_arg = DeclareLaunchArgument(
        'service_timeout_ms',
        default_value=TextSubstitution(text='5000'),
        description='Service wait and response timeout in milliseconds.',
    )

    ns = LaunchConfiguration('namespace')

    joy_container = ComposableNodeContainer(
        name='joy_container',
        namespace='mg400_joy',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='joy',
                plugin='joy::Joy',
                name='joy',
                namespace=ns,
            ),
            ComposableNode(
                package='mg400_joy',
                plugin='mg400_joy::MG400JoyInterfaceNode',
                name='mg400_joy_interface_node',
                namespace=ns,
                parameters=[
                    {
                        'mapping_file': LaunchConfiguration('mapping_file'),
                        'servo_control_type': LaunchConfiguration('servo_control_type'),
                        'linear_speed_mm_s': ParameterValue(
                            LaunchConfiguration('linear_speed_mm_s'), value_type=float
                        ),
                        'angular_speed_deg_s': ParameterValue(
                            LaunchConfiguration('angular_speed_deg_s'), value_type=float
                        ),
                        'joint_speed_deg_s': ParameterValue(
                            LaunchConfiguration('joint_speed_deg_s'), value_type=float
                        ),
                        'controller_axis_threshold': ParameterValue(
                            LaunchConfiguration('controller_axis_threshold'), value_type=float
                        ),
                        'controller_check_timeout_sec': ParameterValue(
                            LaunchConfiguration('controller_check_timeout_sec'), value_type=float
                        ),
                        'service_timeout_ms': ParameterValue(
                            LaunchConfiguration('service_timeout_ms'), value_type=int
                        ),
                    }
                ],
            ),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(ns_arg)
    ld.add_action(mapping_file_arg)
    ld.add_action(servo_control_type_arg)
    ld.add_action(linear_speed_arg)
    ld.add_action(angular_speed_arg)
    ld.add_action(joint_speed_arg)
    ld.add_action(controller_axis_threshold_arg)
    ld.add_action(controller_check_timeout_arg)
    ld.add_action(service_timeout_arg)
    ld.add_action(joy_container)

    return ld
