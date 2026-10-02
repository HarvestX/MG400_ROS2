"""Launch MG400 raw telemetry and RViz for parameter identification."""

# Copyright 2026 HarvestX Inc.
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start the driver, robot model and Identify panel."""
    bringup = FindPackageShare('mg400_bringup')
    namespace = LaunchConfiguration('namespace')
    ip_address = LaunchConfiguration('ip_address')
    publish_error_id = LaunchConfiguration('publish_error_id')

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup, 'launch', 'mg400.launch.py'])
        ),
        launch_arguments={
            'namespace': namespace,
            'ip_address': ip_address,
            'publish_error_id': publish_error_id,
            'publish_joint_currents': 'true',
            'publish_end_pose': 'true',
            'enable_external_force_estimator': 'false',
        }.items(),
    )
    robot_model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup, 'launch', 'rsp.launch.py'])
        ),
        launch_arguments={'namespace': namespace}.items(),
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup, 'launch', 'rviz.launch.py'])
        ),
        launch_arguments={
            'namespace': namespace,
            'rviz_config': 'param_identify.rviz',
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='mg400', description='Robot resource namespace.'
        ),
        DeclareLaunchArgument(
            'ip_address', default_value='192.168.1.6', description='MG400 IP address.'
        ),
        DeclareLaunchArgument(
            'publish_error_id', default_value='false',
            description='Automatically query and publish error IDs in ERROR mode.'
        ),
        driver,
        robot_model,
        rviz,
    ])
