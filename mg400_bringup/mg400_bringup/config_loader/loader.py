"""Load config."""
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

from typing import Dict

from ament_index_python.packages import get_package_share_path

from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution
)


def load_robot_description(filename: str) -> Dict:
    """Load robot description."""
    filepath = get_package_share_path(
        'mg400_description') / 'urdf' / filename
    if 'xacro' in filename:
        robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name='xacro')]),
                ' ',
                str(filepath)
            ],
        )
    else:
        try:
            with open(str(filepath), 'r') as file:
                robot_description_content = file
        except EnvironmentError:
            exit(1)
    return {'robot_description': robot_description_content}


def load_rviz2(filename: str) -> str:
    """Load rviz2 configuration."""
    return str(
        get_package_share_path('mg400_bringup') /
        'rviz' /
        filename)
