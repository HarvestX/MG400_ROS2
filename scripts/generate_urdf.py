#!/usr/bin/env python3
"""Generate urdf from xacro."""
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

import argparse
import logging
from pathlib import Path

from ament_index_python.packages import get_package_share_path

import xacro

package_name = 'mg400_description'
logging.basicConfig(level=logging.INFO)


def make_parser() -> argparse.ArgumentParser:
    """Generate argument parser."""
    parser = argparse.ArgumentParser('Generate URDF parser')
    parser.add_argument(
        'name',
        type=str,
        help='Xacro file basename ex. lab_nano')
    return parser


def get_xacro_filename(name: str) -> Path:
    """Get xacro filename."""
    return get_package_share_path(
        package_name
    ) / 'urdf' / '{}.urdf.xacro'.format(name)


def open_xacro_and_save_urdf(
    name: str
) -> None:
    """Open xacro file as xml and save it as urdf."""
    xacro_filename = get_xacro_filename(name)
    urdf = xacro.process_file(
        str(xacro_filename)
    ).toxml()

    with open('{}.urdf'.format(name), 'w') as f:
        f.write(urdf)


def main():
    """Generate urdf from xacro."""
    args = make_parser().parse_args()
    xacro_filename = get_xacro_filename(args.name)
    if not xacro_filename.is_file():
        logging.error('{} does not exist'.format(str(xacro_filename)))
        exit(1)
    print('Generating URDF: {} 🍺'.format(args.name))
    open_xacro_and_save_urdf(args.name)
    print('Done! 🍻')


if __name__ == '__main__':
    main()
