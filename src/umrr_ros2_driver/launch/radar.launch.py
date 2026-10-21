# Copyright (c) 2021, s.m.s, smart microwave sensors GmbH, Brunswick, Germany
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

import os
import launch_ros

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

PACKAGE_NAME = 'umrr_ros2_driver'


def generate_launch_description():
    """Generate the launch description."""

    radar__params = os.path.join(
           get_package_share_directory(PACKAGE_NAME), 'param/radar.params.template.yaml')

    can_interface_arg = DeclareLaunchArgument(
        'can_interface', default_value='can0',
        description='SocketCAN interface carrying the Smartmicro radar traffic (e.g. can0, '
                    'can3, can4) - overrides radar.params.template.yaml\'s own '
                    'adapters.adapter_0.hw_iface_name so a host whose CAN interface enumerates '
                    'differently does not need that file edited.')

    radar_node = Node(
        package=PACKAGE_NAME,
        executable='smartmicro_radar_node_exe',
        name='smart_radar',
        parameters=[radar__params, {
            'adapters.adapter_0.hw_iface_name': LaunchConfiguration('can_interface'),
        }]
    )
    return LaunchDescription([
        can_interface_arg,
        radar_node,
    ])

    
