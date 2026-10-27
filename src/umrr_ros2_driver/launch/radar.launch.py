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

import yaml
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

PACKAGE_NAME = 'umrr_ros2_driver'

# Used when config/buggy_params.yaml has no `radar_driver:` section. Same values as the shipped file.
DEFAULT = {
    'model': 'umrra4', 'can_interface': 'can0', 'baudrate': 500000, 'dev_id': 2, 'sensor_id': 700,
    'frame_id': 'umrr',
    'models': {
        'umrra4': {'model': 'umrra4_can_v1_2_1', 'uifname': 'umrra4_automotive',
                   'uifmajorv': 1, 'uifminorv': 2, 'uifpatchv': 1},
        'umrr9f': {'model': 'umrr9f_can_v2_1_1', 'uifname': 'umrr9f_t169_automotive',
                   'uifmajorv': 2, 'uifminorv': 1, 'uifpatchv': 1},
    },
}


def _settings(user_file):
    """`radar_driver:` of config/buggy_params.yaml, then of the user's own file (params:=FILE)."""
    cfg = {**DEFAULT, 'models': dict(DEFAULT['models'])}
    try:
        from buggy_pcd_lanelet_map.paths import config_files
        files = config_files(user_file)
    except ImportError:
        files = [user_file] if user_file else []
    for f in files:
        sec = dict(((yaml.safe_load(open(f)) or {}).get('radar_driver') or {}).get('ros__parameters') or {})
        models = sec.pop('models', None) or {}
        cfg.update(sec)
        cfg['models'].update(models)
    return cfg


def _launch(context):
    cfg = _settings(LaunchConfiguration('params').perform(context))
    model = LaunchConfiguration('radar_model').perform(context) or cfg['model']
    can = LaunchConfiguration('can_interface').perform(context) or cfg['can_interface']
    if model not in cfg['models']:
        raise RuntimeError(f"radar_model '{model}' is not defined in radar_driver.models "
                           f"(known: {', '.join(sorted(cfg['models']))})")
    m = cfg['models'][model]
    print(f"[radar.launch] radar_model={model} ({m['model']}, {m['uifname']}) on {can}")
    template = os.path.join(get_package_share_directory(PACKAGE_NAME), 'param/radar.params.template.yaml')
    return [Node(
        package=PACKAGE_NAME, executable='smartmicro_radar_node_exe', name='smart_radar',
        parameters=[template, {
            'adapters.adapter_0.hw_iface_name': can,
            'adapters.adapter_0.hw_dev_id': int(cfg['dev_id']),
            'adapters.adapter_0.baudrate': int(cfg['baudrate']),
            'sensors.sensor_0.model': m['model'],
            'sensors.sensor_0.uifname': m['uifname'],
            'sensors.sensor_0.uifmajorv': int(m['uifmajorv']),
            'sensors.sensor_0.uifminorv': int(m['uifminorv']),
            'sensors.sensor_0.uifpatchv': int(m['uifpatchv']),
            'sensors.sensor_0.id': int(cfg['sensor_id']),
            'sensors.sensor_0.dev_id': int(cfg['dev_id']),
            'sensors.sensor_0.frame_id': str(cfg['frame_id']),
        }])]


def generate_launch_description():
    """Generate the launch description."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'can_interface', default_value='',
            description='SocketCAN interface of the radar; empty = radar_driver.can_interface of '
                        'config/buggy_params.yaml'),
        DeclareLaunchArgument(
            'radar_model', default_value='',
            description='radar model, a key of radar_driver.models (umrra4 | umrr9f); empty = '
                        'radar_driver.model of config/buggy_params.yaml'),
        DeclareLaunchArgument(
            'params', default_value='',
            description='your own parameter file (its radar_driver: overrides config/buggy_params.yaml)'),
        OpaqueFunction(function=_launch),
    ])
