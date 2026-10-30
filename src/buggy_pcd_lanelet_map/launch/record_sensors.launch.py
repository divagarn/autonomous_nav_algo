"""Only the sensor drivers, nothing else: to RECORD a bag on the real buggy (the "Record bag" tab of ./buggy app).

  VLP-16 (UDP 2368) -> /velodyne_points      velodyne_static_tf -> /tf_static
  ECU feedback (serial, READ ONLY) -> /wheel_uart      CHCNAV GNSS (UDP) -> /chcnav/devpvt

No localization, no map, no driving: nothing here writes to the ECU, the buggy cannot move.
  ros2 launch buggy_pcd_lanelet_map record_sensors.launch.py [gnss:=true] [wheel:=true] [gnss_udp_port:=7531]
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from buggy_pcd_lanelet_map.paths import config_files


def launch_setup(context):
    cfg = config_files('')
    actions = [
        Node(package='velodyne_driver', executable='velodyne_driver_node', name='velodyne_driver_node',
             output='screen',
             parameters=[{'device_ip': '', 'gps_time': False, 'time_offset': 0.0, 'enabled': True,
                          'read_once': False, 'read_fast': False, 'repeat_delay': 0.0, 'frame_id': 'velodyne',
                          'model': 'VLP16', 'rpm': 1200.0, 'port': 2368, 'timestamp_first_packet': False}]),
        Node(package='velodyne_pointcloud', executable='velodyne_transform_node', name='velodyne_transform_node',
             output='screen',
             parameters=[{'calibration': os.path.join(get_package_share_directory('velodyne_pointcloud'),
                                                      'params', 'VLP16_hires_db.yaml'),
                          'model': 'VLP16', 'min_range': 0.4, 'max_range': 130.0, 'view_direction': 0.0,
                          'fixed_frame': '', 'target_frame': '', 'organize_cloud': True}]),
        Node(package='buggy_core', executable='velodyne_static_tf', name='velodyne_static_tf',
             output='screen', parameters=[*cfg]),
    ]
    if LaunchConfiguration('wheel').perform(context).lower() == 'true':
        port = LaunchConfiguration('feedback_port').perform(context)
        actions.append(Node(package='buggy_core', executable='uart_bridge', name='uart_bridge', output='screen',
                            parameters=[*cfg, *([{'feedback_port': port}] if port else [])]))
    if LaunchConfiguration('gnss').perform(context).lower() == 'true':
        actions.append(Node(package='chcnav', executable='HcCgiProtocolProcessNode', name='hc_topic_driver',
                            output='screen'))
        actions.append(Node(package='chcnav', executable='HcMsgParserLaunchNode', name='udp_gnss', output='screen',
                            parameters=[{'type': 'udp', 'rate': 1000,
                                         'port': int(LaunchConfiguration('gnss_udp_port').perform(context))}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gnss', default_value='true', description='CHCNAV GNSS driver (/chcnav/devpvt)'),
        DeclareLaunchArgument('gnss_udp_port', default_value='7531'),
        DeclareLaunchArgument('wheel', default_value='true', description='ECU feedback (/wheel_uart), read only'),
        DeclareLaunchArgument('feedback_port', default_value='', description='serial port; empty = buggy_params.yaml'),
        OpaqueFunction(function=launch_setup),
    ])
