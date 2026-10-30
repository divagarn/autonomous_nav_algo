"""Standalone connectivity check for the real chcnav CGI device: brings up
the exact same driver chain as real_gps.launch.py (nothing else - no
velodyne/wheel_odometry/ekf_node/etc. from real_sensors_bringup.launch.py),
plus two `ros2 topic hz` processes printing live rates for /fix and /imu
straight to the terminal.

If the device is sending data, you'll see periodic "average rate: X.XXX"
blocks for both topics within a couple seconds. If either topic prints
nothing at all, that data isn't arriving - check (in order): the CGI
device's UDP output config actually has HCINSPVATZCB/HCRAWIMUIB enabled
(chcnav/doc/README-en.md section 2.3), udp_port here matches the device's
configured destination port, and the PC's IP is what's entered in the
device's UDP client config (same section).

Usage:
    ros2 launch chcnav check_gps_imu.launch.py
    ros2 launch chcnav check_gps_imu.launch.py udp_port:=7531
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_chcnav = get_package_share_directory('chcnav')

    udp_port_arg = DeclareLaunchArgument(
        'udp_port', default_value='7531',
        description='UDP port the CGI device is configured to send to (chcnav/doc/README-en.md 2.3).')
    rate_arg = DeclareLaunchArgument('rate', default_value='1000')

    real_gps = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_chcnav, 'launch', 'real_gps.launch.py')
        ),
        launch_arguments={
            'udp_port': LaunchConfiguration('udp_port'),
            'rate': LaunchConfiguration('rate'),
        }.items(),
    )

    check_fix_hz = ExecuteProcess(
        cmd=['ros2', 'topic', 'hz', '/fix'],
        name='check_fix_hz',
        output='screen',
    )

    check_imu_hz = ExecuteProcess(
        cmd=['ros2', 'topic', 'hz', '/imu'],
        name='check_imu_hz',
        output='screen',
    )

    return LaunchDescription([
        udp_port_arg,
        rate_arg,
        real_gps,
        check_fix_hz,
        check_imu_hz,
    ])
