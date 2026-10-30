"""Sim counterpart of real_gps.launch.py, in this same package.

No chcnav code runs here at all - Gazebo Classic ships native GPS
(libgazebo_ros_gps_sensor.so) and IMU (libgazebo_ros_imu_sensor.so) sensor
plugins, wired in buggy_description's buggy.gps_imu.xacro onto the
gps_mount_link/imu_mount_link placeholders, remapped to publish the exact
same topics the real launch's ChcnavFixDemo node publishes: sensor_msgs/
NavSatFix on /fix, sensor_msgs/Imu on /imu. So anything downstream that
consumes GPS/IMU is agnostic to whether real_gps.launch.py or this file
brought those topics up.

This just spawns Gazebo (same as buggy_sensors/sensors_bringup.launch.py's
own gazebo include) - no perception/radar/velodyne nodes, since this file
is scoped to GPS/IMU only. Bring this up standalone to test GPS/IMU in
isolation, or use sensors_bringup.launch.py for the full sim sensor stack
(which spawns the same URDF and therefore already includes GPS/IMU too,
now that buggy.gps_imu.xacro is wired in - no separate node needed there).

Absolute lat/lon values depend on each world file's <spherical_coordinates>
anchor (see buggy_description/worlds/*.world) - currently a (0,0)
placeholder, not tied to a real deployment site.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    pkg_buggy_description = get_package_share_directory('buggy_description')

    world_file_arg = DeclareLaunchArgument(
        'world_file', default_value='rect_loop_track.world',
        description='World file under buggy_description/worlds/.')
    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='5.0')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='5.0')
    spawn_yaw_arg = DeclareLaunchArgument('spawn_yaw', default_value='0.0')
    gui_arg = DeclareLaunchArgument('gui', default_value='true')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_buggy_description, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': PathJoinSubstitution(
                [pkg_buggy_description, 'worlds', LaunchConfiguration('world_file')]),
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
            'gui': LaunchConfiguration('gui'),
        }.items(),
    )

    return LaunchDescription([
        world_file_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_yaw_arg,
        gui_arg,
        gazebo,
    ])
