"""The LIVE stack with the real buggy replaced by Gazebo + an emulated ECU.

  Gazebo office_track.world (same world/spawn as the old `buggy_mapping localize.launch.py`)
    -> sim Velodyne /velodyne_points -> segment_ground / self_hit_filter / detector
    -> lidar_preprocess -> NDT (against the sim PCD from sim_build_map) + EKF
    -> map->base_link -> operation_manager (AUTO, stop-distance logic)
    -> /steering_angle, /final_red_detected, /final_yellow_detected
    -> uart_writer (LOOPBACK: writes no serial port) -> /uart/frame
    -> ecu_sim (assumed start delay / reaction / decel) -> Gazebo vehicle
    -> /wheel_uart feedback -> wheel_odometry -> EKF   (closed loop)

Everything between the sensors and the ECU is the same code as on the buggy.
Use RViz 2D Pose Estimate at the spawn (x=0, y=0, yaw=0), 2D Goal Pose, AUTO.

  ros2 launch buggy_pcd_lanelet_map sim_live.launch.py [gui:=true] [rviz:=false]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('buggy_pcd_lanelet_map')
    return LaunchDescription([
        DeclareLaunchArgument('map_path',
                              default_value='office_track_sim'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('core', default_value='python', description='python | cpp'),
        DeclareLaunchArgument('autoware_map_loaders', default_value='false'),
        DeclareLaunchArgument('params', default_value='', description='your own parameter YAML'),
        DeclareLaunchArgument('avoidance', default_value='true',
                              description='go around an obstacle that blocks the lane for >5 s'),
        DeclareLaunchArgument('radar', default_value='false',
                              description='synthetic radar (radar_sim) + sensor_fusion radar tracks'),
        DeclareLaunchArgument('camera', default_value='false',
                              description='ignored in Gazebo (no camera model)'),
        # this sim track has wall pieces ~0.3 m from the vehicle body; the real default
        # (1.25 m) would hold the vehicle at the first one
        DeclareLaunchArgument('corridor_half_width', default_value='0.85'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'live.launch.py')),
            launch_arguments={
                'gazebo': 'true', 'drive': 'true', 'uart_write': 'true', 'loopback': 'true',
                'map_path': LaunchConfiguration('map_path'),
                'rviz': LaunchConfiguration('rviz'),
                'gui': LaunchConfiguration('gui'),
                'avoidance': LaunchConfiguration('avoidance'),
                'core': LaunchConfiguration('core'),
                'params': LaunchConfiguration('params'),
            'autoware_map_loaders': LaunchConfiguration('autoware_map_loaders'),
                'radar': LaunchConfiguration('radar'),
                'camera': LaunchConfiguration('camera'),
                'corridor_half_width': LaunchConfiguration('corridor_half_width'),
            }.items()),
    ])
