import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('fusion_vehicles'), 'common', 'radar_tracker.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=params),
        DeclareLaunchArgument('radar_topic', default_value='/smart_radar/can_targets_0'),
        Node(
            package='radar_tracker', executable='radar_tracker_node', name='radar_tracker',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'radar_topic': LaunchConfiguration('radar_topic')}]),
    ])
