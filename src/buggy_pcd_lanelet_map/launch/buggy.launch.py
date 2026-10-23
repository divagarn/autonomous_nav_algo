"""Top-level startup (like autoware_launch): the map stack with the buggy's own
RViz layout (rviz/buggy.rviz).

    ros2 launch buggy_pcd_lanelet_map buggy.launch.py [map_path:=<dir>] [rviz:=false]
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
        DeclareLaunchArgument('map_path', default_value='new_delhi_map'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('avoidance', default_value='true',
                              description='go around an obstacle that blocks the lane for >5 s'),
        DeclareLaunchArgument('core', default_value='python', description='python | cpp'),
        DeclareLaunchArgument('params', default_value='', description='your own parameter YAML'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'map.launch.py')),
            launch_arguments={
                'map_path': LaunchConfiguration('map_path'),
                'rviz': LaunchConfiguration('rviz'),
                'avoidance': LaunchConfiguration('avoidance'),
                'core': LaunchConfiguration('core'),
                'params': LaunchConfiguration('params'),
                'rviz_config': os.path.join(pkg, 'rviz', 'buggy.rviz'),
            }.items(),
        ),
    ])
