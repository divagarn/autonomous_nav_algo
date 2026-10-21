"""Build a PCD of the Gazebo world by driving the sim buggy along a lane.

Sim stand-in for "real vehicle drives, VLP-16 records": Gazebo (office_track.world,
spawn 0,-3,0) -> sim Velodyne /velodyne_points -> pointcloud_map_accumulator in the odom
frame (= lanelet "map" frame, which starts at the spawn pose) -> PCD.

  ./buggy sim-map                      # drives the lane, then:
  ./buggy sim-map --save               # (2nd terminal) writes the PCD
Output: <data>/maps/<out_name>/pointcloud_map.pcd
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from buggy_pcd_lanelet_map.paths import maps_dir, resolve_map


def launch_setup(context):
    pkg_desc = get_package_share_directory('buggy_description')
    out_dir = resolve_map(LaunchConfiguration('out_name').perform(context))
    os.makedirs(out_dir, exist_ok=True)
    lane = os.path.join(resolve_map(LaunchConfiguration('lane_map').perform(context)),
                        'lanelet2_map.osm')
    world = os.path.join(pkg_desc, 'worlds', LaunchConfiguration('world_file').perform(context))
    t = {'use_sim_time': True}
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_desc, 'launch', 'gazebo.launch.py')),
            launch_arguments={
                'world': world,
                'spawn_x': LaunchConfiguration('spawn_x'),
                'spawn_y': LaunchConfiguration('spawn_y'),
                'spawn_yaw': LaunchConfiguration('spawn_yaw'),
                'gui': LaunchConfiguration('gui'),
            }.items()),
        # ackermann_steering_controller's odom->base_link TF -> /tf
        Node(package='buggy_perception', executable='tf_odom_relay', name='tf_odom_relay',
             output='screen', parameters=[t]),
        Node(package='buggy_pcd_lanelet_map', executable='sim_ackermann_bridge',
             name='sim_ackermann_bridge', output='screen', parameters=[t]),
        Node(package='buggy_pcd_lanelet_map', executable='sim_lane_driver',
             name='sim_lane_driver', output='screen',
             parameters=[{'lanelet2_map_path': lane,
                          'speed_kmh': LaunchConfiguration('speed_kmh'), **t}]),
        Node(package='buggy_pcd_lanelet_map', executable='pointcloud_map_accumulator',
             name='pointcloud_map_accumulator', output='screen',
             parameters=[{'input_topic': '/velodyne_points', 'target_frame': 'odom',
                          'voxel_size': 0.2, 'max_range': 25.0,
                          'output_file': os.path.join(out_dir, 'pointcloud_map.pcd'), **t}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world_file', default_value='office_track.world'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='-3.0'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('speed_kmh', default_value='6.0'),
        DeclareLaunchArgument('lane_map', default_value='office_track_sim',
                              description='map NAME whose lanelet2_map.osm lane is driven'),
        DeclareLaunchArgument('out_name', default_value='office_track_sim',
                              description='map NAME (folder under <data>/maps) to write the PCD into'),
        OpaqueFunction(function=launch_setup),
    ])
