"""Autoware-style map bring-up from a single map folder:

  <map_path>/pointcloud_map.pcd
  <map_path>/lanelet2_map.osm
  <map_path>/map_projector_info.yaml
  <map_path>/pointcloud_map_metadata.yaml   (optional, enables partial load)

  map_projection_loader -> /map/map_projector_info
  lanelet2_map_loader   -> /map/vector_map          (HADMapBin, for planners)
  pointcloud_map_loader -> /map/pointcloud_map      (whole map, transient local)
                        -> /map/get_partial_pointcloud_map, ... services
  lanelet_marker_publisher -> /map/vector_map_marker (RViz)
  initial_pose_vehicle  /initialpose (RViz 2D Pose Estimate) -> TF map->base_link
                        + vehicle model on /vehicle/marker
  route_planner         RViz 2D Goal Pose (/goal_pose) -> lane route
                        /planning/route_path, /planning/route_marker

lanelet2_map_loader sits idle until /map/map_projector_info exists, so it is
started 2 s after the projection loader.

    ros2 launch buggy_pcd_lanelet_map map.launch.py
    ros2 launch buggy_pcd_lanelet_map map.launch.py map_path:=/path/to/map rviz:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from buggy_pcd_lanelet_map.paths import resolve_map


def launch_setup(context):
    map_path = resolve_map(LaunchConfiguration('map_path').perform(context))
    pcd = os.path.join(map_path, 'pointcloud_map.pcd')
    osm = os.path.join(map_path, 'lanelet2_map.osm')
    projector = os.path.join(map_path, 'map_projector_info.yaml')
    metadata = os.path.join(map_path, 'pointcloud_map_metadata.yaml')
    for f in (pcd, osm, projector):
        if not os.path.exists(f):
            raise RuntimeError(f'map file missing: {f}')
    has_metadata = os.path.exists(metadata)

    projection_loader = Node(
        package='autoware_map_projection_loader',
        executable='autoware_map_projection_loader_node',
        name='map_projection_loader',
        output='screen',
        parameters=[{'map_projector_info_path': projector, 'lanelet2_map_path': osm}],
    )

    lanelet2_loader = TimerAction(period=2.0, actions=[Node(
        package='autoware_map_loader',
        executable='autoware_lanelet2_map_loader',
        name='lanelet2_map_loader',
        output='screen',
        parameters=[{
            'lanelet2_map_path': osm,
            'allow_unsupported_version': True,
            'center_line_resolution': 5.0,
            'use_waypoints': True,
        }],
        remappings=[('output/lanelet2_map', '/map/vector_map')],
    )])

    pcd_loader_params = {
        'enable_whole_load': True,
        'enable_downsampled_whole_load': False,
        'enable_partial_load': has_metadata,
        'enable_selected_load': False,
        'leaf_size': 3.0,
        'pcd_paths_or_directory': [pcd],
        # statically typed in the node: must be set even when unused
        'pcd_metadata_path': metadata,
    }
    pcd_loader = Node(
        package='autoware_map_loader',
        executable='autoware_pointcloud_map_loader',
        name='pointcloud_map_loader',
        output='screen',
        parameters=[pcd_loader_params],
        remappings=[
            ('output/pointcloud_map', '/map/pointcloud_map'),
            ('service/get_partial_pcd_map', '/map/get_partial_pointcloud_map'),
            ('service/get_selected_pcd_map', '/map/get_selected_pointcloud_map'),
            ('service/get_differential_pcd_map', '/map/get_differential_pointcloud_map'),
        ],
    )

    live = ParameterValue(LaunchConfiguration('live'), value_type=bool)
    sim_time = {'use_sim_time': ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)}

    marker_pub = Node(
        package='buggy_pcd_lanelet_map',
        executable='lanelet_marker_publisher',
        name='lanelet_marker_publisher',
        output='screen',
        parameters=[{'lanelet2_map_path': osm}, sim_time],
    )

    vehicle = Node(
        package='buggy_pcd_lanelet_map',
        executable='initial_pose_vehicle',
        name='initial_pose_vehicle',
        output='screen',
        parameters=[{'live': live,
                     'base_link_at_center': ParameterValue(
                         LaunchConfiguration('base_link_at_center'), value_type=bool)},
                    sim_time],
    )

    route_planner = Node(
        package='buggy_pcd_lanelet_map',
        executable='route_planner',
        name='route_planner',
        output='screen',
        parameters=[{'lanelet2_map_path': osm}, sim_time],
    )

    operation_manager = Node(
        package='buggy_pcd_lanelet_map',
        executable='operation_manager',
        name='operation_manager',
        output='screen',
        parameters=[{'live': live,
                     'base_link_at_center': ParameterValue(
                         LaunchConfiguration('base_link_at_center'), value_type=bool),
                     'avoid_enabled': ParameterValue(
                         LaunchConfiguration('avoidance'), value_type=bool),
                     'obstacle_memory_sec': ParameterValue(
                         LaunchConfiguration('obstacle_memory_sec'), value_type=float),
                     'radar_enabled': ParameterValue(
                         LaunchConfiguration('radar'), value_type=bool),
                     'camera_enabled': ParameterValue(
                         LaunchConfiguration('camera'), value_type=bool),
                     'corridor_half_width': ParameterValue(
                         LaunchConfiguration('corridor_half_width'), value_type=float),
                     'drive_enabled': ParameterValue(LaunchConfiguration('drive'),
                                                     value_type=bool),
                     'require_obstacle_detector': ParameterValue(
                         LaunchConfiguration('obstacle_detection'), value_type=bool)},
                    sim_time],
    )

    dummy_objects = Node(
        package='buggy_pcd_lanelet_map',
        executable='dummy_objects',
        name='dummy_objects',
        output='screen',
        parameters=[sim_time],
    )

    map_display = Node(
        package='buggy_pcd_lanelet_map',
        executable='map_display_cloud',
        name='map_display_cloud',
        output='screen',
        parameters=[{'voxel_size': ParameterValue(LaunchConfiguration('map_display_voxel'),
                                                  value_type=float)}, sim_time],
    )

    # splash screen while RViz loads (rviz2 -s) and the royal-blue Qt theme (rviz2 -stylesheet; Qt
    # itself handles that flag). splash:=none / theme:=none switch them off.
    splash = LaunchConfiguration('splash').perform(context)
    theme = LaunchConfiguration('theme').perform(context)
    # RViz always shows ITS OWN splash (the RViz logo) unless given one: a 1x1 transparent image makes
    # it invisible, so only our splash is ever shown.
    rviz_args = ['-d', LaunchConfiguration('rviz_config'), '-s',
                 os.path.join(get_package_share_directory('buggy_pcd_lanelet_map'), 'rviz', 'no_splash.png')]
    splash_node = None
    if splash not in ('', 'none') and os.path.isfile(splash):
        # splash window for splash_sec; RViz is started only when it has closed (see the return below),
        # so the RViz main page is what appears after the splash. (RViz's own -s splash closes the
        # moment RViz is loaded and leaves its window next to it.)
        splash_node = Node(package='buggy_pcd_lanelet_map', executable='splash_screen',
                           name='splash_screen', output='screen',
                           arguments=[splash, LaunchConfiguration('splash_sec').perform(context)],
                           condition=IfCondition(LaunchConfiguration('rviz')))
    if theme not in ('', 'none') and os.path.isfile(theme):
        rviz_args += ['-stylesheet', theme]
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=rviz_args,
        parameters=[sim_time],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    # RViz comes up maximised (resizable: restore it with the title bar) - rviz_maximize:=false keeps the saved size
    if LaunchConfiguration('rviz_maximize').perform(context).lower() == 'true':
        rviz = [rviz, Node(package='buggy_pcd_lanelet_map', executable='rviz_maximize',
                           name='rviz_maximize', output='screen',
                           condition=IfCondition(LaunchConfiguration('rviz')))]
    else:
        rviz = [rviz]
    nodes = [projection_loader, lanelet2_loader, pcd_loader, marker_pub, vehicle, route_planner,
             map_display, operation_manager, dummy_objects]
    if LaunchConfiguration('avoidance').perform(context).lower() == 'true':
        # detour planner: lanes + (on the real/Gazebo stack) LiDAR free space
        nodes.append(Node(
            package='buggy_pcd_lanelet_map', executable='avoidance_planner',
            name='avoidance_planner', output='screen',
            parameters=[{'lanelet2_map_path': osm,
                         'use_sensors': (LaunchConfiguration('live').perform(context).lower() == 'true'
                                         if LaunchConfiguration('avoid_use_sensors').perform(context) == 'auto'
                                         else LaunchConfiguration('avoid_use_sensors').perform(context).lower() == 'true')},
                        sim_time]))
    if splash_node is None:
        return nodes + rviz
    # the splash exits (timer, Esc, or no display) -> RViz starts. If rviz:=false the splash is skipped
    # by its condition and RViz by its own, so nothing is left waiting.
    return nodes + [splash_node, RegisterEventHandler(OnProcessExit(
        target_action=splash_node, on_exit=rviz))]


def generate_launch_description():
    pkg = get_package_share_directory('buggy_pcd_lanelet_map')
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_path', default_value='new_delhi_map',
            description='Map NAME (folder under <data>/maps) or full path of a folder with '
                        'pointcloud_map.pcd, lanelet2_map.osm, map_projector_info.yaml'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('live', default_value='false',
                              description='true: pose from NDT localization (no click TF/sim)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('obstacle_memory_sec', default_value='4.0',
                              description='keep a blocking sensor obstacle this long after it was last seen'),
        DeclareLaunchArgument('map_display_voxel', default_value='0.3',
                              description='RViz shows a copy of the map thinned to one point per this '
                                          'voxel [m] (0 = full map); stops the map shimmering'),
        DeclareLaunchArgument('avoid_use_sensors', default_value='auto',
                              description='detour planner uses the LiDAR free space: auto = in live mode only'),
        DeclareLaunchArgument('avoidance', default_value='false',
                              description='go around an obstacle that blocks the lane for >5 s'),
        DeclareLaunchArgument('radar', default_value='false'),
        DeclareLaunchArgument('camera', default_value='false'),
        DeclareLaunchArgument('corridor_half_width', default_value='1.25',
                              description='obstacles within this distance of the route centre '
                                          'line stop the vehicle (vehicle half width 0.72 + clearance)'),
        DeclareLaunchArgument('base_link_at_center', default_value='false',
                              description='Gazebo URDF has base_link at the chassis centre; '
                                          'the real stack has it at the rear axle'),
        DeclareLaunchArgument('obstacle_detection', default_value='true',
                              description='live: AUTO is refused/stopped if the LiDAR '
                                          'obstacle detector is not running'),
        DeclareLaunchArgument('drive', default_value='false',
                              description='live only: arm AUTO (steering/stop flags are published)'),
        DeclareLaunchArgument(
            'splash', default_value=os.path.join(pkg, 'rviz', 'buggy_splash.png'),
            description='RViz splash image (none = no splash)'),
        DeclareLaunchArgument(
            'rviz_maximize', default_value='true',
            description='open RViz maximised (still resizable)'),
        DeclareLaunchArgument(
            'splash_sec', default_value='5.0',
            description='minimum time the splash screen stays on screen [s]'),
        DeclareLaunchArgument(
            'theme', default_value=os.path.join(pkg, 'rviz', 'buggy_theme.qss'),
            description='Qt style sheet for RViz, royal blue (none = RViz default look)'),
        DeclareLaunchArgument(
            'rviz_config', default_value=os.path.join(pkg, 'rviz', 'map.rviz')),
        OpaqueFunction(function=launch_setup),
    ])
