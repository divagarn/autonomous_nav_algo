"""LIVE localization: real VLP-16 + wheel feedback from the ECU, NDT scan
matching against the saved PCD map, EKF fusion (Autoware stack), plus the
map / vehicle / RViz from map.launch.py.

  VLP-16 (UDP 2368) -> /velodyne_points -> lidar_preprocess
        -> /localization/pointcloud_downsampled -> ndt_scan_matcher
  ECU (/dev/ttyUSB0) -> /wheel_uart -> wheel_odometry (/odom, TF OFF)
        -> odom_to_twist -> ekf_localizer <-> ndt_scan_matcher
  ekf_localizer publishes TF map->base_link (base_link = rear axle).
  localization_manager: RViz 2D Pose Estimate -> ndt_align_srv -> seed EKF.

Hardware nodes (velodyne driver, uart_bridge, wheel_odometry,
velodyne_static_tf) come from divagar_test1_ws - source its install first
(buggy_live_launcher.sh does). Nothing in it is modified.

bag_path:=<rosbag2 dir> replays a recording (with /clock, sim time) instead
of using the hardware - for validating the chain without the buggy.

Driving (all OFF by default):
  drive:=true        arms AUTO (publishes /steering_angle + stop/speed flags)
  uart_write:=true   starts uart_writer; loopback:=true (default) only LOGS frames
  loopback:=false    uart_writer really writes to sender_port (the buggy moves!)
Run buggy_live_launcher.sh - it asks for confirmation before a real write.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from buggy_pcd_lanelet_map.paths import config_files, data_dir


def opt(context, **spec):
    """{param: value} only for launch arguments the user really gave; an empty argument means
    'take the value from config/buggy_params.yaml'."""
    out = {}
    for param, (arg, typ) in spec.items():
        v = LaunchConfiguration(arg).perform(context)
        if v != '':
            out[param] = (v.lower() == 'true') if typ is bool else typ(v)
    return out


def launch_setup(context):
    pkg = get_package_share_directory('buggy_pcd_lanelet_map')
    # YAML parameter files: config/buggy_params.yaml, then the user's params:=FILE; launch arguments win
    cfg = config_files(LaunchConfiguration('params').perform(context))
    bag_path = LaunchConfiguration('bag_path').perform(context)
    replay = bool(bag_path)
    gz = LaunchConfiguration('gazebo').perform(context).lower() == 'true'
    sim = replay or gz
    t = {'use_sim_time': sim}
    # runtime core: the C++ nodes of buggy_core replace the Python ones of the same name
    core_pkg = ('buggy_core' if LaunchConfiguration('core').perform(context) == 'cpp'
                else 'buggy_pcd_lanelet_map')

    ndt_params = os.path.join(get_package_share_directory('autoware_ndt_scan_matcher'),
                              'config', 'ndt_scan_matcher.param.yaml')
    ekf_params = os.path.join(get_package_share_directory('autoware_ekf_localizer'),
                              'config', 'ekf_localizer.param.yaml')

    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'map.launch.py')),
        launch_arguments={
            'map_path': LaunchConfiguration('map_path'),
            'rviz': LaunchConfiguration('rviz'),
            'rviz_config': os.path.join(pkg, 'rviz', 'buggy.rviz'),
            'splash': LaunchConfiguration('splash'),
            'splash_sec': LaunchConfiguration('splash_sec'),
            'rviz_maximize': LaunchConfiguration('rviz_maximize'),
            'theme': LaunchConfiguration('theme'),
            'live': 'true',
            'core': LaunchConfiguration('core'),
            'drive': LaunchConfiguration('drive'),
            'obstacle_detection': LaunchConfiguration('obstacle_detection'),
            'radar': LaunchConfiguration('radar'),
            'camera': LaunchConfiguration('camera'),
            'avoidance': LaunchConfiguration('avoidance'),
            'map_display_voxel': LaunchConfiguration('map_display_voxel'),
            'obstacle_memory_sec': LaunchConfiguration('obstacle_memory_sec'),
            'params': LaunchConfiguration('params'),
            'use_sim_time': 'true' if sim else 'false',
            'base_link_at_center': 'true' if gz else 'false',
            'corridor_half_width': LaunchConfiguration('corridor_half_width'),
        }.items())]

    # ---- sensors -----------------------------------------------------
    if not gz:    # Gazebo's URDF already provides base_link -> lidar_mount_link
        actions.append(Node(package=('buggy_core' if core_pkg == 'buggy_core' else 'buggy_perception'),
                            executable='velodyne_static_tf',
                            name='velodyne_static_tf', output='screen', parameters=[*cfg, t]))
    actions.append(Node(package=('buggy_core' if core_pkg == 'buggy_core' else 'buggy_perception'),
                       executable='wheel_odometry',
                       name='wheel_odometry', output='screen',
                       parameters=[*cfg, {'publish_tf': False, **t}]))
    if gz:
        # Gazebo vehicle + sim Velodyne instead of the hardware. The ECU is
        # emulated (ecu_sim) from the frames uart_writer would send, and drives
        # the Gazebo vehicle through sim_ackermann_bridge.
        pkg_desc = get_package_share_directory('buggy_description')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_desc, 'launch', 'gazebo.launch.py')),
            launch_arguments={
                'world': os.path.join(pkg_desc, 'worlds', LaunchConfiguration('world_file').perform(context)),
                'spawn_x': LaunchConfiguration('spawn_x'),
                'spawn_y': LaunchConfiguration('spawn_y'),
                'spawn_yaw': LaunchConfiguration('spawn_yaw'),
                'gui': LaunchConfiguration('gui'),
            }.items()))
        actions.append(Node(
            package='buggy_pcd_lanelet_map', executable='sim_ackermann_bridge', name='sim_ackermann_bridge',
            output='screen', parameters=[t],
            remappings=[('/steering_angle', '/sim_ecu/steering_angle'),
                        ('/vehicle_speed', '/sim_ecu/vehicle_speed')]))
        actions.append(Node(package='buggy_pcd_lanelet_map', executable='ecu_sim',
                            name='ecu_sim', output='screen', parameters=[t]))
    elif replay:
        actions.append(TimerAction(period=float(LaunchConfiguration('bag_start_delay').perform(context)),
                                   actions=[ExecuteProcess(
                                       cmd=['ros2', 'bag', 'play', bag_path, '--clock', '-r',
                                            LaunchConfiguration('bag_rate').perform(context)],
                                       name='bag_play', output='screen')]))
    else:
        actions.append(Node(
            package='velodyne_driver', executable='velodyne_driver_node',
            name='velodyne_driver_node', output='screen',
            parameters=[{'device_ip': '', 'gps_time': False, 'time_offset': 0.0,
                         'enabled': True, 'read_once': False, 'read_fast': False,
                         'repeat_delay': 0.0, 'frame_id': 'velodyne', 'model': 'VLP16',
                         'rpm': 1200.0, 'port': 2368, 'timestamp_first_packet': False}]))
        actions.append(Node(
            package='velodyne_pointcloud', executable='velodyne_transform_node',
            name='velodyne_transform_node', output='screen',
            parameters=[{'calibration': os.path.join(
                get_package_share_directory('velodyne_pointcloud'), 'params',
                'VLP16_hires_db.yaml'),
                'model': 'VLP16', 'min_range': 0.4, 'max_range': 130.0,
                'view_direction': 0.0, 'fixed_frame': '', 'target_frame': '',
                'organize_cloud': True}]))
        actions.append(Node(
            package=('buggy_core' if core_pkg == 'buggy_core' else 'buggy_uart'),
            executable='uart_bridge', name='uart_bridge',
            output='screen',
            parameters=[*cfg, opt(context, port=('feedback_port', str))]))

    # ---- localization --------------------------------------------------
    actions.append(Node(
        package=('buggy_core' if core_pkg == 'buggy_core' else 'buggy_pcd_lanelet_map'),
        executable='lidar_preprocess',
        name='lidar_preprocess', output='screen',
        parameters=[*cfg, opt(context, voxel_size=('voxel_size', float), max_range=('max_range', float),
                              max_rate_hz=('ndt_rate_hz', float)), t]))
    actions.append(Node(
        package=('buggy_core' if core_pkg == 'buggy_core' else 'buggy_pcd_lanelet_map'),
        executable='odom_to_twist', name='odom_to_twist',
        output='screen',
        parameters=[*cfg, {'input_topic': '/odom', 'output_topic': '/twist_with_covariance', **t}]))
    actions.append(Node(
        package='autoware_ekf_localizer', executable='autoware_ekf_localizer_node',
        name='ekf_localizer', output='screen', parameters=[ekf_params, t],
        remappings=[
            ('in_pose_with_covariance', '/ndt_pose_with_covariance'),
            ('in_twist_with_covariance', '/twist_with_covariance'),
            ('initialpose', 'initialpose3d'),
            ('trigger_node_srv', '/ekf_localizer/trigger_node'),
            ('ekf_odom', '/ekf_odom'), ('ekf_pose', '/ekf_pose'),
            ('ekf_pose_with_covariance', '/ekf_pose_with_covariance'),
            ('ekf_biased_pose', '/ekf_biased_pose'),
            ('ekf_biased_pose_with_covariance', '/ekf_biased_pose_with_covariance'),
            ('ekf_twist', '/ekf_twist'),
            ('ekf_twist_with_covariance', '/ekf_twist_with_covariance'),
        ]))
    # ndt_scan_matcher is started AFTER the map loaders are up: its map
    # client misses service discovery if it is created before the server
    # (confirmed in autoware_ndt_ws), and never recovers.
    actions.append(TimerAction(period=7.0, actions=[Node(
        package='autoware_ndt_scan_matcher', executable='autoware_ndt_scan_matcher_node',
        name='ndt_scan_matcher', output='screen',
        parameters=[ndt_params, {
            **t,
            # whole map is small (43 MB): load it once
            'dynamic_map_loading.update_distance': 1000.0,
            'dynamic_map_loading.map_radius': 1000.0,
            'dynamic_map_loading.lidar_radius': 200.0,
            # tuned on this map in autoware_ndt_ws (stable scores 1.5-2.2)
            'score_estimation.converged_param_nearest_voxel_transformation_likelihood': 1.4,
            'initial_pose_estimation.particles_num': 30,
            'initial_pose_estimation.n_startup_trials': 15,
        }],
        remappings=[
            ('points_raw', '/localization/pointcloud_downsampled'),
            ('ekf_pose_with_covariance', '/ekf_pose_with_covariance'),
            ('regularization_pose_with_covariance', '/regularization_pose_with_covariance'),
            ('trigger_node_srv', '/ndt_scan_matcher/trigger_node'),
            ('ndt_pose', '/ndt_pose'),
            ('ndt_pose_with_covariance', '/ndt_pose_with_covariance'),
            ('pcd_loader_service', '/map/get_differential_pointcloud_map'),
        ])]))
    actions.append(Node(
        package=('buggy_core' if core_pkg == 'buggy_core' else 'buggy_pcd_lanelet_map'),
        executable='localization_manager',
        name='localization_manager', output='screen', parameters=[*cfg, t]))
    # ---- obstacle detection: ground removal (CPU 'segment' backend, no torch) ->
    # vehicle self-hit filter -> clustering inside the route corridor
    if LaunchConfiguration('obstacle_detection').perform(context).lower() == 'true':
        actions.append(Node(
            package=('buggy_core' if core_pkg == 'buggy_core' else 'buggy_perception'),
            executable='segment_ground', name='segment_ground',
            output='screen', parameters=[*cfg, t],
            additional_env={'SEGMENT_GROUND_METHOD': 'segment'}))
        actions.append(Node(
            package=('buggy_core' if core_pkg == 'buggy_core' else 'buggy_perception'),
            executable='self_hit_filter', name='self_hit_filter',
            output='screen', parameters=[*cfg, t]))
        actions.append(Node(
            package=core_pkg, executable='lidar_obstacle_detector',
            name='lidar_obstacle_detector', output='screen',
            # Gazebo: base_link is the chassis centre, lidar 0.93 m ahead of it
            parameters=[*cfg,
                        {**t, **({'sensor_x': 0.93} if gz else {}),
                         **opt(context, ground_aware=('ground_aware', bool),
                               reject_thin_low=('reject_thin_low', bool),
                               corridor_half_width=('corridor_half_width', float))}]))

    # ---- optional: radar (radar:=true) -----------------------------------------
    # radar_fusion (IMM-tracked radar objects) -> radar_object_bridge -> the obstacle logic.
    # Hardware: umrr_ros2_driver on CAN. Gazebo: radar_sim (synthetic targets). A rosbag has
    # no radar topic, so nothing starts in replay.
    if LaunchConfiguration('radar').perform(context).lower() == 'true':
        if replay:
            print('[live.launch] radar:=true ignored: a rosbag replay has no radar data')
        else:
            if gz:
                actions.append(Node(package='buggy_pcd_lanelet_map', executable='radar_sim',
                                    name='radar_sim', output='screen', parameters=[t]))
            else:
                actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('umrr_ros2_driver'), 'launch',
                                 'radar.launch.py')),
                    launch_arguments={'can_interface': LaunchConfiguration('can_interface')}.items()))
            fusion_params = os.path.join(get_package_share_directory('radar_fusion'),
                                         'config', 'radar_fusion_params.yaml')
            actions.append(Node(
                package='radar_fusion', executable='radar_fusion_node', name='radar_fusion',
                output='screen',
                parameters=[fusion_params, {
                    **t,
                    'output_frame': 'map',        # this stack's global frame (EKF map->base_link)
                    'odom_topic': '/odom',        # wheel_odometry
                    # radar_sim has no Doppler; the real radar does
                    'radar.use_doppler': not gz,
                    'ego_motion.speed_source': 'odom' if gz else 'radar_doppler',
                    # camera association inside radar_fusion needs calibration files; names
                    # come from object_labeler instead
                    'camera.enabled': False}]))
            actions.append(Node(package='buggy_pcd_lanelet_map', executable='radar_object_bridge',
                                name='radar_object_bridge', output='screen', parameters=[t]))

    # ---- optional: camera names (camera:=true) ----------------------------------
    # camera -> YOLOX 18-class detector -> object_labeler gives the LiDAR/radar obstacles the
    # detector's class name instead of "unknown". Hardware: camera_raw_node on camera_device;
    # replay: the bag's /camera/image_raw; Gazebo has no camera model.
    if LaunchConfiguration('camera').perform(context).lower() == 'true':
        if gz:
            print('[live.launch] camera:=true ignored: Gazebo has no camera model')
        else:
            if not replay:
                actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('camera_processing'), 'launch',
                                 'camera_raw.launch.py')),
                    launch_arguments={'device': LaunchConfiguration('camera_device')}.items()))
            actions.append(Node(
                package='camera_processing', executable='yolox_detector_node',
                name='yolox_detector', output='screen',
                parameters=[{'model_path': LaunchConfiguration('detector_model'),
                             'input_topic': '/camera/image_raw',
                             'conf_threshold': 0.3, 'nms_threshold': 0.45, 'num_classes': 18,
                             'decode_grid_stride': False,
                             # keep the laptop CPU for NDT: 2 threads, 5 frames/s
                             'intra_op_threads': ParameterValue(
                                 LaunchConfiguration('detector_threads'), value_type=int),
                             'max_rate_hz': ParameterValue(
                                 LaunchConfiguration('detector_rate_hz'), value_type=float),
                             **t}]))
            actions.append(Node(package='buggy_perception', executable='camera_static_tf',
                                name='camera_static_tf', output='screen', parameters=[t]))
            actions.append(Node(
                package='buggy_pcd_lanelet_map', executable='object_labeler',
                name='object_labeler', output='screen',
                parameters=[{**t, 'debug_image': ParameterValue(
                    LaunchConfiguration('labeler_debug'), value_type=bool)}]))

    # ---- UART write (off unless asked; loopback unless explicitly real) -----
    uart_write = LaunchConfiguration('uart_write').perform(context).lower() == 'true' or gz
    loopback = LaunchConfiguration('loopback').perform(context).lower() != 'false' or gz
    if uart_write:
        if replay and not loopback:
            raise RuntimeError('refusing real UART write while replaying a bag')
        actions.append(Node(
            package=core_pkg, executable='uart_writer', name='uart_writer',
            output='screen',
            parameters=[*cfg, {'loopback': loopback, **t},
                        opt(context, port=('sender_port', str))]))
    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory('buggy_pcd_lanelet_map')
    return LaunchDescription([
        DeclareLaunchArgument('map_path', default_value='new_delhi_map'),
        DeclareLaunchArgument('params', default_value='',
                              description='your own parameter YAML (overrides config/buggy_params.yaml)'),
        DeclareLaunchArgument('feedback_port', default_value='',
                              description='ECU feedback serial port (empty = uart_bridge.port in the YAML)'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('obstacle_detection', default_value='true',
                              description='LiDAR obstacle detection (segment_ground + '
                                          'self_hit_filter + detector); AUTO requires it'),
        DeclareLaunchArgument('drive', default_value='false',
                              description='arm the AUTO button (publishes steering + stop flags)'),
        DeclareLaunchArgument('uart_write', default_value='false',
                              description='start uart_writer (the only serial writer)'),
        DeclareLaunchArgument('loopback', default_value='true',
                              description='uart_writer: true = log frames, write nothing; '
                                          'false = REALLY WRITE to sender_port'),
        DeclareLaunchArgument('sender_port', default_value='',
                              description='ECU command serial port (empty = uart_writer.port in the YAML)'),
        DeclareLaunchArgument('corridor_half_width', default_value='',
                              description='obstacle corridor half width around the route [m]'),
        DeclareLaunchArgument('splash', default_value=os.path.join(pkg_share, 'rviz', 'buggy_splash.png')),
        DeclareLaunchArgument('splash_sec', default_value='5.0'),
        DeclareLaunchArgument('rviz_maximize', default_value='true'),
        DeclareLaunchArgument('theme', default_value=os.path.join(pkg_share, 'rviz', 'buggy_theme.qss')),
        DeclareLaunchArgument('obstacle_memory_sec', default_value='',
                              description='keep a blocking LiDAR/radar obstacle this long after it was '
                                          'last seen (0 = off); covers sensor drop-outs near the bumper'),
        DeclareLaunchArgument('map_display_voxel', default_value='',
                              description='map copy for RViz, one point per voxel [m] (0 = full map)'),
        DeclareLaunchArgument('ground_aware', default_value='',
                              description='obstacle height above the LOCAL ground (slope / pitch safe); '
                                          'false = old flat reference plane'),
        DeclareLaunchArgument('reject_thin_low', default_value='',
                              description='drop clusters that are one thin low beam ring (road bumps); '
                                          'false if you need very low objects such as cones'),
        DeclareLaunchArgument('core', default_value='python',
                              description='python | cpp: run the C++ runtime nodes (uart_writer, '
                                          'lidar_obstacle_detector) from buggy_core instead of the Python ones'),
        DeclareLaunchArgument('avoidance', default_value='false',
                              description='go around an obstacle that blocks the lane for >5 s '
                                          '(leaves the lane centre; off by default on the real buggy)'),
        DeclareLaunchArgument('radar', default_value='false',
                              description='radar_fusion obstacles (hardware: umrr driver on CAN; '
                                          'Gazebo: radar_sim); no effect in a bag replay'),
        DeclareLaunchArgument('can_interface', default_value='can0',
                              description='radar:=true (hardware): SocketCAN interface'),
        DeclareLaunchArgument('camera', default_value='false',
                              description='camera + YOLOX: give obstacles their class name'),
        DeclareLaunchArgument('camera_device', default_value='/dev/video0'),
        DeclareLaunchArgument(
            'detector_model',
            default_value=os.path.join(data_dir(), 'models',
                                       'yolox_s_ti_18class_ti_today_qat.onnx'),
            description='camera:=true: YOLOX 18-class ONNX model'),
        DeclareLaunchArgument('detector_threads', default_value='2',
                              description='camera:=true: CPU threads for the ONNX detector (0 = all)'),
        DeclareLaunchArgument('detector_rate_hz', default_value='5.0',
                              description='camera:=true: max detector frames/s (0 = every frame)'),
        DeclareLaunchArgument('labeler_debug', default_value='false',
                              description='publish /perception/labeler_debug/image (overlay)'),
        DeclareLaunchArgument('gazebo', default_value='false',
                              description='Gazebo vehicle + emulated ECU instead of hardware/bag '
                                          '(use sim_live.launch.py)'),
        DeclareLaunchArgument('world_file', default_value='office_track.world'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='-3.0'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('bag_path', default_value='',
                              description='replay this rosbag2 instead of the hardware'),
        DeclareLaunchArgument('bag_start_delay', default_value='8.0'),
        DeclareLaunchArgument('bag_rate', default_value='1.0',
                              description='replay speed (e.g. 0.3 on a slow machine / with --camera)'),
        DeclareLaunchArgument('voxel_size', default_value=''),          # empty = lidar_preprocess.voxel_size in the YAML
        DeclareLaunchArgument('max_range', default_value=''),           # empty = lidar_preprocess.max_range
        DeclareLaunchArgument('ndt_rate_hz', default_value=''),         # empty = lidar_preprocess.max_rate_hz
        OpaqueFunction(function=launch_setup),
    ])
