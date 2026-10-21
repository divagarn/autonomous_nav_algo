from glob import glob

from setuptools import find_packages, setup

package_name = 'buggy_pcd_lanelet_map'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz') + glob('rviz/*.png') + glob('rviz/*.qss')),
        ('share/' + package_name + '/scripts',
            glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='divagar',
    maintainer_email='n.divagar18@gmail.com',
    description='PCD + Lanelet2 map bring-up, Autoware style',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'lanelet_marker_publisher = buggy_pcd_lanelet_map.lanelet_marker_publisher:main',
            'initial_pose_vehicle = buggy_pcd_lanelet_map.initial_pose_vehicle:main',
            'route_planner = buggy_pcd_lanelet_map.route_planner:main',
            'operation_manager = buggy_pcd_lanelet_map.operation_manager:main',
            'dummy_objects = buggy_pcd_lanelet_map.dummy_objects:main',
            'lidar_preprocess = buggy_pcd_lanelet_map.lidar_preprocess:main',
            'odom_to_twist = buggy_pcd_lanelet_map.odom_to_twist:main',
            'localization_manager = buggy_pcd_lanelet_map.localization_manager:main',
            'sim_lane_driver = buggy_pcd_lanelet_map.sim_lane_driver:main',
            'splash_screen = buggy_pcd_lanelet_map.splash_screen:main',
            'rviz_maximize = buggy_pcd_lanelet_map.rviz_maximize:main',
            'radar_sim = buggy_pcd_lanelet_map.radar_sim:main',
            'radar_object_bridge = buggy_pcd_lanelet_map.radar_object_bridge:main',
            'object_labeler = buggy_pcd_lanelet_map.object_labeler:main',
            'avoidance_planner = buggy_pcd_lanelet_map.avoidance_planner:main',
            'map_display_cloud = buggy_pcd_lanelet_map.map_display_cloud:main',
            'sim_ackermann_bridge = buggy_pcd_lanelet_map.sim_ackermann_bridge:main',
            'uart_writer = buggy_pcd_lanelet_map.uart_writer:main',
            'ecu_sim = buggy_pcd_lanelet_map.ecu_sim:main',
            'brake_calibration = buggy_pcd_lanelet_map.brake_calibration:main',
            'lidar_obstacle_detector = buggy_pcd_lanelet_map.lidar_obstacle_detector:main',
            'pointcloud_map_accumulator = buggy_pcd_lanelet_map.pointcloud_map_accumulator:main',
        ],
    },
)
