# buggy_core: C++ runtime core

One folder per category, repeated under include/, src/, test/ and tools/:

| category | contents | status |
|---|---|---|
| actuation/ | uart_frame, serial_port, feedback_parser, uart_writer + uart_bridge nodes | done |
| perception/ | ground_filter (segment_ground + self_hit_filter), obstacle_clustering, ground_model + detector_core (ground-aware obstacle detection), and their nodes | done |
| control/ | operation_core (logic, no ROS) + operation_manager node (live mode) | done |
| planning/ | path_utils, lanelet_map, route_core + route_planner node | route + avoidance planner done |
| localization/ | wheel_odometry, localization_manager, lidar_preprocess, odom_to_twist, velodyne_static_tf | done |
| tools/golden/ | gen_golden.py, makes golden data from the Python reference | done |
