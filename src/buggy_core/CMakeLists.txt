cmake_minimum_required(VERSION 3.8)
project(buggy_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(ground_segmentation_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(autoware_internal_debug_msgs REQUIRED)
find_package(autoware_internal_localization_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)
find_package(autoware_map_msgs REQUIRED)
pkg_check_modules(TINYXML2 REQUIRED tinyxml2)

# ---- pure C++ library (no ROS): easy to unit-test --------------------------------------------
# one folder per category: actuation/ (ECU link), perception/ (obstacles), control/ (operation manager);
# planning/ and localization/ follow as the Python nodes are ported
add_library(buggy_core_lib SHARED
  src/actuation/serial_port.cpp
  src/actuation/feedback_parser.cpp
  src/perception/obstacle_clustering.cpp
  src/perception/ground_filter.cpp
  src/perception/ground_model.cpp
  src/perception/detector_core.cpp
  src/control/operation_core.cpp
  src/control/mission_core.cpp
  src/planning/path_utils.cpp
  src/mapping/pcd_cells.cpp
  src/planning/lanelet_map.cpp
  src/planning/route_core.cpp
  src/planning/avoidance.cpp
  src/localization/localization_core.cpp
  src/localization/lidar_preprocess.cpp)
target_link_libraries(buggy_core_lib nlohmann_json::nlohmann_json ${TINYXML2_LIBRARIES})
target_include_directories(buggy_core_lib PRIVATE ${TINYXML2_INCLUDE_DIRS})
target_include_directories(buggy_core_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ---- nodes ---------------------------------------------------------------------------------
add_executable(uart_writer src/actuation/uart_writer_node.cpp)
target_link_libraries(uart_writer buggy_core_lib)
ament_target_dependencies(uart_writer rclcpp std_msgs)

add_executable(lidar_obstacle_detector src/perception/lidar_obstacle_detector_node.cpp)
target_link_libraries(lidar_obstacle_detector buggy_core_lib)
ament_target_dependencies(lidar_obstacle_detector rclcpp std_msgs nav_msgs sensor_msgs
  geometry_msgs visualization_msgs tf2 tf2_ros autoware_perception_msgs)

add_executable(segment_ground src/perception/segment_ground_node.cpp)
target_link_libraries(segment_ground buggy_core_lib)
ament_target_dependencies(segment_ground rclcpp sensor_msgs)

add_executable(self_hit_filter src/perception/self_hit_filter_node.cpp)
target_link_libraries(self_hit_filter buggy_core_lib)
ament_target_dependencies(self_hit_filter rclcpp sensor_msgs geometry_msgs visualization_msgs)

add_executable(wheel_odometry src/localization/wheel_odometry_node.cpp)
target_link_libraries(wheel_odometry buggy_core_lib)
ament_target_dependencies(wheel_odometry rclcpp nav_msgs geometry_msgs tf2_ros ground_segmentation_msgs)

add_executable(route_planner src/planning/route_planner_node.cpp)
target_link_libraries(route_planner buggy_core_lib)
ament_target_dependencies(route_planner rclcpp std_msgs nav_msgs geometry_msgs visualization_msgs tf2 tf2_ros)

add_executable(avoidance_planner src/planning/avoidance_planner_node.cpp)
target_link_libraries(avoidance_planner buggy_core_lib)
ament_target_dependencies(avoidance_planner rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs tf2 tf2_ros)

add_executable(localization_manager src/localization/localization_manager_node.cpp)
target_link_libraries(localization_manager buggy_core_lib)
ament_target_dependencies(localization_manager rclcpp std_msgs geometry_msgs std_srvs tf2 tf2_ros tf2_msgs
  autoware_internal_debug_msgs autoware_internal_localization_msgs)

add_executable(uart_bridge src/actuation/uart_bridge_node.cpp)
target_link_libraries(uart_bridge buggy_core_lib)
ament_target_dependencies(uart_bridge rclcpp ground_segmentation_msgs)

add_executable(lidar_preprocess src/localization/lidar_preprocess_node.cpp)
target_link_libraries(lidar_preprocess buggy_core_lib)
ament_target_dependencies(lidar_preprocess rclcpp sensor_msgs)

add_executable(odom_to_twist src/localization/odom_to_twist_node.cpp)
ament_target_dependencies(odom_to_twist rclcpp nav_msgs geometry_msgs)

add_executable(velodyne_static_tf src/localization/velodyne_static_tf_node.cpp)
ament_target_dependencies(velodyne_static_tf rclcpp geometry_msgs tf2_ros)

add_executable(map_projection_loader src/mapping/map_projection_loader_node.cpp)
ament_target_dependencies(map_projection_loader rclcpp autoware_map_msgs)
target_link_libraries(map_projection_loader yaml-cpp)

add_executable(pointcloud_map_loader src/mapping/pointcloud_map_loader_node.cpp)
target_link_libraries(pointcloud_map_loader buggy_core_lib yaml-cpp ${PCL_LIBRARIES})
target_include_directories(pointcloud_map_loader PRIVATE ${PCL_INCLUDE_DIRS})
ament_target_dependencies(pointcloud_map_loader rclcpp sensor_msgs pcl_conversions autoware_map_msgs)

add_executable(operation_manager src/control/operation_manager_node.cpp)
target_link_libraries(operation_manager buggy_core_lib)
ament_target_dependencies(operation_manager rclcpp std_msgs nav_msgs geometry_msgs visualization_msgs
  tf2 tf2_ros autoware_perception_msgs ground_segmentation_msgs)

add_executable(mission_manager src/control/mission_manager_node.cpp)
target_link_libraries(mission_manager buggy_core_lib)
ament_target_dependencies(mission_manager rclcpp std_msgs nav_msgs geometry_msgs visualization_msgs tf2 tf2_ros)

install(TARGETS buggy_core_lib
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS uart_writer lidar_obstacle_detector operation_manager mission_manager
  segment_ground self_hit_filter wheel_odometry route_planner avoidance_planner
  map_projection_loader pointcloud_map_loader
  localization_manager uart_bridge lidar_preprocess odom_to_twist velodyne_static_tf DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

# ---- tests (golden data generated by the Python reference, see tools/golden/gen_golden.py) ---------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_uart_frame test/actuation/test_uart_frame.cpp)
  target_link_libraries(test_uart_frame buggy_core_lib)
  target_compile_definitions(test_uart_frame PRIVATE
    GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/actuation")
  ament_add_gtest(test_obstacle_clustering test/perception/test_obstacle_clustering.cpp)
  target_link_libraries(test_obstacle_clustering buggy_core_lib)
  target_compile_definitions(test_obstacle_clustering PRIVATE
    GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/perception")
  ament_add_gtest(test_ground_filter test/perception/test_ground_filter.cpp)
  target_link_libraries(test_ground_filter buggy_core_lib)
  target_compile_definitions(test_ground_filter PRIVATE
    GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/perception")
  ament_add_gtest(test_wheel_odometry test/localization/test_wheel_odometry.cpp)
  target_link_libraries(test_wheel_odometry buggy_core_lib)
  target_compile_definitions(test_wheel_odometry PRIVATE
    GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/localization")
  ament_add_gtest(test_route_core test/planning/test_route_core.cpp)
  target_link_libraries(test_route_core buggy_core_lib)
  target_compile_definitions(test_route_core PRIVATE
    GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/planning"
    MAPS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../data/maps")
  ament_add_gtest(test_avoidance test/planning/test_avoidance.cpp)
  target_link_libraries(test_avoidance buggy_core_lib)
  target_compile_definitions(test_avoidance PRIVATE
    GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/planning"
    MAPS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../data/maps")
  ament_add_gtest(test_localization_core test/localization/test_localization_core.cpp)
  target_link_libraries(test_localization_core buggy_core_lib)
  target_compile_definitions(test_localization_core PRIVATE
    GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/localization")
  ament_add_gtest(test_feedback_parser test/actuation/test_feedback_parser.cpp)
  target_link_libraries(test_feedback_parser buggy_core_lib)
  target_compile_definitions(test_feedback_parser PRIVATE
    GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/actuation")
  ament_add_gtest(test_lidar_preprocess test/localization/test_lidar_preprocess.cpp)
  target_link_libraries(test_lidar_preprocess buggy_core_lib)
  target_compile_definitions(test_lidar_preprocess PRIVATE
    GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/localization")
  ament_add_gtest(test_detector_core test/perception/test_detector_core.cpp)
  target_link_libraries(test_detector_core buggy_core_lib)
  target_compile_definitions(test_detector_core PRIVATE
    GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/perception")
  ament_add_gtest(test_pcd_cells test/mapping/test_pcd_cells.cpp)
  target_link_libraries(test_pcd_cells buggy_core_lib)
  ament_add_gtest(test_mission_core test/control/test_mission_core.cpp)
  target_link_libraries(test_mission_core buggy_core_lib)
  ament_add_gtest(test_operation_core test/control/test_operation_core.cpp)
  target_link_libraries(test_operation_core buggy_core_lib)
  target_compile_definitions(test_operation_core PRIVATE
    GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/control")
endif()

ament_package()
