cmake_minimum_required(VERSION 3.8)
project(radar_tracker)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(perception_msgs REQUIRED)
find_package(ekf_tracker REQUIRED)   # the EKF tracker (AEB pipeline copy), radar_tracker/ekf

add_executable(radar_tracker_node src/radar_tracker_node.cpp)
target_compile_options(radar_tracker_node PRIVATE -Wall -Wextra)
target_link_libraries(radar_tracker_node ekf_tracker::ekf_tracker)
ament_target_dependencies(radar_tracker_node rclcpp sensor_msgs nav_msgs visualization_msgs perception_msgs)

install(TARGETS radar_tracker_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
