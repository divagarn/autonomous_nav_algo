cmake_minimum_required(VERSION 3.8)
project(fusion_vehicles)
find_package(ament_cmake REQUIRED)
# common/: tuning shared by every vehicle; <vehicle>/: its calibration (every folder is installed)
install(DIRECTORY ./ DESTINATION share/${PROJECT_NAME}
  PATTERN CMakeLists.txt EXCLUDE PATTERN package.xml EXCLUDE)
ament_package()
