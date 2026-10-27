cmake_minimum_required(VERSION 3.8)
project(ekf_tracker)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(fusion_common REQUIRED)   # Eigen + OpenCV core (Jetson-safe header lookup)

# The EKF tracker (AEB pipeline): aeb/ is a byte-identical copy (see SOURCE.md), compiled as is.
# aeb_wrapper.cpp is compiled through src/ekf_aeb_bridge.cpp (#include), not on its own.
# TDA4 builds the original files of the app instead (this package is not compiled there).
add_library(${PROJECT_NAME} STATIC
  aeb/radarprocessing/ekf_tracker.cpp
  aeb/radarprocessing/gating_module.cpp
  aeb/radarprocessing/radar_clusterer.cpp
  aeb/radarprocessing/mio_selector.cpp
  aeb/radarprocessing/mirror_detector.cpp
  src/ekf_aeb_bridge.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/aeb/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/ekf_tracker/aeb>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC fusion_common::fusion_common)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY aeb/include/ DESTINATION include/ekf_tracker/aeb)
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(EXPORT ${PROJECT_NAME}Targets NAMESPACE ${PROJECT_NAME}:: DESTINATION share/${PROJECT_NAME}/cmake)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake [=[
include(CMakeFindDependencyMacro)
find_dependency(fusion_common)
include(${CMAKE_CURRENT_LIST_DIR}/ekf_trackerTargets.cmake)
]=])
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake DESTINATION share/${PROJECT_NAME}/cmake)
