cmake_minimum_required(VERSION 3.8)
project(fusion_common)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

# Plain CMake (no ROS): shared by the fusion, the trackers, the detector and the ROS 2 adapters.
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d)
# Some OpenCV builds (e.g. a source build in /usr/local on Jetson) load a config that leaves
# OpenCV_INCLUDE_DIRS / OpenCV_LIBS empty: look for the headers next to the install and
# link the libraries by name.
get_filename_component(_ocv_prefix "${OpenCV_DIR}/../../.." ABSOLUTE)
find_path(OPENCV_CORE_HEADER opencv2/core.hpp
  HINTS ${OpenCV_INCLUDE_DIRS} ${_ocv_prefix}/include ${OpenCV_INSTALL_PATH}/include /usr/local/include /usr/include
  PATH_SUFFIXES opencv4)
if(NOT OPENCV_CORE_HEADER)
  message(FATAL_ERROR "OpenCV found via ${OpenCV_DIR}, but opencv2/core.hpp is in neither "
    "'${OpenCV_INCLUDE_DIRS}' nor ${_ocv_prefix}/include/opencv4. Install the headers or pass "
    "-DOpenCV_DIR=<dir of the matching OpenCVConfig.cmake> (delete build/ first).")
endif()
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/fusion_opencv.cmake)
fusion_opencv_libs(FUSION_OPENCV_LIBS opencv_core opencv_imgproc opencv_calib3d)
message(STATUS "fusion_common: OpenCV '${OpenCV_VERSION}' from ${OpenCV_DIR}, headers ${OPENCV_CORE_HEADER}, "
  "libs ${FUSION_OPENCV_LIBS}")

# camera model + calibration, Hungarian, class table, common types, ego motion
add_library(${PROJECT_NAME} STATIC src/camera_model.cpp src/object_classes.cpp src/ego_motion.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(${PROJECT_NAME} PUBLIC ${OPENCV_CORE_HEADER})
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen yaml-cpp ${FUSION_OPENCV_LIBS})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(EXPORT ${PROJECT_NAME}Targets NAMESPACE ${PROJECT_NAME}:: DESTINATION share/${PROJECT_NAME}/cmake)
# config: the dependencies again (fusion_opencv_libs for packages that link OpenCV themselves)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake [=[
include(CMakeFindDependencyMacro)
find_dependency(Eigen3)
find_dependency(yaml-cpp)
find_dependency(OpenCV COMPONENTS core imgproc calib3d)
include(${CMAKE_CURRENT_LIST_DIR}/fusion_opencv.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/fusion_commonTargets.cmake)
]=])
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake cmake/fusion_opencv.cmake
  DESTINATION share/${PROJECT_NAME}/cmake)
