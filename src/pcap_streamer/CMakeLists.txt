# Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
cmake_minimum_required(VERSION 3.16)

# This line prevents the CUDA Unknown CMake command "cmake_initialize_per_config_variable" error.
project(pcap_streamer)

# NIE dependencies have to be included before the project is defined
get_filename_component(NIE_DIR ${CMAKE_CURRENT_LIST_DIR}/slam_libs ABSOLUTE)
include("${NIE_DIR}/builder/cmake/build_main.cmake")
# Libraries
include("${NIE_DIR}/mms_shared/common/libs/nie_core/CMakeLists.txt")
include("${NIE_DIR}/mms_shared/common/libs/nie_cv/CMakeLists.txt")
include("${NIE_DIR}/mms_shared/common/libs/nie_formats/CMakeLists.txt")
include("${NIE_DIR}/slam/libs/nie_lidar/CMakeLists.txt")

# This line *has* to be below any of the cmake includes.
# Any of the previous includes will have overwritten the project name.
project(pcap_streamer)

find_package(aiim_ros_cmake REQUIRED)
find_package(ament_cmake REQUIRED)

file(GLOB_RECURSE SOURCES_CPP "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp")

add_executable(${PROJECT_NAME} "${SOURCES_CPP}")
target_link_libraries(${PROJECT_NAME} nie_lidar)
aiim_target_set_default_properties(${PROJECT_NAME})
aiim_auto_ros_dependencies(${PROJECT_NAME} 
    rclcpp 
    sensor_msgs 
    pcl_conversions
    aiim_roscpp)

aiim_target_ament_install(${PROJECT_NAME})

ament_package()
