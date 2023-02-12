# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

cmake_minimum_required(VERSION 3.16)

project(path_cluster_detection)

# Required for pcl functionality
find_package(PCL REQUIRED)

# List of ros packages required by buildtool
find_package(aiim_ros_cmake REQUIRED)
find_package(ament_cmake REQUIRED)

file(GLOB_RECURSE SOURCES_CPP "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp")

add_executable(${PROJECT_NAME} "${SOURCES_CPP}")

# Custom function that searches for and add ros packages
aiim_auto_ros_dependencies(${PROJECT_NAME}
    aiim_roscpp
    aiim_autoware_msgs
    aiim_geometry
    geometry_msgs
    rclcpp
    sensor_msgs
    std_msgs
    tf2
    tf2_ros
    visualization_msgs
    )

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${PCL_INCLUDE_DIRS})

aiim_target_set_default_properties(${PROJECT_NAME})

aiim_target_ament_install(${PROJECT_NAME})

ament_package()