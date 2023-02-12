# This is a modified version of the original points_downsampler CMakeLists.txt from Autoware.
# The original is licensed under the Apache License, Version 2.0. For more information
# see the LICENSE file in the repository root
#
# The changes made in this file, of which a summary is listed below, are copyrighted:
#
# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
#
# List of changes:
# * Everything not needed for the voxel_grid_filter is removed
# * -O2 and -g CXX Flags have been removed
# * Migration to ROS2
# * Update to use aiim common cmake

cmake_minimum_required(VERSION 3.5)

project(voxel_grid_filter)

find_package(PCL)
# ROS2 Dependencies
find_package(ament_cmake REQUIRED)
find_package(aiim_ros_cmake REQUIRED)

add_executable(${PROJECT_NAME} src/voxel_grid_filter.cpp)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
aiim_auto_ros_dependencies(${PROJECT_NAME} 
        aiim_roscpp
        pcl_conversions
        rclcpp
        sensor_msgs)
aiim_target_set_default_properties(${PROJECT_NAME})

aiim_target_ament_install(${PROJECT_NAME})

ament_package()
