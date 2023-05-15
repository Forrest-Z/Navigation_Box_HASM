# This is a modified version of the original map_file CMakeLists.txt from Autoware.
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
# * Everything not related to the points map loader is removed
# * -O2 CXX Flag has been removed
# * CURL has been removed
# * stdc++fs has been added to deal with experimental filesystem library
# * Updated to ROS2 using ament. Most of the file has been replaced with new syntax
# * Updated to use new AIIM cmake format

cmake_minimum_required(VERSION 3.5)

project(points_map_loader)

# PCL Dependent
find_package(PCL REQUIRED COMPONENTS io)
include_directories(${PCL_IO_INCLUDE_DIRS})

# ROS2 Dependencies
find_package(ament_cmake REQUIRED)
find_package(aiim_ros_cmake REQUIRED)

###########
## Build ##
###########

add_executable(${PROJECT_NAME} src/points_map_loader.cpp)
target_link_libraries(${PROJECT_NAME} ${PCL_IO_LIBRARIES})
aiim_target_set_default_properties(${PROJECT_NAME})
aiim_auto_ros_dependencies(${PROJECT_NAME} 
        aiim_autoware_msgs
        geometry_msgs
        pcl_conversions
        rclcpp
        sensor_msgs)

aiim_target_ament_install(${PROJECT_NAME})

ament_package()