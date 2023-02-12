# Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
# --------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.5)
project(ouster_initializer)

# Add ouster client library
add_subdirectory(ouster_client)

# Find cmake dependencies
find_package(ament_cmake REQUIRED)
find_package(aiim_ros_cmake REQUIRED)

# Create executable
add_executable(${PROJECT_NAME} src/ouster_initializer.cpp)
target_link_libraries(${PROJECT_NAME} ouster_client)
aiim_target_set_default_properties(${PROJECT_NAME})
aiim_auto_ros_dependencies(${PROJECT_NAME}
        rclcpp
        aiim_roscpp)

# Install
aiim_target_ament_install(${PROJECT_NAME})

# Finalize package
ament_package()
