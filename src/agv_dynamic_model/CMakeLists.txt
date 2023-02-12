# Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
cmake_minimum_required(VERSION 3.16)

project(agv_simulator)

find_package(ament_cmake REQUIRED)
find_package(aiim_ros_cmake REQUIRED)

file(GLOB_RECURSE SOURCES_CPP "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp")

add_executable(${PROJECT_NAME} "${SOURCES_CPP}")

aiim_target_set_default_properties(${PROJECT_NAME})
aiim_auto_ros_dependencies(${PROJECT_NAME}
        rclcpp
        sensor_msgs
        aiim_autoware_msgs
        aiim_roscpp
        tf2_ros)

aiim_target_ament_install(${PROJECT_NAME})

ament_package()

