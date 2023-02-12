# Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
cmake_minimum_required(VERSION 3.5)

project(collision_avoidance_client)

# dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(COLLISION_AVOIDANCE_CLIENT_SRC
  src/collision_avoidance_client.cpp
)

set(COLLISION_AVOIDANCE_CLIENT_HEADERS
  src/collision_avoidance_client.hpp
)

# generate component node library
ament_auto_add_library(collision_avoidance_client_lib SHARED
  ${COLLISION_AVOIDANCE_CLIENT_SRC}
  ${COLLISION_AVOIDANCE_CLIENT_HEADERS}
)
rclcpp_components_register_node(collision_avoidance_client_lib
  PLUGIN "aiim::collision_avoidance_client::CollisionAvoidanceClient"
  EXECUTABLE collision_avoidance_client
)

# required for tf2
target_include_directories(collision_avoidance_client_lib SYSTEM PUBLIC ${tf2_INCLUDE_DIRS})

# ament package generation and installing
ament_auto_package(
  INSTALL_TO_SHARE
    params
    launch
)
