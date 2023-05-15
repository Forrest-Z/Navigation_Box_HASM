# Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
cmake_minimum_required(VERSION 3.16)

# Project name has to be the same to package name
project(aiim_ros_cmake)

# Required for building a package
find_package(ament_cmake REQUIRED)

# This tells ament that we want to make the file publicly accessible
# It can thus be used when they use find_package(aiim_ros_cmake)
# As it is only used for building, we do not have to install
list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS
  "./cmake/aiim_ros_cmake.cmake")

# It is still a package so:
ament_package()