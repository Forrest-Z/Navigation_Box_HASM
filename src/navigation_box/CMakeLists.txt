# Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
cmake_minimum_required(VERSION 3.16)

project(navigation_box)

find_package(ament_cmake REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  navigation_box/localization_nodes.py
  navigation_box/planning_nodes.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY params/
        DESTINATION share/${PROJECT_NAME}/params)
install(DIRECTORY rviz/
        DESTINATION share/${PROJECT_NAME}/rviz)
install(DIRECTORY models/
        DESTINATION share/${PROJECT_NAME}/models)

ament_package()
