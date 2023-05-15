# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
project(lidar_odometry_viewer)

find_package(VTK 8.2)
if(VTK_VERSION VERSION_LESS "8.2")
    message(WARNING "VTK version is smaller than required 8.2. Not building ${PROJECT_NAME}")
else()
    set(LIBS nie_lidar)

    file(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp")
    cxx_executable_with_libs(${PROJECT_NAME} "${LIBS}" "${SOURCES}")
endif()
