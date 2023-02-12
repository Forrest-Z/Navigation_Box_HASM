# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
project(lidar_projector)

set(LIBS nie_formats)

file(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp")

cxx_executable_with_libs(${PROJECT_NAME} "${LIBS}" "${SOURCES}")
