# Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
project(nie_radar)

set(LIBS nie_lidar)

file(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp")

cxx_library_shared_with_libs(${PROJECT_NAME} "${LIBS}" "${SOURCES}")
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR}/src)

if(BUILD_TESTS)
    nie_build_tests()
endif(BUILD_TESTS)
