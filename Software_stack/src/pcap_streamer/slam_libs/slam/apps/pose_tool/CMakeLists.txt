# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
project(pose_tool)

set(LIBS nie_lidar nie_geo_convert nie_cv kmlbase kmldom kmlconvenience)

file(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp" "${CMAKE_CURRENT_LIST_DIR}/src/*.inl")

cxx_executable_with_libs(${PROJECT_NAME} "${LIBS}" "${SOURCES}")
target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_LIST_DIR}/src/")

if(BUILD_TESTS)
    nie_build_tests()
endif(BUILD_TESTS)
