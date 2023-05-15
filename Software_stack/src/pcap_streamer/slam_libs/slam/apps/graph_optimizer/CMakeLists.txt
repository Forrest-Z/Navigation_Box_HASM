# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
project(graph_optimizer)

set(LIBS nie_cv nie_formats)

file(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp")

cxx_executable_with_libs(${PROJECT_NAME} "${LIBS}" "${SOURCES}")

if(BUILD_TESTS)
    # There are no header only includes that need target_include_directories.
    # It is needed to get clean includes of header files from ${PROJECT_NAME} within the unit tests.
    target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_LIST_DIR}/src/")
    nie_build_tests()
endif(BUILD_TESTS)
