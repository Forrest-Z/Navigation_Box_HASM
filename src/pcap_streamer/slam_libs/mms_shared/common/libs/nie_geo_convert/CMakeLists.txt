# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
project(nie_geo_convert)

find_package(GeographicLib REQUIRED)

# TODO nie_core is only being used to include the header-only rotation.hpp file so nie_geo_convert does not
# need to be linked against it. To fix this in the proper CMake way, the *_INCLUDE_DIRS must be set for our libraries
list(APPEND nie_geo_convert_libs ${GLOG_LIBRARIES} ${GeographicLib_LIBRARIES} nie_core)

file(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp" "${CMAKE_CURRENT_LIST_DIR}/src/*.inl")

cxx_library_shared_with_libs(${PROJECT_NAME} "${nie_geo_convert_libs}" ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC ${GLOG_INCLUDE_DIRS} ${GeographicLib_INCLUDE_DIRS}
                                                  "${CMAKE_CURRENT_LIST_DIR}/src")

if(BUILD_TESTS)
    nie_build_tests()
endif(BUILD_TESTS)
