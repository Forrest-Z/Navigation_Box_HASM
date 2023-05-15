# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
project(nie_lidar)

find_package(Pcap 1.8.1 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(PDAL 2.0.1 REQUIRED)
set(LIBS nie_cv nie_formats ${Pcap_LIBRARIES} ${PCL_LIBRARIES} ${PDAL_LIBRARIES})

file(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp")

cxx_library_shared_with_libs(${PROJECT_NAME} "${LIBS}" "${SOURCES}")
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR}/src)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${LIBLAS_INCLUDE_DIRS} ${Pcap_INCLUDE_DIRS}
                                                         ${PCL_INCLUDE_DIRS} ${PDAL_INCLUDE_DIRS})

if(BUILD_TESTS)
    nie_build_tests()
endif(BUILD_TESTS)
