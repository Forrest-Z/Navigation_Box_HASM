# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
# Needs to be false.
# NOTE: Call the necessary include()'s *before* project() so local variables (e.g. PROJECT_NAME) are not overwritten
project(nie_hardware)

if(NOT IN_JENKINS)
    set(WITH_XSENS_GPS_IMU
        "FALSE"
        CACHE BOOL "Build with XSENS GPS/IMU support")
else()
    message("Skipping projects that can't build in Jenkins")
endif()
set(LIBS nie_cv nie_formats nie_geo_convert)

file(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp" "${CMAKE_CURRENT_LIST_DIR}/src/*.inl")

add_library(${PROJECT_NAME} SHARED)

# Basler
set(WITH_BASLER_CAMERA
    "TRUE"
    CACHE BOOL "Build with Basler camera support")
if(${WITH_BASLER_CAMERA})
    find_package(Pylon REQUIRED)
    list(APPEND LIBS ${Pylon_LIBRARIES})
    target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${Pylon_INCLUDE_DIRS})
else()
    # Remove globbed item.
    list(REMOVE_ITEM SOURCES "${CMAKE_CURRENT_LIST_DIR}/src/nie/hardware/camera/basler_camera.cpp")
    list(REMOVE_ITEM SOURCES "${CMAKE_CURRENT_LIST_DIR}/src/nie/hardware/device/stereo_triggering.cpp")
endif()

# Xsens GPS/IMU
if(${WITH_XSENS_GPS_IMU})
    find_package(Xsens REQUIRED)
    list(APPEND LIBS ${Xsens_LIBRARIES})
    target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${Xsens_INCLUDE_DIRS})
else()
    # Remove globbed items.
    list(REMOVE_ITEM SOURCES "${CMAKE_CURRENT_LIST_DIR}/src/nie/hardware/positioning/helper_xsens_gps_imu.cpp")
    list(REMOVE_ITEM SOURCES "${CMAKE_CURRENT_LIST_DIR}/src/nie/hardware/positioning/xsens_gps_imu.cpp")
endif()

target_sources(${PROJECT_NAME} PUBLIC ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_LIST_DIR}/src")
target_link_libraries(${PROJECT_NAME} ${LIBS})
cxx_default_target_properties(${PROJECT_NAME})

if(BUILD_TESTS)
    nie_build_tests()
endif(BUILD_TESTS)
