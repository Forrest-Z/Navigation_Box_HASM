# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
cmake_minimum_required(VERSION 3.5)

include("${CMAKE_CURRENT_LIST_DIR}/../builder/cmake/build_main.cmake")

# Libraries
if(NOT TARGET nie_core)
    include("${NIE_DIR}/mms_shared/common/libs/nie_core/CMakeLists.txt")
endif()

if(NOT TARGET nie_cv)
    include("${NIE_DIR}/mms_shared/common/libs/nie_cv/CMakeLists.txt")
endif()

if(NOT TARGET nie_formats)
    include("${NIE_DIR}/mms_shared/common/libs/nie_formats/CMakeLists.txt")
endif()

if(NOT TARGET nie_hardware)
    include("${NIE_DIR}/mms_shared/common/libs/nie_hardware/CMakeLists.txt")
endif()

if(NOT TARGET nie_geo_convert)
    include("${NIE_DIR}/mms_shared/common/libs/nie_geo_convert/CMakeLists.txt")
endif()

if(NOT TARGET nie_python_utils)
    include("${NIE_DIR}/mms_shared/common/libs/nie_python_utils/CMakeLists.txt")
endif()

# Apps
set(BUILD_WEIYA_INTRINSICS_CONVERTER
    "TRUE"
    CACHE BOOL "Build Weiya intrinsics converter")
if(BUILD_WEIYA_INTRINSICS_CONVERTER AND NOT TARGET weiya_intrinsics_converter)
    include("${NIE_DIR}/mms_shared/common/apps/weiya_intrinsics_converter/CMakeLists.txt")
endif()

if(WITH_CUDA)
    set(BUILD_CALIBRATOR
        "TRUE"
        CACHE BOOL "Build camera lens calibrator")
endif()

if(WITH_BASLER_CAMERA)
    set(BUILD_RECORDER
        "TRUE"
        CACHE BOOL "Build recorder ")
endif()

if(BUILD_CALIBRATOR AND NOT TARGET calibrator)
    include("${NIE_DIR}/mms_shared/common/apps/calibrator/CMakeLists.txt")
endif()

if(BUILD_RECORDER AND NOT TARGET recorder)
    include("${NIE_DIR}/mms_shared/common/apps/recorder/CMakeLists.txt")
endif()
