# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

include("${CMAKE_CURRENT_LIST_DIR}/../builder/cmake/build_main.cmake")

include("${NIE_DIR}/mms_shared/CMakeLists.txt")

if(NOT TARGET nie_lidar)
    include("${CMAKE_CURRENT_LIST_DIR}/libs/nie_lidar/CMakeLists.txt")
endif()

if(NOT TARGET nie_radar)
    include("${CMAKE_CURRENT_LIST_DIR}/libs/nie_radar/CMakeLists.txt")
endif()

if(NOT TARGET nie_loam)
    include("${CMAKE_CURRENT_LIST_DIR}/libs/nie_loam/CMakeLists.txt")
endif()

if(NOT TARGET nie_drawing)
    include("${CMAKE_CURRENT_LIST_DIR}/libs/nie_drawing/CMakeLists.txt")
endif()

if(NOT TARGET nie_vo)
    include("${CMAKE_CURRENT_LIST_DIR}/libs/nie_vo/CMakeLists.txt")
endif()

add_subdirectory(apps/flattener/)

# General projects

set(BUILD_BBOX_VIEWER
    "TRUE"
    CACHE BOOL "Build bounding box viewer application")
if(BUILD_BBOX_VIEWER)
    include("${CMAKE_CURRENT_SOURCE_DIR}/apps/bbox_viewer/CMakeLists.txt")
endif()

set(BUILD_LOOP_FINDER
    "TRUE"
    CACHE BOOL "Build loop finder application")
if(BUILD_LOOP_FINDER)
    include("${CMAKE_CURRENT_SOURCE_DIR}/apps/loop_finder/CMakeLists.txt")
endif()

set(BUILD_POSE_TOOL
    "TRUE"
    CACHE BOOL "Build pose file analysis tool")
if(BUILD_POSE_TOOL)
    include("${CMAKE_CURRENT_SOURCE_DIR}/apps/pose_tool/CMakeLists.txt")
endif()

set(BUILD_POSE_BENCHMARKER
    "TRUE"
    CACHE BOOL "Build pose benchmarking tool")
if(BUILD_POSE_TOOL)
    include("${CMAKE_CURRENT_SOURCE_DIR}/apps/pose_benchmarker/CMakeLists.txt")
endif()

set(BUILD_PGO_PROBLEM_BUILDER
    "TRUE"
    CACHE BOOL "Build tool to construct the PGO problem from a pose file")
if(BUILD_PGO_PROBLEM_BUILDER)
    include("${CMAKE_CURRENT_SOURCE_DIR}/apps/pgo_problem_builder/CMakeLists.txt")
endif()

set(BUILD_BBOX_POSE_REL_TO_ABS
    "TRUE"
    CACHE BOOL "Build tool to interpolate two pose files")
if(BUILD_BBOX_POSE_REL_TO_ABS)
    include("${CMAKE_CURRENT_SOURCE_DIR}/apps/bbox_pose_rel_to_abs/CMakeLists.txt")
endif()

set(BUILD_CSV_TO_POSE_EDGE
    "TRUE"
    CACHE BOOL "Build app for converting csv to pose edges")
if(BUILD_CSV_TO_POSE_EDGE)
    include("${CMAKE_CURRENT_SOURCE_DIR}/apps/csv_to_pose_edge/CMakeLists.txt")
endif()

set(BUILD_POSE_DISTORTER
    "TRUE"
    CACHE BOOL "Build app for applying distortions to poses")
if(BUILD_POSE_DISTORTER)
    include("${CMAKE_CURRENT_SOURCE_DIR}/apps/pose_distorter/CMakeLists.txt")
endif()

set(BUILD_ODOMETRY_EDGE_REWEIGHTER
    "TRUE"
    CACHE BOOL "Build odometry edge reweighter application")
if(BUILD_ODOMETRY_EDGE_REWEIGHTER)
    include("${CMAKE_CURRENT_SOURCE_DIR}/apps/odometry_edge_reweighter/CMakeLists.txt")
endif()

set(BUILD_SCRIPTS
    "TRUE"
    CACHE BOOL "Build/copy all scripts")
if(BUILD_SCRIPTS)
    include("${CMAKE_CURRENT_SOURCE_DIR}/scripts/CMakeLists.txt")
endif()

# Visual slam projects

set(BUILD_GRAPH_OPTIMIZER
    "TRUE"
    CACHE BOOL "Build graph optimizer application")
if(BUILD_GRAPH_OPTIMIZER)
    include("${CMAKE_CURRENT_SOURCE_DIR}/apps/graph_optimizer/CMakeLists.txt")
endif()

set(BUILD_VISUAL_ODOMETRY_MONO
    "TRUE"
    CACHE BOOL "Build monocular visual odometry application")
if(BUILD_VISUAL_ODOMETRY_MONO)
    include("${CMAKE_CURRENT_SOURCE_DIR}/apps/visual_odometry_mono/CMakeLists.txt")
endif()

set(BUILD_KITTI_BENCHMARKER
    "TRUE"
    CACHE BOOL "Build KITTI benckmark application")
if(BUILD_KITTI_BENCHMARKER)
    include("${CMAKE_CURRENT_SOURCE_DIR}/apps/kitti_benchmarker/CMakeLists.txt")
endif()

set(BUILD_KITTI_POSE_INTERPOLATOR
    "TRUE"
    CACHE BOOL "Build KITTI pose interpolator application")
if(BUILD_KITTI_POSE_INTERPOLATOR)
    include("${CMAKE_CURRENT_SOURCE_DIR}/apps/kitti_pose_interpolator/CMakeLists.txt")
endif()

# LiDAR slam projects

set(BUILD_ALIGNMENT_VIEWER
    "TRUE"
    CACHE BOOL "Build a viewer for analyzing match results")
if(BUILD_ALIGNMENT_VIEWER)
    include("${CMAKE_CURRENT_SOURCE_DIR}/apps/alignment_viewer/CMakeLists.txt")
endif()

set(BUILD_CORRESPONDENCE_VIEWER
    "TRUE"
    CACHE BOOL "Build a viewer for visualizing correspondences")
if(BUILD_CORRESPONDENCE_VIEWER)
    include("${CMAKE_CURRENT_SOURCE_DIR}/apps/correspondence_viewer/CMakeLists.txt")
endif()

set(BUILD_SET_VIEWER
    "TRUE"
    CACHE BOOL "Build a viewer for analyzing a set of match results")
if(BUILD_SET_VIEWER)
    include("${CMAKE_CURRENT_SOURCE_DIR}/apps/set_viewer/CMakeLists.txt")
endif()

set(BUILD_PCAP_TRAJECTORY_TOOL
    "TRUE"
    CACHE BOOL "Build pcap trajectory tool application")
if(BUILD_PCAP_TRAJECTORY_TOOL)
    include("${CMAKE_CURRENT_SOURCE_DIR}/apps/pcap_trajectory_tool/CMakeLists.txt")
endif()

set(BUILD_LIDAR_LOOP_FILTER
    "TRUE"
    CACHE BOOL "Build lidar loop filter application")
if(BUILD_LIDAR_LOOP_FILTER)
    include("${CMAKE_CURRENT_SOURCE_DIR}/apps/lidar_loop_filter/CMakeLists.txt")
endif()

set(BUILD_MANUAL_LOOP_CLOSER
    "TRUE"
    CACHE BOOL "Build manual loop closer application")
if(BUILD_MANUAL_LOOP_CLOSER)
    include("${CMAKE_CURRENT_SOURCE_DIR}/apps/manual_loop_closer/CMakeLists.txt")
endif()

set(BUILD_LIDAR_LOOP_CLOSER
    "TRUE"
    CACHE BOOL "Build lidar loop closer application")
if(BUILD_LIDAR_LOOP_CLOSER)
    include("${CMAKE_CURRENT_SOURCE_DIR}/apps/lidar_loop_closer/CMakeLists.txt")
endif()

set(BUILD_LIDAR_LOOP_COMPARE_TOOL
    "TRUE"
    CACHE BOOL "Build lidar loop compare tool")
if(BUILD_LIDAR_LOOP_COMPARE_TOOL)
    include("${CMAKE_CURRENT_SOURCE_DIR}/apps/lidar_loop_compare_tool/CMakeLists.txt")
endif()

set(BUILD_BBOX_TO_POSE_EDGE_CONVERTER
    "TRUE"
    CACHE BOOL "Build tool for converting bbox based pgo edges to pose based pgo edges.")
if(BUILD_BBOX_TO_POSE_EDGE_CONVERTER)
    include("${CMAKE_CURRENT_SOURCE_DIR}/apps/bbox_to_pose_edge_converter/CMakeLists.txt")
endif()

set(BUILD_LIDAR_ODOMETRY_ESTIMATOR
    "TRUE"
    CACHE BOOL "Build lidar odometry estimator application")
if(BUILD_LIDAR_ODOMETRY_ESTIMATOR)
    include("${CMAKE_CURRENT_SOURCE_DIR}/apps/lidar_odometry_estimator/CMakeLists.txt")
endif()

set(BUILD_LAS_CREATOR
    "TRUE"
    CACHE BOOL "Build las creator application")
if(BUILD_LAS_CREATOR)
    include("${CMAKE_CURRENT_SOURCE_DIR}/apps/las_creator/CMakeLists.txt")
endif()

set(BUILD_LIDAR_ODOMETRY_VIEWER
    "TRUE"
    CACHE BOOL "Build lidar odometry estimator application")
if(BUILD_LIDAR_ODOMETRY_VIEWER)
    include("${CMAKE_CURRENT_SOURCE_DIR}/apps/lidar_odometry_viewer/CMakeLists.txt")
endif()

set(BUILD_LIDAR_SWEEP_WRITER
    "TRUE"
    CACHE BOOL "Build lidar sweep writer application")
if(BUILD_LAS_CREATOR)
    include("${CMAKE_CURRENT_SOURCE_DIR}/apps/lidar_sweep_writer/CMakeLists.txt")
endif()

set(BUILD_LOAM_POSE_REL_TO_ABS
    "TRUE"
    CACHE BOOL "Build tool to convert the relative loam poses to absolute.")
if(BUILD_LOAM_POSE_REL_TO_ABS)
    include("${CMAKE_CURRENT_SOURCE_DIR}/apps/loam_pose_rel_to_abs/CMakeLists.txt")
endif()

if(WITH_LADYBUG)
    set(BUILD_LIDAR_PROJECTOR
        "TRUE"
        CACHE BOOL "Build lidar projector application")
    if(BUILD_LIDAR_PROJECTOR)
        include("${CMAKE_CURRENT_SOURCE_DIR}/apps/lidar_projector/CMakeLists.txt")
    endif()
endif()

# GNSS Mapping projects
set(BUILD_LOCALIZATION_PCL_CONVERTER
    "TRUE"
    CACHE BOOL "Build localization pcl converter application")
if(BUILD_LOCALIZATION_PCL_CONVERTER)
    include("${CMAKE_CURRENT_SOURCE_DIR}/apps/localization_pcl_converter/CMakeLists.txt")
endif()

# Customer specific tools

set(BUILD_NIE_CHINA_LAS_NAMER
    "TRUE"
    CACHE BOOL "Build tool to rename las files according to Navinfo China naming convection.")
if(BUILD_NIE_CHINA_LAS_NAMER)
    include("${CMAKE_CURRENT_SOURCE_DIR}/apps/nie_china_las_namer/CMakeLists.txt")
endif()
