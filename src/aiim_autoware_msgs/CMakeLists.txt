# Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

cmake_minimum_required(VERSION 3.5)
project(aiim_autoware_msgs)

# Generate messages
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
# Messages 
  "msg/BoundingBox.idl"
  "msg/BoundingBoxArray.idl"  
  "msg/Complex32.idl"
  "msg/ControlDiagnostic.idl"
  "msg/DiagnosticHeader.idl"
  "msg/HADMapBin.idl"
  "msg/MapPrimitive.idl"
  "msg/Quaternion32.idl"
  "msg/Route.idl"
  "msg/Trajectory.idl"
  "msg/TrajectoryPoint.idl"
  "msg/VehicleControlCommand.idl"
  "msg/VehicleKinematicState.idl"
  "msg/VehicleStateCommand.idl"
  "msg/VehicleStateReport.idl"
# Newly Added from Autoware
  "msg/HADMapRoute.idl"
  "msg/HADMapSegment.idl"
  "msg/TrajectoryPointAutoware.idl"
  "msg/TrajectoryAutoware.idl"
  "msg/VehicleKinematicStateAutoware.idl"
  "msg/PointClusters.idl"
  "msg/PointXYZIF.idl"
  "msg/DetectedObject.idl"
  "msg/DetectedObjectKinematics.idl"
  "msg/ObjectClassification.idl"
  "msg/Shape.idl"
  "msg/GearReport.idl"
  "msg/GearCommand.idl"

# Services
  "srv/HADMapService.idl"
  "srv/ModifyTrajectory.idl"
  "srv/ModifyTrajectoryAutoware.idl"   

# Actions
  "action/PlanTrajectory.idl"
  # Newly Added from Autoware
  "action/PlanTrajectoryAutoware.idl"
  "action/PlannerCostmap.idl"
  
  DEPENDENCIES "std_msgs" "geometry_msgs" "sensor_msgs" "nav_msgs"
 )

# find dependencies
# find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
