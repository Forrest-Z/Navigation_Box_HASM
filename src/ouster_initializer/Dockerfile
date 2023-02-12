# syntax = docker/dockerfile:experimental
# ^This line is required for passing the ssh-agent information
# --------------------------------------------------------------------------------
# Copyright (C) 2022 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
# --------------------------------------------------------------------------------
#######################################################################################
FROM osrf/ros:foxy-desktop AS apt_dependencies
 
# Ensure non-interactive execution
ARG DEBIAN_FRONTEND=noninteractive
 
# Set maintainer information
LABEL maintainer="NavInfo Europe"
 
# Install required base and build packages
RUN apt-get update \
 && apt-get install --no-install-recommends --assume-yes \
       apt-utils=2.0.* \
       build-essential=12.8* \
       cmake=3.16.* \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*
 
## TODO: Check if you have other apt dependencies!
 
#######################################################################################
FROM apt_dependencies as pre_build
 
# Copy our source code into the container
COPY . /ros_ws/src/my_pkg
 
# Disable strict checking to avoid permission issues
RUN git config --global core.sshCommand 'ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no'
# Clone the other packages we need
WORKDIR /ros_ws/src
## TODO: Add/remove depending on your package!
RUN --mount=type=ssh git clone ssh://git@ssh.bitbucket.cms.navinfo.cloud:7999/aiim_ros/aiim_ros_common.git
 
#######################################################################################
FROM pre_build as build
 
# These build instructions should be the same for all ROS2 packages
WORKDIR /ros_ws/
RUN . /opt/ros/foxy/setup.sh \
&& colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
 
#######################################################################################
FROM build as test
 
# If we have tests, we can run them here
## TODO: Add instructions how to run the tests
RUN echo "No tests yet"