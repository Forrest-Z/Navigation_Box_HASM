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


# Install development packages
RUN apt-get update \
 && apt-get install --no-install-recommends --assume-yes \
        googletest=1.10.* \
        libgeographic-dev=1.50.* \
        libgoogle-glog-dev=0.4.0* \
        libeigen3-dev=3.3.7* \
        libpcap-dev=1.9.1* \
        libpcl-dev=1.10.0* \
        libpdal-dev=2.0.1* \
        libsuitesparse-dev=1:5.7.* \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

# Install custom NavInfo .deb packages
# NOTICE: tensorflow is added temporarily as the build will fail when WITH_TENSORFLOW=TRUE
# TODO: Some of these are replaced in 20.04 by the default libraries such as libpdal
RUN echo "deb [trusted=yes] http://aptmirror.navinfo.eu/navinfo_2004 /" > /etc/apt/sources.list.d/navinfo.list \
 && apt-get update \
 && apt-get install --no-install-recommends --assume-yes \
        navinfo-ceres-solver=1.14.0-4  \
        navinfo-libnanoflann-dev=1.3.1-2 \
        navinfo-libdate-dev=2.4.1* \
        navinfo-libopencv-dev-desktop=4.5.0* \
        navinfo-libopencv-dev-desktop=4.5.0* \
        pylon 

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

