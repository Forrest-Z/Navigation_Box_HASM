# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
# --------------------------------------------------------------------------------
FROM osrf/ros:foxy-desktop AS build_env

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
RUN echo "deb [trusted=yes] http://aptmirror.navinfo.eu/navinfo_2004 /" > /etc/apt/sources.list.d/navinfo.list \
 && apt-get update \
 && apt-get install --no-install-recommends --assume-yes \
       navinfo-ceres-solver=1.14.0-4  \
       navinfo-libnanoflann-dev=1.3.1-2 \
       navinfo-libdate-dev=2.4.1* \
       navinfo-libopencv-dev-desktop=4.5.0* \
       navinfo-liblas-dev=1.8.1* \
       pylon=5.1.*

# Install extra ROS dependencies
RUN apt-get update \
 && apt-get install --no-install-recommends --assume-yes \
       ros-foxy-ros-testing