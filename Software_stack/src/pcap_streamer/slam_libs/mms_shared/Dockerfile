# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
# --------------------------------------------------------------------------------
FROM nvidia/cuda:11.2.0-devel-ubuntu20.04 AS build_env

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
       libboost-all-dev=1.71.* \
       libgeographic-dev=1.50.* \
       libgoogle-glog-dev=0.4.0* \
       libeigen3-dev=3.3.7* \
       libceres-dev=1.14.0* \
       && apt-get clean \
       && rm -rf /var/lib/apt/lists/*

# Install needed python dependencies
RUN apt-get update \
 && apt-get install --no-install-recommends --assume-yes \
        python3-pip \
        python3-setuptools \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*
RUN pip3 install \
        pytest \
        coverage \
        setuptools \
        wheel \
        pytest-check

# Install custom NavInfo .deb packages
# NOTICE: tensorflow is added temporarily as the build will fail when WITH_TENSORFLOW=TRUE
RUN echo "deb [trusted=yes] http://aptmirror.navinfo.eu/navinfo_2004 /" > /etc/apt/sources.list.d/navinfo.list \
       && apt-get update \
       && apt-get install --no-install-recommends --assume-yes \
       navinfo-libnanoflann-dev=1.3.1-2 \
       navinfo-libdate-dev=2.4.1* \
       navinfo-libopencv-dev-desktop-cuda=4.5.0* \
       pylon=5.1.* \
       && apt-get clean \
       && rm -rf /var/lib/apt/lists/*

# The creation of these folders is necessary for proper usage of PIP to install local Python libraries
RUN mkdir /.local && chmod a+rwx /.local && mkdir /.cache && chmod a+rwx  /.cache

#######################################################################################

