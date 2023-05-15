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
        googletest \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

#######################################################################################
