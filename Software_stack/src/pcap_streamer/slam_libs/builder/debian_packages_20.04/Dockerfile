# Copyright (C) 2020, 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
# Start from a base Ubuntu 20.04 installation, with CUDA 11.2.0, and OpenGL support
# https://hub.docker.com/r/nvidia/cudagl/tags
FROM nvidia/cudagl:11.2.0-devel-ubuntu20.04

# Ensure non-interactive execution
ARG DEBIAN_FRONTEND=noninteractive

# Install dependencies that are required for building but are not specific to a single package
RUN apt-get update \
 && apt-get install --no-install-recommends --assume-yes \
        apt-utils \
        wget \
        build-essential \
        git \
        automake \
        autoconf \
        libtool \
        cmake \
        checkinstall \
        ca-certificates \
        pkg-config \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

# Copy checkinstall config file
COPY checkinstallrc /etc/

# Set work directory
WORKDIR /build
