#!/bin/bash
# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

# Exit on errors and undefined variables
set -euo pipefail

# Simple check of input parameters
if [[ $# -eq 0 ]]; then
    echo "Please specify the names of one or more packages to build"
    exit 2
fi

# Define directories
SCRIPT_DIR=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")
SRC_DIR="${SCRIPT_DIR}/src"  # Directory where build scripts for individual packages are located
LOG_DIR="${SCRIPT_DIR}/log"  # Directory where log of build script execution is written to
DEB_DIR="${SCRIPT_DIR}/deb"  # Directory where all compiled debian packages are saved

# Create directories for build logs and created deb packages
mkdir -p "${LOG_DIR}"
mkdir -p "${DEB_DIR}"

# Define name of docker image to build custom packages
IMAGE_NAME=nie_custom_deb_builder_20_04

# Build the docker image to build packages (It is probably cached)
echo "Creating package build environment"
docker build -t ${IMAGE_NAME} "${SCRIPT_DIR}" &> "${LOG_DIR}/${IMAGE_NAME}.log"

# Derive package architecture
ARCH=$(uname -m) # x86_64 / aarch64
[[ "$ARCH" == "x86_64" ]] && PKG_ARCH="amd64"
[[ "$ARCH" == "aarch64" ]] && PKG_ARCH="arm64"

# Define error handler
build_error()
{
    echo "ERROR occured while building package '$1', please check log file '$2'"
}

# Loop over all given package names
for package in "$@"
do
    if [[ ! -f "${SRC_DIR}/${package}" ]]; then
        echo "Error: Could not find build script for package '${package}'"
        exit 1
    fi

    # Install error handler for this package
    trap 'build_error ${package} "${LOG_DIR}/${package}.log"' ERR

    echo "Creating Debian package for '${package}'"
    docker run \
        --rm \
        --env PKG_ARCH="${PKG_ARCH}" \
        --volume "${SRC_DIR}":/src \
        --volume "${DEB_DIR}":/deb \
        ${IMAGE_NAME}:latest \
        bash -c "/src/${package}" &> "${LOG_DIR}/${package}.log"
done
