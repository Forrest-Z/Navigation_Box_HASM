#!/bin/bash
# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

# Exit on errors and undefined variables
set -euo pipefail

# Simple check of input parameters
if [[ $# -ne 2 ]]; then
    echo "Usage: $0 <docker_tag> <git_tag_or_branch>"
    echo
    echo "e.g. $0 lidar_slam:20.05_v1 master"

    exit 2
fi

# Copy the private key from the .ssh directory of the current user into the build context
# NOTE: There is a better / safer approach, but docker 18.09 or higher is needed for this, see:
#       https://confluence.navinfo.eu/pages/viewpage.action?pageId=35323925#Dockerrelease/deliverytherightway-CloningBitbucketrepositorieswithoutsharingsecrets
cp "$HOME/.ssh/id_rsa" .

# Create docker image
#DOCKER_BUILDKIT=1 docker build --ssh default ...
docker build \
  --network=host \
  --no-cache \
  --tag="$1" \
  --build-arg branch="$2" \
  --file=../Dockerfile .

# Clean up
rm id_rsa
