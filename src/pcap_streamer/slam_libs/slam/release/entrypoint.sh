#!/bin/bash
# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

# Exit on errors and undefined variables
set -euo pipefail

# Check if the variable are set, and fall back to defaults otherwise
USER_NAME=${USER_NAME:-user}
USER_GID=${USER_GID:-1000}
USER_UID=${USER_UID:-1000}

# Create group with USER_GID
if ! getent group ${USER_NAME} >/dev/null; then
  groupadd --force \
           --gid ${USER_GID} \
           ${USER_NAME} 2> /dev/null
fi

# Create user with USER_UID
# NOTE: We need to pass --force-badname, because the given username can be anything
if ! getent passwd ${USER_NAME} >/dev/null; then
  adduser --disabled-login \
          --force-badname \
          --uid ${USER_UID} \
          --gid ${USER_GID} \
          --gecos ${USER_NAME} \
          ${USER_NAME} > /dev/null

  # Set the password to the empty string (Via hash)
  echo ${USER_NAME}:U6aMy0wojraho | sudo chpasswd -e
fi

# Add user to the sudo group
usermod -aG sudo ${USER_NAME}

# Create fake XDG_RUNTIME_DIR
export XDG_RUNTIME_DIR=/run/user/${USER_UID}
install -d -m 0700 -o ${USER_UID} -g ${USER_GID} ${XDG_RUNTIME_DIR}

# Change to the user home directory
cd /home/${USER_NAME}

# Drop from root account to user account and run passed cmd
exec gosu ${USER_NAME} "$@"
