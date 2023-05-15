#!/bin/bash
# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

set -uo pipefail

cd "${0%/*}"/.. #go to the directory that have the builder repo
if [ $? -ne 0 ]; then
	echo "ERROR! Failed to cd to the folder that contains builder repo. Run script with './sync.sh' to make sure it works correctly"
	exit 1
fi

echo "************************************************************"
echo "*** Syncing all git repos (active branch only) ***"
echo "************************************************************"
echo ""

for dir in */
do
    cd "${dir}"

	header="Working in : ${dir}"
	echo $header
	for i in $(seq 1 ${#header}); do printf "="; done #puts a line under the header
	echo; echo

    #if it have a .git dir
    if [ -d .git ]; then
		echo "This is a git repo, current active branch is: `git rev-parse --abbrev-ref HEAD`"
		echo
		
		echo "Running git pull:"
        git pull --ff-only 2>&1 | sed 's/^/   /' | sed $'s/error\|fatal/\e[31m&\e[m/' # will fail if the pull will result in conflict

		echo
		echo "Running git push:"
		git push origin 2>&1 | sed 's/^/   /' | sed $'s/error\|fatal/\e[31m&\e[m/'
    else
		echo "not a git folder!"
	fi

	echo

    cd ..

done
