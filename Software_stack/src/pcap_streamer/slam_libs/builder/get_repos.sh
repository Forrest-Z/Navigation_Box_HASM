#!/bin/bash
# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

set -uo pipefail

clone_or_update_repo()
{
	proj_and_repo=$1
	branch_name=$2

	repo_name=${proj_and_repo##*/} #get repo name which is after the slash

	if [ -d "$repo_name" ]; then
		echo "*** Repo ${repo_name} already exist, cloning skipped."

		cd "$repo_name"
		echo
		
		echo "*** Fetching to see new branches:"
		git fetch 2>&1 | sed 's/^/   /' | sed $'s/error\|fatal/\e[31m&\e[m/'
		echo
		
		echo "*** Checkout branch ${branch_name}:"
		git checkout $branch_name 2>&1 | sed 's/^/   /' | sed $'s/error\|fatal/\e[31m&\e[m/'
		git_status=$?
		echo

		if [ $git_status -ne 0 ]; then
			echo "*** Branch checkout failed, so checking out master:"
			git checkout master 2>&1 | sed 's/^/   /' | sed $'s/error\|fatal/\e[31m&\e[m/'
			echo
		fi

		echo "*** Pulling latest in branch:"
		git pull --ff-only 2>&1 | sed 's/^/   /' | sed $'s/error\|fatal/\e[31m&\e[m/' # will fail if the pull will result in conflict
		echo

	else
		echo "*** Running git clone for repo ${repo_name}:"
		git clone "ssh://git@bitbucket.navinfo.eu:7999/$proj_and_repo.git" 2>&1 | sed 's/^/   /'
		echo

		cd "$repo_name"
		echo "*** Checkout branch ${branch_name}:"
		git checkout $branch_name 2>&1 | sed 's/^/   /' | sed $'s/error\|fatal/\e[31m&\e[m/'
		echo
	fi

	cd ..
}

show_help()
{
	echo "Repository Synchornization Helper Script"
	echo "----------------------------------------"
	echo "This script will aid the process of cloning repositories and keeping them up to date. You"
	echo "can use this script to clone repositories for the first time, or to synchronize already"
	echo "existing repositories to make sure you have the latest version on the master branch (or"
	echo "whichever branch you define)."
	echo
	echo "The script will take repository dependencies into account, by reading the dependencies.txt"
	echo "file at the root of the target repository. Therefore, cloning the 'evvis/node' repository"
	echo "for example, will also clone (or update) the dependencies: evvis/object_detection,"
	echo "evvis/nie_libraries, evvis/caffe, and evvis/builder."
	echo
	echo "If you are working on a specific JIRA issue with an existing branch on one or more"
	echo "repositories, the script will checkout that branch on all repositories in the hierarchy"
	echo "where that branch exists."
	echo
	echo "Usage: $0 <repository> <branch>"
	echo
	echo "       repository: The name (including project) of the repository you would like to checkout,"
	echo "                   together with its dependencies."
	echo
	echo "       branch:     Optional. The branch to checkout on all the repositories. If given, it"
	echo "                   will checkout this branch name on all repositories. Default: 'master'"
	echo
	echo "Examples:"
	echo
	echo "       $0 evvis/node"
	echo "       $0 evvis/node EVV-700-refactor-node-interface"
}

#Check correct number of arguments
if [ "$#" -lt 1 ]; then
	show_help
	exit 1
fi

#Parse input argument
repo_with_proj=$1
if [ "$#" -lt 2 ]; then
	branch_name="master"
else
	branch_name=$2
fi

#Check that first argument have project name
if [[ "$repo_with_proj" != */* ]]; then 
	show_help
	exit 1
fi

#Get the repo
echo
echo "*******************************"
echo "****** Getting main repo ******"
echo "*******************************"
echo

cd "${0%/*}"/.. #go to the directory that have the builder repo
if [ $? -ne 0 ]; then
	echo "ERROR! Failed to cd to the folder that contains builder repo. Run script with './get_repos.sh' to make sure it works correctly".
	exit 1
fi

clone_or_update_repo $repo_with_proj $branch_name

#Get the repo's dependencies
echo "******************************"
echo "**** Getting dependnecies ****"
echo "******************************"
echo

for dependency in `cat "$repo_name/dependencies.txt"`
do
	dependency=${dependency%%[[:cntrl:]]} #removes trailing \r if exist

	header="Dependency '$dependency'"
	echo $header
	for i in $(seq 1 ${#header}); do printf "="; done #puts a line under the header
	echo; echo

	clone_or_update_repo $dependency $branch_name
done
