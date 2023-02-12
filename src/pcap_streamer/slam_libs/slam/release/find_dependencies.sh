# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
function readlink_fullpath
{
  path=$(readlink "$1")
  if [ $? -ne 0 ] ; then
    return 1
  fi

  case "$path" in
    /*) echo "$path" ;;
    *)  echo "$(dirname "$1")/$path" ;;
  esac

  return 0
}


for path in $(ldd "$@" | grep "=> /" | awk '{print $3}')
do
  echo "$path"
  while path=$(readlink_fullpath "$path"); do
    echo "$path"
  done
done
