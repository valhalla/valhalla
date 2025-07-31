#!/bin/bash

# get latest tag from GIT repsoitory
LATEST_TAG=$(git tag | sort -V | tail -1)
echo "<<< current tag is: $LATEST_TAG >>>"

# split major, minor and revision number from latest tag
major=`echo $LATEST_TAG | sed 's/v//' | cut -d. -f1`
minor=`echo $LATEST_TAG | sed 's/v//' | cut -d. -f2`
revision=`echo $LATEST_TAG | sed 's/v//' | cut -d. -f3`

# check which version number should be increased
if [[ ${SHARED_VERSION_MAJOR} -gt $major ]]
then
  # if global major version has increased, reset minor an revision number
  echo "<<< create major tag >>>"
  major=${SHARED_VERSION_MAJOR}
  minor=`echo 0`
  revision=`echo 0`
elif [[ ${BUILD_TAG} == "feature" ]]
then
  # if local minor version should increase, reset revision number
  echo "<<< create minor tag >>>"
  minor=`expr $minor + 1`
  revision=`echo 0`
else
  # increase revision number
  echo "<<< create revision tag >>>"
  revision=`expr $revision + 1`
fi

# create new tag with the caluclated numbers and push it back to repositoty
TAG_VERSION="$major.$minor.$revision"
echo "<<< new tag is: ${TAG_VERSION} >>>"