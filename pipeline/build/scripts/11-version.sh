#!/bin/bash

# disable case sensitive comparison
shopt -s nocasematch

echo "<<< generate version for applicaton >>>"
if [[ ${BUILD_SOURCEBRANCH} == "refs/heads/release/"* ]]
then
  # release branch versions look like this: SEMANTIC_VERSION-GIT_HASH (e.g. 1.0.5-c294b92)
  echo "<<< this is a release branch >>>"
  RELEASE_VERSION=${BUILD_SOURCEBRANCHNAME}
  VERSION=${RELEASE_VERSION}-$(git rev-parse --short ${BUILD_SOURCEVERSION})
  LATEST=true
elif [[ ${BUILD_SOURCEBRANCH} == *"main" ]] || [[ ${BUILD_SOURCEBRANCH} == *"master" ]]
then
  # master/main branch versions look like this: BUILD_NUMBER-GIT_HASH (e.g. 20220124.2-c294b92)
  echo "<<< this is the main branch >>>"
  VERSION=${BUILD_BUILDNUMBER}-$(git rev-parse --short ${BUILD_SOURCEVERSION})
  LATEST=true
else
  # all other branch versions look like this: BUILD_NUMBER-GIT_HASH.BRANCH_NAME (e.g. 20220124.2-c294b92.pipeline-development)
  echo "<<< this is branch ${BUILD_SOURCEBRANCHNAME} >>>"
  ORIGINAL_BRANCH_NAME=${BUILD_SOURCEBRANCHNAME}
  BRANCH_NAME=${ORIGINAL_BRANCH_NAME//_/-}
  VERSION=${BUILD_BUILDNUMBER}-$(git rev-parse --short ${BUILD_SOURCEVERSION}).${BRANCH_NAME}
  LATEST=false
fi

# combine the two boolean variables with different syntax into a new variable
if [[ "${APP_VERSION_LATEST}" = true ]] && [[ "$LATEST" = true ]]
then
    LATEST=true
else
    LATEST=false
fi

echo "<<< current build version: ${VERSION} >>>"
echo "<<< latest build version: $LATEST >>>"