#!/bin/bash

if [[ ${BUILD_SOURCEBRANCH} == "refs/heads/release/"* ]]
then
  # Release branch versions look like this: SEMANTIC_VERSION-GIT_HASH (e.g. 1.0.5-c294b92)
  echo "This is a release branch"
  RELEASE_VERSION=${BUILD_SOURCEBRANCHNAME}
  VERSION=${RELEASE_VERSION}-$(git rev-parse --short ${BUILD_SOURCEVERSION})
  LATEST=true
elif [[ ${BUILD_SOURCEBRANCH} == *"main" ]] || [[ ${BUILD_SOURCEBRANCH} == *"master" ]]
then
  # Master/main branch versions look like this: BUILD_NUMBER-GIT_HASH (e.g. 20220124.2-c294b92)
  echo "This is the main branch"
  VERSION=${BUILD_BUILDNUMBER}-$(git rev-parse --short ${BUILD_SOURCEVERSION})
  LATEST=true
else
  # All other branch versions look like this: BUILD_NUMBER-GIT_HASH.BRANCH_NAME (e.g. 20220124.2-c294b92.pipeline-development)
  echo "This is branch ${BUILD_SOURCEBRANCHNAME}"
  ORIGINAL_BRANCH_NAME=${BUILD_SOURCEBRANCHNAME}
  BRANCH_NAME=${ORIGINAL_BRANCH_NAME//_/-}
  VERSION=${BUILD_BUILDNUMBER}-$(git rev-parse --short ${BUILD_SOURCEVERSION}).${BRANCH_NAME}
  LATEST=false
fi

echo "Current build version: ${VERSION}"
echo "##vso[build.updatebuildnumber]${VERSION}"
echo "##vso[task.setvariable variable=app.version;isoutput=true]${VERSION}"
echo "##vso[task.setvariable variable=app.version.latest]${LATEST}"
