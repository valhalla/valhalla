#!/bin/bash
shopt -s nocasematch

# license scanner
if [[ "${BLACKDUCK_ENABLED}" == true ]]
then
  # check which phase should be set for the scan (DEVELOPMENT, PLANNING, PRERELEASE, RELEASED, ARCHIVED, DEPRECATED)
  BLACKDUCK_PHASE=PLANNING
  if [[ "${APP_VERSION_RELEASE}" == true ]]
  then
    if [[ "${RUN_RELEASE}" == true ]]
    then
      BLACKDUCK_PHASE=RELEASED
    fi
  else
    BLACKDUCK_PHASE=DEPRECATED
  fi

  # login into Azure Container Registry
  echo "<<< login into container regsitry >>>"
  echo ${SHARE_REGISTRY_PASSWORD} | \
  docker login ${SHARE_REGISTRY_URL} \
    --username ${SHARE_REGISTRY_USERNAME} \
    --password-stdin \
  || exit 2
  
  # build special container which includes all dependencies
  echo "<<< build container license scan >>>"
  echo "SYSTEM_ACCESSTOKEN=\"$DEVOPS_TOKEN\"" > ./system_accesstoken
  DOCKER_BUILDKIT=1 docker build . --file "Dockerfile" \
    --tag ${SHARE_REGISTRY_URL}/${APP_NAME}:builder \
    --target builder \
    --build-arg DOCKER_TAG="${DOCKER_TAG}" \
    --build-arg SYSTEM_ACCESSTOKEN="${DEVOPS_TOKEN}" \
    --build-arg CONTAINER_REGISTRY="${SHARE_REGISTRY_URL}" \
    --build-arg VALHALLA_VERSION="${VALHALLA_VERSION}" \
    --build-arg BUILD_DEBUG_MODE="${RUN_BUILD_DEBUG}" \
    --secret id=system_accesstoken,src=./system_accesstoken \
  || exit 3

  # scan special container with all depended licenses
  echo "<<< scan container for license issues >>>"
  echo "set phase to $BLACKDUCK_PHASE"
  chmod +x ./pipeline/build/scripts/31-blackduck.sh
  docker run --rm \
    -v $SYSTEM_DEFAULTWORKINGDIRECTORY/pipeline/build/scripts/31-blackduck.sh:/workspace/build/blackduck.sh \
    --env BLACKDUCK_PLATFORM=$BLACKDUCK_PLATFORM_URL \
    --env BLACKDUCK_PREFIX=$BLACKDUCK_PREFIX \
    --env BLACKDUCK_PROJECT=$APP_NAME \
    --env BLACKDUCK_NAME=$APP_VERSION \
    --env BLACKDUCK_SOURCE=/usr/local/src/valhalla/build \
    --env BLACKDUCK_PHASE=$BLACKDUCK_PHASE \
    --env BLACKDUCK_TOKEN=$BLACKDUCK_TOKEN \
    ${SHARE_REGISTRY_URL}/${APP_NAME}:builder \
    /workspace/build/blackduck.sh \
  || exit 4
fi