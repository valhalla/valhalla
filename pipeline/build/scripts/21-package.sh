#!/bin/bash
shopt -s nocasematch

echo ${SHARE_REGISTRY_PASSWORD} | \
docker login ${SHARE_REGISTRY_URL} \
--username ${SHARE_REGISTRY_USERNAME} \
--password-stdin \
|| exit 1

# To allow debugging of a C++ application it needs to be build with special flags (see Dockerfile for details)
DOCKER_TAG=""
if [[ "${BUILD_DEBUG_MODE}" == true ]]
then
  echo "WARNING! Enabling debug mode and assertions for this build"
  DOCKER_TAG="-debug -assertions"
fi

if [[ "${APP_VERSION_LATEST}" = true ]]
then
  echo "build application with tag: ${APP_NAME}:${APP_VERSION}"
  echo "build application with tag: latest"
  docker build . --file "Dockerfile" \
  --tag ${SHARE_REGISTRY_URL}/${APP_NAME}:${APP_VERSION} \
  --tag ${SHARE_REGISTRY_URL}/${APP_NAME}:latest \
  --build-arg DOCKER_TAG="${DOCKER_TAG}" \
  || exit 2
else
  echo "build application with tag: ${APP_NAME}:${APP_VERSION}"
  docker build . --file "Dockerfile" \
  --tag ${SHARE_REGISTRY_URL}/${APP_NAME}:${APP_VERSION} \
  --build-arg DOCKER_TAG="${DOCKER_TAG}" \
  || exit 2
fi

# push image to Azure Container Registry
echo "<<< push container to container registry >>>"
docker push -a ${SHARE_REGISTRY_URL}/${APP_NAME} \
|| exit 3