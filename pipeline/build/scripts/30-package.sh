#!/bin/bash

echo ${APP_REGISTRY_PASSWORD} | \
docker login ${APP_REGISTRY_URL} \
--username ${APP_REGISTRY_USERNAME} \
--password-stdin \
|| exit 1


if ! docker manifest inspect ${APP_REGISTRY_URL}/valhalla:latest > /dev/null;
then
   echo "base image not present, building it now"
   docker build --file "docker/base/Dockerfile" --tag ${APP_REGISTRY_URL}/valhalla:latest .
   docker push ${APP_REGISTRY_URL}/valhalla:latest
fi

DOCKER_TAG=""
# To allow debugging of a C++ application it needs to be build with special flags (see Dockerfile for details)
shopt -s nocasematch
if [[ "${BUILD_DEBUG_MODE}" == true ]]
then
  echo "WARNING! Enabling debug mode and assertions for this build"
  DOCKER_TAG="-debug -assertions"
fi

if [[ "${APP_VERSION_LATEST}" = true ]]
then
  docker build . --file "docker/Dockerfile" \
  --tag ${APP_REGISTRY_URL}/${APP_NAME}:${APP_VERSION} \
  --tag ${APP_REGISTRY_URL}/${APP_NAME}:latest \
  --build-arg DOCKER_TAG="${DOCKER_TAG}" \
  || exit 2
else
  docker build . --file "docker/Dockerfile" \
  --tag ${APP_REGISTRY_URL}/${APP_NAME}:${APP_VERSION} \
  --build-arg DOCKER_TAG="${DOCKER_TAG}" \
  || exit 2
fi

shopt -s nocasematch
if [[ "${TRIVY_ENABLED}" = true ]]
then
    # scan image for vulnerabilities
    docker run --rm \
    -v /var/run/docker.sock:/var/run/docker.sock \
    -v ${PWD}/.trivyignore:/.trivyignore:ro \
    aquasec/trivy:0.23.0 \
    --debug \
    image \
    --severity CRITICAL \
    --ignorefile /.trivyignore \
    --ignore-unfixed \
    --timeout 10m \
    --exit-code 1 \
    ${APP_REGISTRY_URL}/${APP_NAME}:${APP_VERSION} \
    || exit 4
fi

docker push -a ${APP_REGISTRY_URL}/${APP_NAME} \
|| exit 5
