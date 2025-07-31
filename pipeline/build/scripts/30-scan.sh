#!/bin/bash
shopt -s nocasematch

# code scanner
if [[ "${SCAN_ENABLE_SONAR}" == true ]]
then
  echo "<<< scan code for improvements >>>"
  keytool -list -keystore ${SONAR_CERT_PATH} -storepass ${SONAR_CERT_PASS}
  docker run \
    --rm \
    -e SONAR_HOST_URL=${SONAR_URL} \
    -e SONAR_LOGIN=${SONAR_TOKEN} \
    -e SONAR_SCANNER_OPTS="-Djavax.net.ssl.keyStore=${SONAR_CERT_PATH} -Djavax.net.ssl.keyStoreType=PKCS12 -Djavax.net.ssl.keyStorePassword=${SONAR_CERT_PASS}" \
    -v "${BUILD_SOURCESDIRECTORY}:/project" \
    -v "${SONAR_CERT_PATH}:${SONAR_CERT_PATH}" \
    sonarsource/sonar-scanner-cli \
    -Dsonar.projectBaseDir=/project \
    -Dsonar.projectKey=MICTNTPRXY \
    -Dsonar.projectName=${APP_NAME} \
    -Dsonar.projectVersion=${APP_VERSION} \
    -Dsonar.branch.name=${BUILD_SOURCEBRANCHNAME} \
    -Dsonar.sources=. \
    -Dsonar.exclusions=**/*_test.go \
    -Dsonar.tests=. \
    -Dsonar.test.inclusions=**/*_test.go
fi

# vulnerability scanner
if [[ "${SCAN_ENABLE_TRIVY}" = true ]]
then
    echo "<<< scan container for vulnerabilities >>>"
    if [[ ! -f "${SCAN_ENABLE_TRIVY_CACHE}/db/db.tar.gz" ]]
    then
        oras pull ghcr.io/aquasecurity/trivy-db:2
        mkdir -p "${SCAN_ENABLE_TRIVY_CACHE}/db"
        mv db.tar.gz "${SCAN_ENABLE_TRIVY_CACHE}/db"
        ( cd "${SCAN_ENABLE_TRIVY_CACHE}/db" && tar xvf db.tar.gz )
    else
        echo 'trivy database found in cache.'
    fi
    
    docker run --rm \
    -v /var/run/docker.sock:/var/run/docker.sock \
    -v ${PWD}/.trivyignore:/.trivyignore:ro \
    -v $HOME/Library/Caches:/root/.cache/ \
    -v ${SCAN_ENABLE_TRIVY_CACHE}/db:/root/.cache/trivy/db \
    aquasec/trivy \
    --cache-dir /root/.cache/trivy \
    --debug image \
    --severity CRITICAL \
    --ignorefile /.trivyignore \
    --ignore-unfixed \
    --skip-db-update \
    --timeout 30m \
    --exit-code 1 \
    ${SHARE_REGISTRY_URL}/${APP_NAME}:${APP_VERSION} \
    || exit 1
fi

# license scanner
if [[ "${SCAN_ENABLE_BLACKDUCK}" == true ]]
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