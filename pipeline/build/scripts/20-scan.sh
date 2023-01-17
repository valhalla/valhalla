#!/bin/bash

shopt -s nocasematch
if [[ "${SONAR_ENABLED}" == true ]]
then
  keytool -list -keystore ${SONAR_CERT_PATH} -storepass ${SONAR_CERT_PASSPHRASE}
  docker run \
    --rm \
    -e SONAR_HOST_URL=${SONAR_URL} \
    -e SONAR_LOGIN=${SONAR_TOKEN} \
    -e SONAR_SCANNER_OPTS="-Djavax.net.ssl.keyStore=${SONAR_CERT_PATH} -Djavax.net.ssl.keyStoreType=PKCS12 -Djavax.net.ssl.keyStorePassword=${SONAR_CERT_PASSPHRASE}" \
    -v "${BUILD_SOURCESDIRECTORY}:/src" \
    -v "${SONAR_CERT_PATH}:${SONAR_CERT_PATH}" \
    sonarsource/sonar-scanner-cli \
    -Dsonar.projectKey=WAYVALHAL \
    -Dsonar.projectName=${APP_NAME} \
    -Dsonar.projectVersion=${APP_VERSION} \
    -Dsonar.branchName=${BUILD_SOURCEBRANCHNAME}
fi

if [[ "${BLACKDUCK_ENABLED}" == true ]]
then
  curl -s -L https://detect.synopsys.com/detect7.sh | \
  bash -s - \
    --blackduck.url=${BLACKDUCK_URL} \
    --blackduck.api.token=${BLACKDUCK_API_TOKEN} \
    --detect.project.name=MICWAY \
    --detect.project.version.name=${APP_VERSION} \
    --detect.project.codelocation.unmap=true \
    --detect.timeout=36000
fi