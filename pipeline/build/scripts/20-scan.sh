#!/bin/bash

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