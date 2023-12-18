#!/bin/bash

curl -s -L https://detect.synopsys.com/detect9.sh | \
bash -s - \
  --blackduck.url=${BLACKDUCK_PLATFORM_URL} \
  --blackduck.api.token=${BLACKDUCK_API_TOKEN} \
  --detect.project.name=${BLACKDUCK_PROJECT_PREFIX}${APP_NAME} \
  --detect.project.version.name=${APP_VERSION} \
  --detect.project.codelocation.unmap=true \
  --detect.timeout=36000
