#!/bin/bash

if [ -z "$APP_VERSION" ]
then
  echo '##vso[task.setvariable variable=app.version]latest'
  echo "Current build version empty, setting to latest."
else
  echo "Current build version already set to: ${APP_VERSION}"
fi
