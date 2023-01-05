#!/bin/bash

if [ -z "$APP_VERSION" ]
then
  echo "Current build version: latest"
  echo "##vso[task.setvariable variable=app.version;]latest"
else
  echo "Current build version: ${APP_VERSION}"
fi
