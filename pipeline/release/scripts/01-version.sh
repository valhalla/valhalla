#!/bin/bash

if [ -z "$APP_VERSION" ]
then
  echo "Current build version: latest"
  echo "##vso[task.setvariable variable=app.version]latest"
fi
