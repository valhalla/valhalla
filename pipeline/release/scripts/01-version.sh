#!/bin/bash

if [ -z "$APP_VERSION" ]
then
  echo "Current build version empty, setting to latest."
  APP_VERSION="latest"
else
  echo "Current build version already set to: ${APP_VERSION}"
fi
