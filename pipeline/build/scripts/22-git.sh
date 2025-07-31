#!/bin/bash

if [[ ! -z "${BUILD_VERSION}" ]]
then
  git tag v$BUILD_VERSION
  git push origin v$BUILD_VERSION
  echo "<<< create new git tag: ${BUILD_VERSION} >>>"
fi