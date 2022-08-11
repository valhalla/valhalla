#!/bin/bash

if [[ -z ${APP_KEY} ]]
then
  echo "ERROR: APP_KEY not defined." && exit 1
fi

if [[ -z ${APP_NAME} ]]
then
  echo "ERROR: APP_NAME not defined." && exit 1
fi

if [[ -z ${APP_REGISTRY_URL} ]]
then
  echo "ERROR: APP_REGISTRY_URL not defined." && exit 1
fi

if [[ -z ${APP_REGISTRY_USERNAME} ]]
then
  echo "ERROR: APP_REGISTRY_USERNAME not defined." && exit 1
fi

if [[ -z ${APP_REGISTRY_PASSWORD} ]]
then
  echo "ERROR: APP_REGISTRY_PASSWORD not defined." && exit 1
fi

if [[ -z ${SHARED_CLIENT_ID} ]]
then
  echo "ERROR: SHARED_CLIENT_ID not defined." && exit 1
fi

if [[ -z ${SHARED_CLIENT_SECRET} ]]
then
  echo "ERROR: SHARED_CLIENT_SECRET not defined." && exit 1
fi

if [[ -z ${SHARED_SUBSCRIPTION_ID} ]]
then
  echo "ERROR: SHARED_SUBSCRIPTION_ID not defined." && exit 1
fi

if [[ -z ${SHARED_TENANT_ID} ]]
then
  echo "ERROR: SHARED_TENANT_ID not defined." && exit 1
fi