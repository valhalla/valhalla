#!/bin/bash

if [[ -z ${REGION} ]]
then
  echo "ERROR: REGION not defined." && exit 1
fi

if [[ -z ${ENVIRONMENT} ]]
then
  echo "ERROR: ENVIRONMENT not defined." && exit 1
fi

if [[ -z ${NAMESPACE} ]]
then
  echo "ERROR: NAMESPACE not defined." && exit 1
fi

if [[ -z ${APP_KEY} ]]
then
  echo "ERROR: APP_KEY not defined." && exit 1
fi

if [[ -z ${APP_NAME} ]]
then
  echo "ERROR: APP_NAME not defined." && exit 1
fi

if [[ -z ${APP_VERSION} ]]
then
  echo "ERROR: APP_VERSION not defined." && exit 1
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

if [[ -z ${APP_CLIENT_ID} ]]
then
  echo "ERROR: APP_CLIENT_ID not defined." && exit 1
fi

if [[ -z ${APP_CLIENT_SECRET} ]]
then
  echo "ERROR: APP_CLIENT_SECRET not defined." && exit 1
fi

if [[ -z ${APP_SUBSCRIPTION_ID} ]]
then
  echo "ERROR: APP_SUBSCRIPTION_ID not defined." && exit 1
fi

if [[ -z ${APP_TENANT_ID} ]]
then
  echo "ERROR: APP_TENANT_ID not defined." && exit 1
fi

if [[ -z ${APP_LOCATION} ]]
then
  echo "ERROR: APP_LOCATION not defined." && exit 1
fi

if [[ -z ${APP_STAGE} ]]
then
  echo "ERROR: APP_STAGE not defined." && exit 1
fi

#if [[ -z ${AKS_CLIENT_ID} ]]
#then
#  echo "ERROR: AKS_CLIENT_ID not defined." && exit 1
#fi

#if [[ -z ${AKS_CLIENT_SECRET} ]]
#then
#  echo "ERROR: AKS_CLIENT_SECRET not defined." && exit 1
#fi

#if [[ -z ${AKS_SUBSCRIPTION_ID} ]]
#then
#  echo "ERROR: AKS_SUBSCRIPTION_ID not defined." && exit 1
#fi

#if [[ -z ${AKS_TENANT_ID} ]]
#then
#  echo "ERROR: AKS_TENANT_ID not defined." && exit 1
#fi

#if [[ -z ${AKS_STAGE} ]]
#then
#  echo "ERROR: AKS_STAGE not defined." && exit 1
#fi

#if [[ -z "${DATADOG_API_KEY}" ]]
#then
#  echo "ERROR DATADOG_API_KEY variable not defined." && exit 1
#fi

#if [[ -z "${DATADOG_APP_KEY}" ]]
#then
#  echo "ERROR DATADOG_APP_KEY variable not defined." && exit 1
#fi

#if [[ -z "${VALHALLA_GRAPH}" ]]
#then
#  echo "ERROR GRAPH_PATH variable not defined." && exit 1
#fi

if [[ -z "${GRAPH_SHARE_NAME}" ]]
then
  echo "ERROR GRAPH_SHARE_NAME variable not defined." && exit 1
fi

if [[ -z "${DATADOG_API_KEY}" ]]
then
  echo "ERROR DATADOG_API_KEY variable not defined." && exit 1
fi

if [[ -z "${DATADOG_APP_KEY}" ]]
then
  echo "ERROR DATADOG_APP_KEY variable not defined." && exit 1
fi
