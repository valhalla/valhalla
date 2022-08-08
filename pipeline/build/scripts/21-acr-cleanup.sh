#!/bin/bash

PURGE_CMD="acr purge \
  --filter '${APP_NAME}:.*' \
  --ago 7d --keep 500"

if [[ ${REGION} == "china" ]]
then
  az cloud set --name AzureChinaCloud
else
  az cloud set --name AzureCloud
fi

az login \
--service-principal \
--tenant ${SHARED_TENANT_ID} \
--username ${SHARED_CLIENT_ID} \
--password ${SHARED_CLIENT_SECRET} \
--output table \
|| exit 1

az acr task create --name PurgeTask \
  --cmd "$PURGE_CMD" \
  --schedule "0 1 * * Sun" \
  --registry ${APP_REGISTRY_USERNAME} \
  --context /dev/null \
  --subscription ${SHARED_SUBSCRIPTION_ID} \
|| exit 2

az logout