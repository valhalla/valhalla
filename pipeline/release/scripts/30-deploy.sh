#!/bin/bash

if [[ ${REGION} == "china" ]]
then
  az cloud set --name AzureChinaCloud
else
  az cloud set --name AzureCloud
fi

az login \
--service-principal \
--tenant ${APP_TENANT_ID} \
--username ${APP_CLIENT_ID} \
--password ${APP_CLIENT_SECRET} \
--output table \
|| exit 1

KEYVAULT_NAME=${APP_STAGE}-keyvault

GRAPH_STORAGE_NAME=$(az keyvault secret show \
--name graph-storage-name \
--vault-name ${KEYVAULT_NAME} \
--subscription ${APP_SUBSCRIPTION_ID} \
--query value \
--output tsv)
GRAPH_STORAGE_KEY=$(az keyvault secret show \
--name graph-storage-key \
--vault-name ${KEYVAULT_NAME} \
--subscription ${APP_SUBSCRIPTION_ID} \
--query value \
--output tsv)
if [[ $? -ne 0 ]]
then
  echo "ERROR Failed to read secrets from key vault ${APP_STAGE}-keyvault. Make sure the key vault exists."
  exit 3
fi

az logout

az login \
--service-principal \
--tenant ${APP_TENANT_ID} \
--username ${APP_CLIENT_ID} \
--password ${APP_CLIENT_SECRET} \
--output table \
|| exit 2


echo "getting aks credentials"


az aks get-credentials \
--resource-group ${APP_STAGE}-k8s-rg \
--name ${APP_STAGE}-k8s \
--subscription ${APP_SUBSCRIPTION_ID} \
|| exit 3

kubectl get namespace ${NAMESPACE} \
|| kubectl create namespace ${NAMESPACE}

TIMEOUT=20m0s
if [[ ${REGION} == "china" ]]
then
  TIMEOUT=10m0s
fi

echo "appVersion: ${APP_VERSION}" >> helm/app/Chart.yaml \
&& cat helm/app/Chart.yaml

helm version

helm upgrade valhalla helm/app \
--wait \
--install \
--timeout ${TIMEOUT} \
--namespace ${NAMESPACE} \
--values helm/stages/${REGION}.${ENVIRONMENT}/values.yaml \
--set stage.environment=${ENVIRONMENT} \
--set stage.region=${REGION} \
--set app.key=${APP_KEY} \
--set app.name=valhalla \
--set app.version=${APP_VERSION} \
--set image.registry=${APP_REGISTRY_URL} \
--set image.username=${APP_REGISTRY_USERNAME} \
--set image.password=${APP_REGISTRY_PASSWORD} \
--set image.repository=${APP_NAME} \
--set image.tag=${APP_VERSION} \
--set graph.share.name=${GRAPH_SHARE_NAME} \
--set graph.storage.name=${GRAPH_STORAGE_NAME} \
--set graph.storage.key=${GRAPH_STORAGE_KEY} \
|| exit 5

az logout
