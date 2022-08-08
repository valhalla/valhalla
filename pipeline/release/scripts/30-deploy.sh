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

EVENTHUB_URL=$(az keyvault secret show \
--name eventhub-url \
--vault-name ${KEYVAULT_NAME} \
--subscription ${APP_SUBSCRIPTION_ID} \
--query value \
--output tsv)
EVENTHUB_NAME=$(az keyvault secret show \
--name eventhub-matched-traffic-name \
--vault-name ${KEYVAULT_NAME} \
--subscription ${APP_SUBSCRIPTION_ID} \
--query value \
--output tsv)
EVENTHUB_KEYNAME=$(az keyvault secret show \
--name eventhub-matched-traffic-osrm-customize-keyname \
--vault-name ${KEYVAULT_NAME} \
--subscription ${APP_SUBSCRIPTION_ID} \
--query value \
--output tsv)
EVENTHUB_KEY=$(az keyvault secret show \
--name eventhub-matched-traffic-osrm-customize-key \
--vault-name ${KEYVAULT_NAME} \
--subscription ${APP_SUBSCRIPTION_ID} \
--query value \
--output tsv)
EVENTHUB_CONSUMER_GROUP=$(az keyvault secret show \
--name eventhub-osrm-customize-consumergroup \
--vault-name ${KEYVAULT_NAME} \
--subscription ${APP_SUBSCRIPTION_ID} \
--query value \
--output tsv)
EVENTHUB_PARTITIONS=$(az keyvault secret show \
--name eventhub-osrm-customize-partition-count \
--vault-name ${KEYVAULT_NAME} \
--subscription ${APP_SUBSCRIPTION_ID} \
--query value \
--output tsv)
METRIC_STORAGE_CONNECTION=$(az keyvault secret show \
--name metric-storage-connectionstring \
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
--tenant ${SHARED_TENANT_ID} \
--username ${SHARED_CLIENT_ID} \
--password ${SHARED_CLIENT_SECRET} \
--output table \
|| exit 1

KEYVAULT_NAME=${SHARED_STAGE}-keyvault

GRAPH_STORAGE_NAME=$(az keyvault secret show \
--name graph-storage-name \
--vault-name ${KEYVAULT_NAME} \
--subscription ${SHARED_SUBSCRIPTION_ID} \
--query value \
--output tsv)
GRAPH_STORAGE_KEY=$(az keyvault secret show \
--name graph-storage-key \
--vault-name ${KEYVAULT_NAME} \
--subscription ${SHARED_SUBSCRIPTION_ID} \
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
--tenant ${AKS_TENANT_ID} \
--username ${AKS_CLIENT_ID} \
--password ${AKS_CLIENT_SECRET} \
--output table \
|| exit 2

az aks get-credentials \
--resource-group ${AKS_STAGE}-k8s-rg \
--name ${AKS_STAGE}-k8s \
--subscription ${AKS_SUBSCRIPTION_ID} \
|| exit 3

kubectl get namespace ${NAMESPACE} \
|| kubectl create namespace ${NAMESPACE}

TIMEOUT=20m0s
if [[ ${REGION} == "china" ]]
then
  TIMEOUT=10m0s
fi

echo "appVersion: ${APP_VERSION}" >> helm/customize/app/Chart.yaml \
&& cat helm/customize/app/Chart.yaml

helm version

helm upgrade osrm-customize helm/customize/app \
--wait \
--install \
--timeout ${TIMEOUT} \
--namespace ${NAMESPACE} \
--values helm/customize/stages/${REGION}.${ENVIRONMENT}/values.yaml \
--set stage.environment=${ENVIRONMENT} \
--set stage.region=${REGION} \
--set app.key=${APP_KEY} \
--set app.name=osrm-customize \
--set app.version=${APP_VERSION} \
--set eventHub.host=${EVENTHUB_URL} \
--set eventHub.name=${EVENTHUB_NAME} \
--set eventHub.sharedAccessKeyName=${EVENTHUB_KEYNAME} \
--set eventHub.sharedAccessKey=${EVENTHUB_KEY} \
--set eventHub.consumerGroup=${EVENTHUB_CONSUMER_GROUP} \
--set eventHub.numberOfPartitions=${EVENTHUB_PARTITIONS} \
--set image.registry=${APP_REGISTRY_URL} \
--set image.username=${APP_REGISTRY_USERNAME} \
--set image.password=${APP_REGISTRY_PASSWORD} \
--set image.repository=${APP_NAME} \
--set image.tag=${APP_VERSION} \
--set graph.path=${OSRM_BACKEND_GRAPH} \
--set graph.share.name=${GRAPH_SHARE_NAME} \
--set graph.storage.name=${GRAPH_STORAGE_NAME} \
--set graph.storage.key=${GRAPH_STORAGE_KEY} \
--set metrics.connectionString=${METRIC_STORAGE_CONNECTION} \
|| exit 4

echo "appVersion: ${APP_VERSION}" >> helm/router/app/Chart.yaml \
&& cat helm/router/app/Chart.yaml

helm version

helm upgrade osrm-router helm/router/app \
--wait \
--install \
--timeout ${TIMEOUT} \
--namespace ${NAMESPACE} \
--values helm/router/stages/${REGION}.${ENVIRONMENT}/values.yaml \
--set stage.environment=${ENVIRONMENT} \
--set stage.region=${REGION} \
--set app.key=${APP_KEY} \
--set app.name=osrm-router \
--set app.version=${APP_VERSION} \
--set image.registry=${APP_REGISTRY_URL} \
--set image.username=${APP_REGISTRY_USERNAME} \
--set image.password=${APP_REGISTRY_PASSWORD} \
--set image.repository=${APP_NAME} \
--set image.tag=${APP_VERSION} \
--set graph.path=${OSRM_BACKEND_GRAPH} \
--set graph.share.name=${GRAPH_SHARE_NAME} \
--set graph.storage.name=${GRAPH_STORAGE_NAME} \
--set graph.storage.key=${GRAPH_STORAGE_KEY} \
--set metrics.connectionString=${METRIC_STORAGE_CONNECTION} \
|| exit 5

az logout
