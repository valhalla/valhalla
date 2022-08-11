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

APP_STORAGE_NAME=${APP_STAGE_SHORT}
APP_STORAGE_RESOURCE_GROUP=${APP_STAGE}-terraform-statefile-rg
if [[ ${REGION} == "china" ]]
then
  APP_STORAGE_ENVIRONMENT=china
else
  APP_STORAGE_ENVIRONMENT=public
fi

APP_STORAGE_KEY=$(az storage account keys list \
--subscription ${APP_SUBSCRIPTION_ID} \
--resource-group ${APP_STORAGE_RESOURCE_GROUP} \
--account-name ${APP_STORAGE_NAME} \
--query [0].value -o tsv)
if [[ $? -ne 0 ]]
then
  echo "ERROR: Failed to read Azure storage access key."
  echo "Create a storage account in your subscription in:"
  echo "resource-group: ${APP_STORAGE_RESOURCE_GROUP}"
  echo "name: ${APP_STORAGE_NAME}"
  exit 2
fi

az storage container create \
--subscription ${APP_SUBSCRIPTION_ID} \
--account-name ${APP_STORAGE_NAME} \
--account-key ${APP_STORAGE_KEY} \
--name datadog

az logout

export ARM_ENVIRONMENT=public
if [[ ${REGION} == "china" ]]
then
  export ARM_ENVIRONMENT=china
fi
export ARM_SUBSCRIPTION_ID=${APP_SUBSCRIPTION_ID}
export ARM_CLIENT_ID=${APP_CLIENT_ID}
export ARM_CLIENT_SECRET=${APP_CLIENT_SECRET}
export ARM_TENANT_ID=${APP_TENANT_ID}
export ARM_ACCESS_KEY=${APP_STORAGE_KEY}

export TF_VAR_app_stage=${APP_STAGE}
export TF_VAR_app_key=${APP_KEY}
export TF_VAR_app_name=${APP_NAME}
export TF_VAR_app_geo=${APP_GEO}

export TF_VAR_datadog_api_key=${DATADOG_API_KEY}
export TF_VAR_datadog_app_key=${DATADOG_APP_KEY}

export TF_VAR_recipients=${DATADOG_RECIPIENTS}

find . -name "*.tfstate" -type f -delete
find . -name "*.tfstate.backup" -type f -delete

terraform \
-chdir=terraform/stages/${REGION}.${ENVIRONMENT}/datadog \
init -no-color \
-backend-config="environment=${APP_STORAGE_ENVIRONMENT}" \
-backend-config="storage_account_name=${APP_STORAGE_NAME}" \
-backend-config="container_name=datadog" \
-backend-config="key=${APP_NAME}.tfstate" \
|| exit 6

terraform \
-chdir=terraform/stages/${REGION}.${ENVIRONMENT}/datadog \
plan -no-color -input=false \
|| exit 7

terraform \
-chdir=terraform/stages/${REGION}.${ENVIRONMENT}/datadog \
apply -auto-approve -no-color -input=false \
|| exit 8