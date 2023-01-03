#!/bin/bash

if [[ -z "${APP_SALT}" ]]
then
  APP_SALT="salt"
fi

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

APP_STORAGE_NAME=${APP_STAGE//-}
APP_STORAGE_RESOURCE_GROUP=${APP_STAGE}-terraform-rg
if [[ ${REGION} == "china" ]]
then
  APP_STORAGE_ENVIRONMENT=china
else
  APP_STORAGE_ENVIRONMENT=public
fi

az group create \
--subscription ${APP_SUBSCRIPTION_ID} \
--location ${APP_LOCATION} \
--name ${APP_STORAGE_RESOURCE_GROUP}

az storage account create \
--subscription ${APP_SUBSCRIPTION_ID} \
--name ${APP_STORAGE_NAME} \
--resource-group ${APP_STORAGE_RESOURCE_GROUP} \
--location ${APP_LOCATION} \
--kind BlobStorage \
--access-tier Hot \
--sku Standard_GRS

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

export TF_VAR_aks_subscription_id=${AKS_SUBSCRIPTION_ID}
export TF_VAR_aks_client_id=${AKS_CLIENT_ID}
export TF_VAR_aks_client_secret=${AKS_CLIENT_SECRET}
export TF_VAR_aks_tenant_id=${AKS_TENANT_ID}
export TF_VAR_aks_stage=${AKS_STAGE}
export TF_VAR_aks_namespace=${AKS_NAMESPACE}
export TF_VAR_aks_scp_cluster=${AKS_SCP_CLUSTER}
export TF_VAR_app_subscription_id=${APP_SUBSCRIPTION_ID}
export TF_VAR_app_client_id=${APP_CLIENT_ID}
export TF_VAR_app_client_secret=${APP_CLIENT_SECRET}
export TF_VAR_app_tenant_id=${APP_TENANT_ID}
export TF_VAR_app_stage=${APP_STAGE}
export TF_VAR_app_location=${APP_LOCATION}
export TF_VAR_app_key=${APP_KEY}
export TF_VAR_app_name=${APP_NAME}
export TF_VAR_app_salt=${APP_SALT}
export TF_VAR_app_geo=${APP_GEO}

shopt -s nocasematch
echo "Create Datadog Monitors"
export TF_VAR_datadog_api_key=${DATADOG_API_KEY}
export TF_VAR_datadog_app_key=${DATADOG_APP_KEY}
export TF_VAR_app_geo=${APP_GEO}

find . -name "*.tfstate" -type f -delete
find . -name "*.tfstate.backup" -type f -delete

# set git credentials
git config --global url."https://daimler-mic:${SYSTEM_ACCESSTOKEN}@dev.azure.com".insteadOf https://dev.azure.com
git config --global credential.helper store

terraform --version
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
plan -detailed-exitcode -no-color -input=false
datadogPlanStatus=$?
if [[ $datadogPlanStatus == 1 ]]
then
  exit 7
fi

if [[ $datadogPlanStatus != 0 ]]
then
  terraform \
  -chdir=terraform/stages/${REGION}.${ENVIRONMENT}/datadog \
  apply -auto-approve -no-color -input=false \
  || exit 8
else
  echo "Terraform apply for datadog skipped"
fi