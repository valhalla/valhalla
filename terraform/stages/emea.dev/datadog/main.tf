terraform {
  backend "azurerm" {

  }
}

# --- environment variables ---
variable "app_stage" {}
variable "app_key" {}
variable "app_name" {}
variable "aks_namespace" {}
variable "app_geo" {}
variable "datadog_api_key" {}
variable "datadog_app_key" {}

module "datadog" {
  source          = "../../../datadog"
  mic_stage       = "dev"
  app_key         = var.app_key
  app_name        = var.app_name
  aks_namespace   = var.aks_namespace
  aks_clustername = "${var.app_stage}-k8s"
  app_geo         = var.app_geo
  datadog_api_key = var.datadog_api_key
  datadog_app_key = var.datadog_app_key
  create_dashboards = false
}