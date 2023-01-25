variable "mic_stage" {
  type = string
}

variable "aks_clustername" {
  type = string
}

variable "app_name" {
  description = "type name of microservice. Example: blueprint-service"
  type        = string
}

variable "app_geo" {
  description = "type name geo. Example: emea"
  type        = string
}

variable "app_key" {
  description = "type key of microservice. Example: blp"
  type        = string
}

variable "datadog_api_key" {
  type = string
}

variable "datadog_app_key" {
  type = string
}

variable "aks_namespace" {
  type = string
}

variable "create_dashboards" {
  default = false
}