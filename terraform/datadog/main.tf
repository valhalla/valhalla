provider "datadog" {
  api_key = var.datadog_api_key
  app_key = var.datadog_app_key
}
# disabled by default for SCP, enable if you manage your own cluster
module "cpu-idle-monitor" {
  source = "git::https://dev.azure.com/daimler-mic/apmtools/_git/datadog-monitor-tf-modules//cpu-idle-monitor?ref=v0.9.6"
  thresholds = {
    warning = "20"
    alert   = "10"
  }
  aks_clustername = var.aks_clustername
  teamname        = "mic-team-way"
  #ToDo: add your recipients here e.g. {warn="@webhook-mattermost-mic-team-<<code>>" alert="@opsgenie-mic-team-<<code>> @webhook-mattermost-mic-team-<<code>>"}
  notifications = {
    warn     = ""
    alert    = ""
    recovery = ""
    default  = ""
  }
  env = var.mic_stage
  geo = var.app_geo
  priority = 3
}

# disabled by default for SCP, enable if you manage your own cluster
module "mem-usable-monitor" {
  source = "git::https://dev.azure.com/daimler-mic/apmtools/_git/datadog-monitor-tf-modules//mem-usable-monitor?ref=v0.9.6"
  thresholds = {
    warning = "15"
    alert   = "10"
  }
  aks_clustername = var.aks_clustername
  teamname        = "mic-team-way"
  #ToDo: add your recipients here e.g. {warn="@webhook-mattermost-mic-team-<<code>>" alert="@opsgenie-mic-team-<<code>> @webhook-mattermost-mic-team-<<code>>"}
  notifications = {
    warn     = ""
    alert    = ""
    recovery = ""
    default  = ""
  }
  env             = var.mic_stage
  geo             = var.app_geo
  priority = 3

}

# disabled by default for SCP, enable if you manage your own cluster
module "disk-free-monitor" {
  source = "git::https://dev.azure.com/daimler-mic/apmtools/_git/datadog-monitor-tf-modules//disk-free-monitor?ref=v0.9.6"
  thresholds = {
    warning = "20"
    alert   = "10"
  }
  aks_clustername = var.aks_clustername
  teamname        = "mic-team-way"
  #ToDo: add your recipients here e.g. {warn="@webhook-mattermost-mic-team-<<code>>" alert="@opsgenie-mic-team-<<code>> @webhook-mattermost-mic-team-<<code>>"}
  notifications = {
    warn     = ""
    alert    = ""
    recovery = ""
    default  = ""
  }
  env = var.mic_stage
  geo = var.app_geo
  priority = 3

}

# disabled by default for SCP, enable if you manage your own cluster
module "k8s-pods-monitor" {
  source = "git::https://dev.azure.com/daimler-mic/apmtools/_git/datadog-monitor-tf-modules//k8s-pods-monitor?ref=v0.9.6"
  thresholds = {
    warning = "3"
    alert   = "5"
  }
  teamname        = "mic-team-way"
  #ToDo: add your recipients here e.g. {warn="@webhook-mattermost-mic-team-<<code>>" alert="@opsgenie-mic-team-<<code>> @webhook-mattermost-mic-team-<<code>>"}
  notifications = {
    warn     = ""
    alert    = ""
    recovery = ""
    default  = ""
  }
  env             = var.mic_stage
  geo             = var.app_geo
  kube_namespace  = var.aks_namespace
  aks_clustername = var.aks_clustername
  priority = 3

}

# disabled by default for SCP, enable if you manage your own cluster
module "k8s-container-cpu-usage-limits-monitor" {
  source = "git::https://dev.azure.com/daimler-mic/apmtools/_git/datadog-monitor-tf-modules//k8s-container-cpu-usage-limits-monitor?ref=v0.9.6"
  thresholds = {
    warning = "85"
    alert   = "90"
  }
  teamname = "mic-team-way"
  #ToDo: add your recipients here e.g. {warn="@webhook-mattermost-mic-team-<<code>>" alert="@opsgenie-mic-team-<<code>> @webhook-mattermost-mic-team-<<code>>"}
  notifications = {
    warn     = ""
    alert    = ""
    recovery = ""
    default  = ""
  }
  env             = var.mic_stage
  geo             = var.app_geo
  kube_namespace  = var.aks_namespace
  aks_clustername = var.aks_clustername
  priority = 3

}

# disabled by default for SCP, enable if you manage your own cluster
module "k8s-container-memory-usage-limits-monitor" {
  source = "git::https://dev.azure.com/daimler-mic/apmtools/_git/datadog-monitor-tf-modules//k8s-container-memory-usage-limits-monitor?ref=v0.9.6"
  thresholds = {
    warning = "85"
    alert   = "90"
  }
  teamname = "mic-team-way"
  #ToDo: add your recipients here e.g. {warn="@webhook-mattermost-mic-team-<<code>>" alert="@opsgenie-mic-team-<<code>> @webhook-mattermost-mic-team-<<code>>"}
  notifications = {
    warn     = ""
    alert    = ""
    recovery = ""
    default  = ""
  }
  env             = var.mic_stage
  geo             = var.app_geo
  kube_namespace  = var.aks_namespace
  aks_clustername = var.aks_clustername
  priority = 3

}

# disabled by default for SCP, enable if you manage your own cluster
module "k8s-container-cpu-usage-requests-monitor" {
  source = "git::https://dev.azure.com/daimler-mic/apmtools/_git/datadog-monitor-tf-modules//k8s-container-cpu-usage-requests-monitor?ref=v0.9.6"
  thresholds = {
    warning = "85"
    alert   = "90"
  }
  teamname = "mic-team-way"
  #ToDo: add your recipients here e.g. {warn="@webhook-mattermost-mic-team-<<code>>" alert="@opsgenie-mic-team-<<code>> @webhook-mattermost-mic-team-<<code>>"}
  notifications = {
    warn     = ""
    alert    = ""
    recovery = ""
    default  = ""
  }
  env             = var.mic_stage
  geo             = var.app_geo
  kube_namespace  = var.aks_namespace
  aks_clustername = var.aks_clustername
  priority = 3

}

# disabled by default for SCP, enable if you manage your own cluster
module "k8s-container-memory-usage-requests-monitor" {
  source = "git::https://dev.azure.com/daimler-mic/apmtools/_git/datadog-monitor-tf-modules//k8s-container-memory-usage-requests-monitor?ref=v0.9.6"
  thresholds = {
    warning = "85"
    alert   = "90"
  }
  teamname = "mic-team-way"
  #ToDo: add your recipients here e.g. {warn="@webhook-mattermost-mic-team-<<code>>" alert="@opsgenie-mic-team-<<code>> @webhook-mattermost-mic-team-<<code>>"}
  notifications = {
    warn     = ""
    alert    = ""
    recovery = ""
    default  = ""
  }
  env             = var.mic_stage
  geo             = var.app_geo
  kube_namespace  = var.aks_namespace
  aks_clustername = var.aks_clustername
  priority = 3

}

# disabled by default for SCP, enable if you manage your own cluster
module "k8s-unschedulable-nodes-monitor" {
  source = "git::https://dev.azure.com/daimler-mic/apmtools/_git/datadog-monitor-tf-modules//k8s-unschedulable-nodes-monitor?ref=v0.9.6"
  thresholds = {
    warning = "90"
    alert   = "80"
  }
  aks_clustername = var.aks_clustername
  teamname        = "mic-team-way"
  #ToDo: add your recipients here e.g. {warn="@webhook-mattermost-mic-team-<<code>>" alert="@opsgenie-mic-team-<<code>> @webhook-mattermost-mic-team-<<code>>"}
  notifications = {
    warn     = ""
    alert    = ""
    recovery = ""
    default  = ""
  }
  env             = var.mic_stage
  geo             = var.app_geo
  priority = 3
}

# disabled by default for SCP, enable if you manage your own cluster
module "k8s-crashloopbackoff-monitor" {
  source = "git::https://dev.azure.com/daimler-mic/apmtools/_git/datadog-monitor-tf-modules//k8s-crashloopbackoff-monitor?ref=v0.9.6"
  teamname = "mic-team-way"
  #ToDo: add your recipients here e.g. {warn="@webhook-mattermost-mic-team-<<code>>" alert="@opsgenie-mic-team-<<code>> @webhook-mattermost-mic-team-<<code>>"}
  notifications = {
    warn     = ""
    alert    = ""
    recovery = ""
    default  = ""
  }
  env             = var.mic_stage
  geo             = var.app_geo
  aks_clustername = var.aks_clustername
  kube_namespace  = var.aks_namespace
  priority = 3
}

# disabled by default for SCP, enable if you manage your own cluster
module "nginx-upstream-monitor" {
  source = "git::https://dev.azure.com/daimler-mic/apmtools/_git/datadog-monitor-tf-modules//nginx-upstream-monitor?ref=v0.9.6"
  app_name = "tools-public-nginx"
  teamname = "mic-team-way"
  #ToDo: add your recipients here e.g. {warn="@webhook-mattermost-mic-team-<<code>>" alert="@opsgenie-mic-team-<<code>> @webhook-mattermost-mic-team-<<code>>"}
  notifications = {
    warn     = ""
    alert    = ""
    recovery = ""
    default  = ""
  }
  env      = var.mic_stage
  geo      = var.app_geo
  aks_clustername = var.aks_clustername
  priority = 3
}

# disabled by default for SCP, enable if you manage your own cluster
module "k8s-requests-cpu-monitor" {
  source = "git::https://dev.azure.com/daimler-mic/apmtools/_git/datadog-monitor-tf-modules//k8s-requests-cpu-monitor?ref=v0.9.6"
  thresholds = {
    warning = "80"
    alert   = "90"
  }
  teamname = "mic-team-way"
  #ToDo: add your recipients here e.g. {warn="@webhook-mattermost-mic-team-<<code>>" alert="@opsgenie-mic-team-<<code>> @webhook-mattermost-mic-team-<<code>>"}
  notifications = {
    warn     = ""
    alert    = ""
    recovery = ""
    default  = ""
  }
  env      = var.mic_stage
  geo      = var.app_geo
  app_name = var.app_name
  app_key  = var.app_key
  aks_clustername = var.aks_clustername
  priority = 3
}

# disabled by default for SCP, enable if you manage your own cluster
module "k8s-requests-memory-monitor" {
  source = "git::https://dev.azure.com/daimler-mic/apmtools/_git/datadog-monitor-tf-modules//k8s-requests-memory-monitor?ref=v0.9.6"
  thresholds = {
    warning = "80"
    alert   = "90"
  }
  teamname = "mic-team-way"
  #ToDo: add your recipients here e.g. {warn="@webhook-mattermost-mic-team-<<code>>" alert="@opsgenie-mic-team-<<code>> @webhook-mattermost-mic-team-<<code>>"}
  notifications = {
    warn     = ""
    alert    = ""
    recovery = ""
    default  = ""
  }
  env      = var.mic_stage
  geo      = var.app_geo
  app_name = var.app_name
  app_key  = var.app_key
  aks_clustername = var.aks_clustername
  priority = 3

}

module "resp-pct90-monitor" {
  source = "git::https://dev.azure.com/daimler-mic/apmtools/_git/datadog-monitor-tf-modules//resp-pct90-monitor?ref=v0.9.6"
  thresholds = {
    warning = "1.5"
    alert   = "2"
  }
  app_key  = var.app_key
  app_name = var.app_name
  teamname = "mic-team-way"
  #ToDo: add your recipients here e.g. {warn="@webhook-mattermost-mic-team-<<code>>" alert="@opsgenie-mic-team-<<code>> @webhook-mattermost-mic-team-<<code>>"}
  notifications = {
    warn     = ""
    alert    = ""
    recovery = ""
    default  = ""
  }
  env      = var.mic_stage
  geo      = var.app_geo
  aks_clustername = var.aks_clustername
  priority = 3
}

module "resp-pct95-monitor" {
  source = "git::https://dev.azure.com/daimler-mic/apmtools/_git/datadog-monitor-tf-modules//resp-pct95-monitor?ref=v0.9.6"
  thresholds = {
    warning = "1.5"
    alert   = "2"
  }
  app_key  = var.app_key
  app_name = var.app_name
  teamname = "mic-team-way"
  #ToDo: add your recipients here e.g. {warn="@webhook-mattermost-mic-team-<<code>>" alert="@opsgenie-mic-team-<<code>> @webhook-mattermost-mic-team-<<code>>"}
  notifications = {
    warn     = ""
    alert    = ""
    recovery = ""
    default  = ""
  }
  env      = var.mic_stage
  geo      = var.app_geo
  aks_clustername = var.aks_clustername
  priority = 3
}

module "errorrate-monitor" {
  source = "git::https://dev.azure.com/daimler-mic/apmtools/_git/datadog-monitor-tf-modules//errorrate-monitor?ref=v0.9.6"
  thresholds = {
    warning = "0.01"
    alert   = "0.05"
  }
  teamname = "mic-team-way"
  #ToDo: add your recipients here e.g. {warn="@webhook-mattermost-mic-team-<<code>>" alert="@opsgenie-mic-team-<<code>> @webhook-mattermost-mic-team-<<code>>"}
  notifications = {
    warn     = ""
    alert    = ""
    recovery = ""
    default  = ""
  }
  env      = var.mic_stage
  geo      = var.app_geo
  app_name = var.app_name
  app_key  = var.app_key
  aks_clustername = var.aks_clustername
  priority = 3
}

module "service-dashboard" {
  source = "git::https://dev.azure.com/daimler-mic/apmtools/_git/datadog-monitor-tf-modules//service-dashboard?ref=v0.9.6"
  count           = var.create_dashboards ? 1 : 0
  cluster_name    = var.aks_clustername
  env             = var.mic_stage
  jvm_enabled     = true
  network_enabled = true
  secondary_primary_tag = {
    "key"   = "geo"
    "value" = var.app_geo
  }
  service = {
    "name"           = var.app_name
    "operation_name" = "servlet.request"
  }
}