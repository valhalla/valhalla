{{/* vim: set filetype=mustache: */}}

{{- define "image" }}
{{- printf "%s/%s:%s" .Values.image.registry .Values.image.repository .Values.image.tag }}
{{- end }}

{{- define "image.secret" }}
{{- printf "{\"auths\": {\"%s\": {\"auth\": \"%s\"}}}" .Values.image.registry (printf "%s:%s" .Values.image.username .Values.image.password | b64enc) | b64enc }}
{{- end }}

{{- define "labels" }}
key: {{ .Values.app.key }}
name: {{ .Values.app.name }}
version: {{ .Values.app.version | quote }}
app: {{ .Chart.Name }}
chart: {{ printf "%s-%s" .Chart.Name .Chart.Version }}
release: {{ .Release.Name }}
heritage: {{ .Release.Service }}
helm.sh/chart: {{ printf "%s-%s" .Chart.Name .Chart.Version }}
app.kubernetes.io/name: {{ .Values.app.name }}
app.kubernetes.io/version: {{ .Values.app.version | quote }}
app.kubernetes.io/instance: {{ .Release.Name }}
app.kubernetes.io/managed-by: {{ .Release.Service }}
tags.datadoghq.com/env: {{ .Values.stage.environment }}
tags.datadoghq.com/service: {{ printf "%s-%s" .Values.app.key .Values.app.name }}
tags.datadoghq.com/version: {{ .Values.app.version | quote }}
{{- end }}

{{- define "annotations" }}
meta.helm.sh/release-name: {{ .Release.Name }}
meta.helm.sh/release-namespace: {{ .Release.Namespace }}
{{- end }}
