#!/bin/bash
set -e

tag=$(git tag | tail -n1)
patch=$(echo ${tag} | awk -F. '{print $3}')
((patch += 1 ))
tag=$(echo ${tag} | sed -e "s/[0-9]\+$/${patch}/g")
printf "${tag}"
