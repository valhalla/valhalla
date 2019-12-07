#!/usr/bin/env bash
set -e

# get all the ppas we might need
apt-add-repository -y ppa:valhalla-core/valhalla
apt-get update -y

apt-get install -y \
  libvalhalla0 \
  libvalhalla-dev \
  valhalla-bin

add-apt-repository ppa:valhalla-core/opentraffic
apt-get update -y

apt-get install -y \
  osmlr
