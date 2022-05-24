#!/usr/bin/env bash

set -o errexit -o pipefail -o nounset

# For each dockerfile, run hadolint on it.
find ../* -name Dockerfile-* -exec \
    sh -c 'src=${1#./} && { set -x && docker run --rm -i hadolint/hadolint hadolint "$1"; }' sh "{}" \;
