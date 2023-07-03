#!/usr/bin/env bash

set -o errexit -o pipefail -o nounset

function usage() {
  echo "Usage: $0 [file_of_requests=request_files.txt] [conf=../../conf/valhalla.json]"
  echo "Example: $0"
  echo "Example: $0 my_own_files.txt"
  echo "Example: $0 request_files.txt ~/valhalla.json"
  exit 1
}

# Default file of requests
DEFAULT_REQUESTS="request_files.txt"

# set requests variable to first argument or default
REQUESTS=${1:-${DEFAULT_REQUESTS}}

# verify file of requeets exists
if [ ! -f "${REQUESTS}" ]; then
  echo "Invalid file of requests: ${REQUESTS}"
  usage
fi

# Default config
DEFAULT_CONFIG="../../conf/valhalla.json"

# set config variable to second argument or default
CONFIG=${2:-${DEFAULT_CONFIG}}

# verify config file exists
if [ ! -f "${CONFIG}" ]; then
  echo "Invalid config file: ${CONFIG}"
  usage
fi

REQUEST_FILES=`cat ${REQUESTS}`

for i in ${REQUEST_FILES}; do
  echo "Processing ${i}..."
  ./run.sh "../test_requests/${i}" "${CONFIG}"
done

