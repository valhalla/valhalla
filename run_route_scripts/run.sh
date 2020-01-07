#!/usr/bin/env bash

set -o errexit -o pipefail -o nounset

# Default config
DEFAULT_CONFIG="../../conf/valhalla.json"

function usage() {
	echo "Usage: $0 path_test_request_file [conf=../../conf/valhalla.json]"
        echo "Example: $0 requests/demo_routes.txt"
	echo "Example: $0 ../test_requests/demo_routes.txt ~/valhalla.json"
	exit 1
}

#set the input file and verify
if [ -z "${1:-}" ]; then
	echo "Missing path_test_request_file"
	usage
elif [ ! -f "${1:-}" ]; then
	echo "Invalid path_test_request_file: ${1:-}"
	usage
else
	INPUT="${1}"
fi

#set config variable to second argument or default
CONF=${2:-${DEFAULT_CONFIG}}

# verify config file exists
if [ ! -f "${CONF}" ]; then
	echo "Invalid config file: ${CONF}"
	usage
fi

PATH=../build/:$PATH ./batch.sh ${INPUT} ${CONF}

