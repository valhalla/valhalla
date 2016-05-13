#!/bin/bash

function usage() {
	echo "Usage: $0 path_test_request_file [conf=../../conf/valhalla.json]"
        echo "Example: $0 demo_routes.txt"
	echo "Example: $0 demo_routes.txt ~/valhalla.json"
	exit 1
}

#get the input file
if [ -z "requests/${1}" ]; then
	usage
elif [ ! -f "requests/${1}" ]; then
	usage
else
	INPUT="${1}"
fi

#diffing or not, default to not
if [ -z "${2}" ]; then
	CONF="../../conf/valhalla.json"
else
	CONF="${2}"
fi

PATH=../:$PATH ./batch.sh ${INPUT} ${CONF}

