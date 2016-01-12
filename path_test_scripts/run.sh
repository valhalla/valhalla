#!/bin/bash

function usage() {
	echo "Usage: $0 path_test_request_file [diff_dir]"
        echo "Example: $0 demo_routes.txt"
	echo "Example: $0 demo_routes.txt 20160104_160939_demo_routes"
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
DIFF="${2}"

PATH=../:$PATH ./batch.sh ${INPUT} ${DIFF}

