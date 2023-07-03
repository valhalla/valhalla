#!/bin/bash
set -ex

# Default config
DEFAULT_CONFIG="../../conf/valhalla.json"

function usage() {
	echo "Usage: $0 path_test_request_file [conf=../../conf/valhalla.json]"
	echo "Example: $0 ../test/pinpoints/turn_lanes/right_active_pinpoint.txt"
	echo "Example: $0 ../test/pinpoints/turn_lanes/right_active_pinpoint.txt ../conf/valhalla.json"
	exit 1
}

#set the input file and verify
if [ -z "${1}" ]; then
	echo "Missing path_test_request_file"
	usage
elif [ ! -f "${1}" ]; then
	echo "Invalid path_test_request_file: ${1}"
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

# Set env to save the path pbf and run the specified route
export SAVE_PATH_PBF=true
cat ${INPUT} | xargs -I {} ../build/valhalla_run_route {} --config ${CONF}

###############################################
# Move the path.pbf file to INPUT_FILE_PREFIX.pbf if it exists
INPUT_PATH=$(dirname ${INPUT})
INPUT_BASENAME=$(basename ${INPUT})
INPUT_FILE_PREFIX=${INPUT_BASENAME%.*}
INPUT_FILE_EXT=${INPUT_BASENAME##*.}
#echo path=${INPUT_PATH};echo pref=${INPUT_FILE_PREFIX};echo ext=${INPUT_FILE_EXT}
PBF_IN_PATH="."
PBF_FILE_PREFIX="path"
PBF_FILE_EXT="pbf"
PBF_OUT_PATH=${INPUT_PATH}
if [ -f "${PBF_IN_PATH}/${PBF_FILE_PREFIX}.${PBF_FILE_EXT}" ]
then
   mv ${PBF_IN_PATH}/${PBF_FILE_PREFIX}.${PBF_FILE_EXT} ${PBF_OUT_PATH}/${INPUT_FILE_PREFIX}.${PBF_FILE_EXT}
else
   echo "ERROR: ${PBF_IN_PATH}/${PBF_FILE_PREFIX}.${PBF_FILE_EXT} not found."
fi

