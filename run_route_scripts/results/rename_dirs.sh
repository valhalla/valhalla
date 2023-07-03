#!/usr/bin/env bash

set -o errexit -o pipefail -o nounset

function usage() {
    echo "Usage: $0 <DIECTORY_LIST_FILENAME> <TAG>"
    echo "Example: $0 dirs.txt \"MASTER\""
    exit 1
}

# Verify arguments exist
if [ $# -ne 2 ]
then
  echo "Must supply one file with a list of directories and the tag name to append to each directory"
  usage
fi

# Verify file exists
DIRS_FILE=${1}
if [ ! -f "${DIRS_FILE}" ]
then
  echo "${DIRS_FILE} does not exist"
  usage
fi

TAG=${2}
echo "Appending the following tag: ${TAG}"

DIRS_LIST=`cat ${DIRS_FILE}`

for DIR in ${DIRS_LIST}; do
  echo ${DIR}_${TAG}
  mv ${DIR} ${DIR}_${TAG}
done

