#!/usr/bin/env bash

set -o errexit -o pipefail -o nounset

###############################################################################
### This script runs the diff_results.sh script between the specified 
### baseline route directories and the specified latest route directories.
### This script creates a multi diff report with the number of diffs for each
### directory pair as well the actual diffs here:
### $(date +%Y%m%d_%H%M%S)_multi_diff_report.diff

function usage() {
    echo "Usage: $0 baseline.txt latest.txt"
    exit 1
}

# Verify argument exists
if [ $# -ne 2 ]
then
  echo "Must supply two files with a list of directories"
  usage
fi

# Verify file exists
DIRS_BEFORE=${1}
if [ ! -f "${DIRS_BEFORE}" ]
then
  echo "${DIRS_BEFORE} does not exist"
  usage
fi

# Verify file exists
DIRS_AFTER=${2}
if [ ! -f "${DIRS_AFTER}" ]
then
  echo "${DIRS_AFTER} does not exist"
  usage
fi

readarray -t DIRS_BEFORE_LIST < ${DIRS_BEFORE}
DIRS_BEFORE_LIST_COUNT=${#DIRS_BEFORE_LIST[@]}
echo "DIRS_BEFORE_LIST_COUNT=${DIRS_BEFORE_LIST_COUNT}"

readarray -t DIRS_AFTER_LIST < ${DIRS_AFTER}
DIRS_AFTER_LIST_COUNT=${#DIRS_AFTER_LIST[@]}
echo "DIRS_AFTER_LIST_COUNT=${DIRS_AFTER_LIST_COUNT}"

if [[ ${DIRS_BEFORE_LIST_COUNT} -ne ${DIRS_AFTER_LIST_COUNT} ]]
then
  echo "ERROR: ${DIRS_BEFORE} and ${DIRS_AFTER} do not have equal number of items"
  exit 1
fi

readonly SCRIPT_PATH="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
readonly MULTI_DIFF_REPORT="${SCRIPT_PATH}/$(date +%Y%m%d_%H%M%S)_multi_diff_report.diff"

for ((i = 0; i < ${DIRS_BEFORE_LIST_COUNT}; i++)); do
  DIFF_DIR="${DIRS_BEFORE_LIST[$i]}_${DIRS_AFTER_LIST[$i]}_diff"
  echo "" | tee -a "${MULTI_DIFF_REPORT}"
  echo "#################################################################################" | tee -a "${MULTI_DIFF_REPORT}"
  ./diff_results.sh ${DIRS_BEFORE_LIST[$i]} ${DIRS_AFTER_LIST[$i]}
  cd ${DIFF_DIR}
  DIFF_COUNT=`../cnd`
  echo "${DIFF_COUNT} @ ${DIFF_DIR}" | tee -a "${MULTI_DIFF_REPORT}"
  ../catnd >>  "${MULTI_DIFF_REPORT}"
  cd ..
done

