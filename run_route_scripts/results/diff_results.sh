#!/bin/bash

# Verify argument exists
if [ $# -ne 2 ]
then
  echo "Must supply both directories"
  exit 1
fi

# Verify directory exists
OLD_DIR=${1}
if [ ! -d "${OLD_DIR}" ]
then
  echo "${OLD_DIR} does not exist"
  exit 1
fi
# strip trailing fwd slash (eg: if you used tab completion for dir name)
OLD_DIR=${OLD_DIR%/}

# Verify directory exists
NEW_DIR=${2}
if [ ! -d "${NEW_DIR}" ]
then
  echo "${NEW_DIR} does not exist"
  exit 1
fi
NEW_DIR=${NEW_DIR%/}

# Verify diff directory does not alreadys exist
DIFF_DIR=$(echo "${OLD_DIR}_${NEW_DIR}_diff" | sed -e "s@/@_@g")
if [ -d "${DIFF_DIR}" ]
then
  echo "${DIFF_DIR} already exists"
  exit 1
fi

CONCURRENCY=$(nproc)
echo -e "\x1b[32;1mDiffing the output of ${OLD_DIR} with ${NEW_DIR} to ${DIFF_DIR}\x1b[0m"
mkdir -p "${DIFF_DIR}"
find ${OLD_DIR}/*.txt -printf "%f\n" | parallel --progress -P "${CONCURRENCY}" "diff ${OLD_DIR}/{} ${NEW_DIR}/{} > ${DIFF_DIR}/{}"
                                                                                                       
COMBINED_STATS_FILE="combined_statistics.csv"
echo -e "\x1b[32;1mGenerating combined statistics at ${DIFF_DIR}/${COMBINED_STATS_FILE}\x1b[0m"
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
${DIR}/combine_route_stats.py ${OLD_DIR}/statistics.csv ${NEW_DIR}/statistics.csv ${DIFF_DIR}/${COMBINED_STATS_FILE}
