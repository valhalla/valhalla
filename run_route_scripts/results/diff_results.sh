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

# Verify directory exists
NEW_DIR=${2}
if [ ! -d "${NEW_DIR}" ]
then
  echo "${NEW_DIR} does not exist"
  exit 1
fi

# Verify diff directory does not alreadys exist
DIFF_DIR=$(echo "${OLD_DIR}_${NEW_DIR}_diff" | sed -e "s@/@_@g")
if [ -d "${DIFF_DIR}" ]
then
  echo "${DIFF_DIR} already exists"
  exit 1
fi

CONCURRENCY=$(nproc)
echo -e "\x1b[32;1mDiffing the output of ${OLD_DIR} with ${NEW_DIR} to ${OLD_DIR}_${NEW_DIR}_diff\x1b[0m"
mkdir -p "${DIFF_DIR}"
find ${OLD_DIR}/*.txt -printf "%f\n" | parallel --progress -P "${CONCURRENCY}" "diff ${OLD_DIR}/{} ${NEW_DIR}/{} > ${DIFF_DIR}/{}"
                                                                                                       
exit

