#!/usr/bin/env bash
# Script to run a single test

set -o pipefail -o nounset

TEST_PATH="$1"
TEST_NAME="$(basename "$TEST_PATH")"
TEST_DIR="$(dirname "$TEST_PATH")"
TEST_LOG="${TEST_DIR}/${TEST_NAME}.log"

# run the test
${TEST_PATH} >& "${TEST_LOG}"
exit=$?

if [[ $exit -eq 0 ]]; then
  echo [PASS] ${TEST_NAME}
else
  echo [FAIL] ${TEST_NAME}
  cat ${TEST_LOG}
  exit $exit
fi
