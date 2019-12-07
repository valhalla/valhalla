#!/bin/bash
set -e

catch_exception() {
  if [ $? != 0 ]; then
    echo "[FAILURE] Detected non zero exit status while processing transit tiles!"
    exit 1
  fi
}

# this has to exist
if [ -z "${TRANSITLAND_API_KEY}" ]; then
  echo "[ERROR] Environment variable TRANSITLAND_API_KEY is not set. Exiting."
  exit 1
fi

REGION=${REGION:-"us-east-1"}
DATA_DIR="/data/valhalla"

# some defaults, if needed.
export TRANSITLAND_URL=${TRANSITLAND_URL:-"http://transit.land"}
export TRANSIT_TILE_DIR=${TRANSIT_TILE_DIR:-"${DATA_DIR}/transit"}
export TRANSITLAND_PER_PAGE=${TRANSITLAND_PER_PAGE:-5000}
export TRANSITLAND_LEVELS=${TRANSITLAND_LEVELS:-"4"}

# clean up from previous runs
if [ -d "${TRANSIT_TILE_DIR}" ]; then
  echo "[INFO] Removing contents of prior run in ${TRANSIT_TILE_DIR}/*..."
  rm -rf "${TRANSIT_TILE_DIR}"; catch_exception
fi

# create dirs
mkdir -p "${DATA_DIR}"; catch_exception
mkdir -p "${TRANSIT_TILE_DIR}"; catch_exception

# only run the tests for production.
if [ "$TRANSITLAND_URL" == "http://transit.land" ]; then
  wget -q "https://raw.githubusercontent.com/valhalla/valhalla/master/test_requests/transit_acceptance_tests.txt" -O ${DATA_DIR}/transit_acceptance_tests.txt; catch_exception
  TRANSIT_TEST_FILE=${DATA_DIR}/transit_acceptance_tests.txt
fi

# for now....build the timezones.
echo "[INFO] Building timezones... "
valhalla_build_timezones conf/valhalla.json; catch_exception

# build transit tiles
echo "[INFO] Building tiles... "
valhalla_build_transit \
  conf/valhalla.json \
  ${TRANSITLAND_URL} \
  ${TRANSITLAND_PER_PAGE} \
  ${TRANSIT_TILE_DIR} \
  ${TRANSITLAND_API_KEY} \
  ${TRANSITLAND_LEVELS} \
  ${TRANSITLAND_FEED} 
# don't catch_exception here: this will throw a custom error

echo "[SUCCESS] valhalla_build_transit completed!"

echo "[INFO] Valdating transit tiles... "
valhalla_validate_transit \
  --config conf/valhalla.json \
  validate \
  ${TRANSIT_TEST_FILE}
catch_exception

echo "[SUCCESS] valhalla_validate_transit completed!"

# time_stamp
stamp=$(date +%Y_%m_%d-%H_%M_%S)

# upload to s3
if  [ -n "$TRANSIT_S3_PATH" ]; then
  echo "[INFO] Copying tiles to S3... "
  tar pcf - -C ${TRANSIT_TILE_DIR} . --exclude ./2 | pigz -9 > ${DATA_DIR}/transit_${stamp}.tgz
  catch_exception

  #push up to s3 the new file
  aws --region ${REGION} s3 cp ${DATA_DIR}/transit_${stamp}.tgz s3://${TRANSIT_S3_PATH}/ --acl public-read
  catch_exception
  rm -f ${DATA_DIR}/transit_${stamp}.tgz
  echo "[SUCCESS] Tiles successfully copied to S3!"
fi

# cya
echo "[SUCCESS] Run complete, exiting."
exit 0
