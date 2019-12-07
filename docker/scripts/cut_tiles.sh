#!/bin/bash
set -e

# some required vars to get and put data.
#S3_SEGMENT_PATH = OSMLR data location, e.g s3://osmlr-tiles/prod/
#S3_PATH = Final results of the build tiles and associate segments, e.g. s3://a_bucket
#S3_TRANSIT_PATH = Where to get transit data, e.g. s3://transit-data/dev/

export DATA_DIR=${DATA_DIR:-"/data/valhalla"}
export TOUCHFILE=${TOUCHFILE:-"${DATA_DIR}/CUT_COMPLETE"}
export TILES_DIR=${TILES_DIR:-"${DATA_DIR}/tiles"}
export TESTS_DIR=${TESTS_DIR:-"${DATA_DIR}/tests"}
export EXTRACTS_DIR=${EXTRACTS_DIR:-"${DATA_DIR}/extracts"}
export INCLUDE_ELEVATION=${INCLUDE_ELEVATION:-"false"}
export ELEVATION_DIR=${ELEVATION_DIR:-"${DATA_DIR}/elevation"}
export TRANSIT_DIR=${TRANSIT_DIR:-"${DATA_DIR}/transit"}
export CONF_FILE=${CONF_FILE:-"/conf/valhalla.json"}
export REGION=${REGION:-"us-east-1"}
export OSMLR_DIR=${OSMLR_DIR:-"${DATA_DIR}/osmlr"}
export NUMBER_OF_THREADS=${NUMBER_OF_THREADS:-"4"}
export INCLUDE_EXTRAS=${INCLUDE_EXTRAS:-"true"}
export BUILD_GEOJSON_OSMLR=${BUILD_GEOJSON_OSMLR:-"false"}

catch_exception() {
  if [ $? != 0 ]; then
    echo "[FAILURE] Detected non zero exit status while processing valhalla tiles!"
    exit 1
  fi
}

mv_stamp() {
  local b=$(basename ${1})
  mv ${1} ${b%.*}_${2}.${b##*.}
  catch_exception
}

cp_stamp() {
  local b=$(basename ${1})
  cp -rp ${1} ${b%.*}_${2}.${b##*.}
  catch_exception
}

get_latest_transit() {
  file=$(aws --region ${REGION} s3 ls ${1}transit_ | sort | tail -1)
  file_name=$(echo ${file} | awk '{print $4}')
  latest_upload=${1}${file_name}

  #use the latest...if not already
  if [ ! -f ${DATA_DIR}/${file_name} ]; then
    # rm old tarball
    rm -f ${DATA_DIR}/transit_*.tgz
    aws --region ${REGION} s3 cp $latest_upload ${DATA_DIR}/${file_name}
    catch_exception

    # remove old data
    rm -rf ${TRANSIT_DIR}
    catch_exception
    mkdir ${TRANSIT_DIR}
    catch_exception
    tar pxf ${DATA_DIR}/${file_name} -C ${TRANSIT_DIR}
    catch_exception
  fi
}

get_latest_osmlr() {
  file=$(aws --region ${REGION} s3 ls ${1}osmlr_ | sort | tail -1)
  file_name=$(echo ${file} | awk '{print $4}')
  latest_upload=${1}${file_name}

  #use the latest...if not already
  if [ ! -f ${DATA_DIR}/${file_name} ]; then
    # rm old tarball
    rm -f ${DATA_DIR}/osmlr_*.tgz
    catch_exception
    aws --region ${REGION} s3 cp $latest_upload ${DATA_DIR}/${file_name}
    catch_exception
    
    # remove old data
    rm -rf ${OSMLR_DIR}
    catch_exception
    mkdir ${OSMLR_DIR}
    catch_exception
    tar pxf ${DATA_DIR}/${file_name} -C ${OSMLR_DIR}
    catch_exception
    rm ${DATA_DIR}/${file_name}
    catch_exception
  fi
}

export PATH=$PATH:/usr/sbin:/usr/local/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
MAX_CACHE_SIZE=`echo "$((1024 * 1024 * 1024))"`

rm -rf ${TILES_DIR}
catch_exception
mkdir -p ${TILES_DIR}
catch_exception

echo "[INFO] updating config."
#config needs to be updated for cutting tiles.
valhalla_build_config \
      --mjolnir-tile-dir ${TILES_DIR} \
      --mjolnir-max-cache-size ${MAX_CACHE_SIZE} \
      >${CONF_FILE}
catch_exception

# name the dir where this will go
stamp=$(date +%Y_%m_%d-%H_%M_%S)
cd ${DATA_DIR}
catch_exception
# things we need to make if we dont have them
extracts=$(find ${EXTRACTS_DIR} -type f -name "*.pbf")
catch_exception
admin_file=$(jq -r '.mjolnir.admin' ${CONF_FILE})
catch_exception
timezone_file=$(jq -r '.mjolnir.timezone' ${CONF_FILE})
catch_exception
if [ ! -e $admin_file ]; then
  echo "[INFO] building admins."
  valhalla_build_admins -c ${CONF_FILE} $(find ${EXTRACTS_DIR} -type f -name "*.pbf")
  catch_exception
fi
if [ ! -e $timezone_file ]; then
  echo "[INFO] building timezones."
  valhalla_build_timezones ${CONF_FILE}
  catch_exception
fi

#transit data
if  [ -n "$S3_TRANSIT_PATH" ]; then
  echo "[INFO] getting transit data."
  get_latest_transit ${S3_TRANSIT_PATH}
  catch_exception
fi

#elevation
if [ "${INCLUDE_ELEVATION}" == "true" ]; then
  if [ ! -d ${ELEVATION_DIR} ]; then
    mkdir -p ${ELEVATION_DIR}

    echo "[INFO] building elevation."
    valhalla_build_elevation -180 180 -90 90 ${ELEVATION_DIR} $(nproc)
    catch_exception
  else
    echo "[WARN] elevation directory ${ELEVATION_DIR} already exists, not (re)building elevation tiles!"
  fi
fi

# cut tiles from the data
echo "[INFO] building tiles."
valhalla_build_tiles -c ${CONF_FILE} $(find ${EXTRACTS_DIR} -type f -name "*.pbf")
catch_exception
rm -rf *.bin
catch_exception

#only run if url exists
if  [ -n "$TEST_FILE_URL" ]; then
  echo "[INFO] running tests."
  rm -rf ${TESTS_DIR}
  catch_exception
  mkdir -p ${TESTS_DIR}
  catch_exception
  # see if these tiles are any good
  cp /scripts/test_tiles.sh ${TESTS_DIR}/test_tiles.sh
  catch_exception
  cp /scripts/batch.sh ${TESTS_DIR}/batch.sh
  catch_exception
  cp /scripts/run.sh ${TESTS_DIR}/run.sh
  catch_exception
  wget -q "${TEST_FILE_URL}" -O ${TESTS_DIR}/tests.txt; catch_exception
  ${TESTS_DIR}/test_tiles.sh ${CONF_FILE} ${TESTS_DIR} tests.txt
  catch_exception
  echo "[SUCCESS] Tests passed."
fi

#only run if osmlr segment path exists
if  [ -n "$S3_SEGMENT_PATH" ]; then
  #osmlr data
  get_latest_osmlr ${S3_SEGMENT_PATH}

  echo "[INFO] Associating segments... "
  valhalla_associate_segments \
    -t ${OSMLR_DIR} \
    -j ${NUMBER_OF_THREADS} \
    --config ${CONF_FILE}
  catch_exception

  echo "[SUCCESS] valhalla_associate_segments completed!"
fi

CUR_PLANET_DIR=${DATA_DIR}/planet_${stamp}
mkdir -p ${CUR_PLANET_DIR}
catch_exception
pushd ${CUR_PLANET_DIR}
catch_exception

if [[ "$INCLUDE_EXTRAS" == "true" ]]; then
  echo "[INFO] building connectivity."
  valhalla_build_connectivity -c ${CONF_FILE}
  catch_exception
  echo "[INFO] building stats."
  valhalla_build_statistics -c ${CONF_FILE}
  catch_exception
  echo "[INFO] exporting edges."
  valhalla_export_edges --config ${CONF_FILE} > edges_${stamp}.0sv
  catch_exception

  for f in connectivity*; do  mv_stamp $f ${stamp}; done
  mv_stamp statistics.sqlite ${stamp}
  mv_stamp maproulette_tasks.geojson ${stamp}
  cp_stamp ${DATA_DIR}/$(basename ${admin_file}) ${stamp}
  cp_stamp ${DATA_DIR}/$(basename ${timezone_file}) ${stamp}
else
  rm -f maproulette_tasks.geojson
fi

if [ "$BUILD_GEOJSON_OSMLR" == "true" ] && [ -n "$S3_SEGMENT_PATH" ]; then
  echo "[INFO] building geojson osmlr."

  mkdir -p ${TILES_DIR}/osmlr
  catch_exception

  geojson_osmlr \
    -i ${OSMLR_DIR} \
    -o ${TILES_DIR}/osmlr \
    -t 1 \
    --config ${CONF_FILE}
  catch_exception

  mv ${TILES_DIR}/osmlr ${CUR_PLANET_DIR}/osmlr
  catch_exception
  echo "[SUCCESS] geojson osmlr completed!"
fi

pushd ${TILES_DIR}
catch_exception
find . | sort -n | tar -cf ${CUR_PLANET_DIR}/planet_${stamp}.tar --no-recursion -T -
catch_exception
mv ${TILES_DIR}/0 ${CUR_PLANET_DIR}/0
catch_exception
mv ${TILES_DIR}/1 ${CUR_PLANET_DIR}/1
catch_exception
mv ${TILES_DIR}/2 ${CUR_PLANET_DIR}/2
catch_exception

#rm admin and tz after they have been included in the tar.
if [[ "$INCLUDE_EXTRAS" == "false" ]]; then
  rm -f ${admin_file}
  rm -f ${timezone_file}
fi

popd
catch_exception
popd
catch_exception

# do we want to send this update to s3 (do so in the background)
if  [ -n "$S3_PATH" ]; then
  {
    echo "[INFO] uploading data."
    #push up s3 new files
    aws --region ${REGION} s3 cp --recursive ${CUR_PLANET_DIR} ${S3_PATH}/planet_${stamp}
    catch_exception

    echo "[INFO] purging local data."
    #clean it up the new stuff
    rm -rf ${CUR_PLANET_DIR}
    catch_exception
  }
fi
echo "[SUCCESS] Run complete. Valhalla tile creation finished, exiting after touching CUT_COMPLETE file."
touch ${TOUCHFILE}
