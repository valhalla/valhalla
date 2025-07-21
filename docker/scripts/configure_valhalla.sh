#!/usr/bin/env bash

set -o errexit -o pipefail -o nounset

# source the helpers for globals and functions
. /valhalla/scripts/helpers.sh

# Strategy:
#   - rebuild if:
#     - admins or tz db don't exist
#     - build_elevation is true and elevation doesn't exist (force)
#     - tile extract or folder doesn't exist (force)
#     - use_tiles_ignore_pbf is false and new file constellations exist
#   - downloads pbfs if there are none in the folder

if ! test -f "${HASH_FILE}"; then
  touch ${HASH_FILE}
fi

# determine if extras were requested
do_elevation="False"
do_transit="False"
do_admins="False"
do_timezones="False"
# anything other than True or Force will result in not building the dbs
if ([[ "${build_admins}" == "True" ]] && ! test -f "${ADMIN_DB}") || [[ "${build_admins}" == "Force" ]]; then
  do_admins="True"
fi
if ([[ "${build_time_zones}" == "True" ]] && ! test -f "${TIMEZONE_DB}") || [[ "${build_time_zones}" == "Force" ]]; then
  do_timezones="True"
fi
if [[ "${build_elevation}" == "True" ]] || [[ "${build_elevation}" == "Force" ]]; then
  do_elevation="True"
fi
# if there's no transit tiles yet, but it should build transit, then do that; or force and remove
if [[ "${build_transit}" == "Force" ]] || (! [[ -d ${TRANSIT_DIR} ]] && [[ "${build_transit}" == "True" ]]) || ([[ $(find ${TRANSIT_DIR} -maxdepth 1 -type d | wc -l) -eq 1 ]] && [[ "${build_transit}" == "True" ]]); then
  rm -r ${TRANSIT_DIR} 2> /dev/null
  do_transit="True"
  if ! [[ -d ${GTFS_DIR} ]]; then
    echo "WARNING: Transit build requested, but no GTFS datasets found at ${GTFS_DIR}, skipping transit.."
    do_transit="False"
  fi
fi

# Find and add .pbf files to the list of files
new_hashes="False"
files=""
files_counter=0
for file in $(ls $CUSTOM_FILES/*.pbf 2>/dev/null); do
  hash_echo=$(hash_exists ${file})
  if ! [[ $hash_echo =~ "True" ]] ; then
    echo "WARNING: Hash not found for: ${file}!"
    new_hashes="True"
  fi
  files="${files} ${file}"
  files_counter=$((files_counter + 1))
done

# determine whether we need a full tile build or not
do_build="False"
if ! test -f "${TILE_TAR}" && ! [ -n "$(ls -A ${TILE_DIR} 2>/dev/null)" ]; then
  # build if no tiles exist
  echo "WARNING: No routing tiles found at ${TILE_TAR} or ${TILE_DIR}, starting a new build"
  do_build="True"
elif [[ ${force_rebuild} == "True" ]]; then
  # respect the env var
  echo "WARNING: force_rebuild ${force_rebuild}, starting a new build"
  do_build="True"
elif [[ "${do_transit}" == "True" ]]; then
  echo "WARNING: Requested transit data build, starting a new tile build"
  do_build="True"
elif [[ "${do_admins}" == "True" ]]; then
  # rebuild if the admin db has to be built
  echo "WARNING: Requested admin db, but none found, starting a new tile build"
  do_build="True"
elif [[ "${do_timezones}" == "True" ]]; then
  # rebuild if the timezone db has to be built
  echo "WARNING: Requested timezone db, but none found, starting a new build"
  do_build="True"
elif [[ "${do_elevation}" == "True" ]] && ! [ -n "$(ls -A ${ELEVATION_PATH} 2>/dev/null)" ]; then
  # build if elevation was requested but not downloaded yet
  echo "WARNING: Requested elevation support, but none found, starting a new build"
  do_build="True"
elif [[ "${use_tiles_ignore_pbf}" == "True" ]]; then
  # don't build if there are tiles and we want to load them
  echo "INFO: Found routing tiles with 'use_tiles_ignore_pbf' True."
  echo "INFO: Jumping directly to the tile loading!"
  do_build="False"
elif [[ "${new_hashes}" == "True" ]]; then
  # build if there are new/other PBF files
  echo "WARNING: New PBF files were detected, starting new build"
  echo "WARNING: Hashes $(cat ${HASH_FILE})"
  do_build="True"
else
  # files exist and no reason to rebuild
  echo "INFO: Routing tiles exist and no need to rebuild."
fi

# if no PBFs were found try downloading some
if [[ ${files_counter} == 0 ]] && [[ ${do_build} == "True" ]]; then
  if [[ -z "${tile_urls}" ]]; then
    echo "ERROR: No local PBF files, valhalla_tiles.tar and no tile URLs found. Nothing to do."
    exit 1
  else
    echo "WARNING: No local files and no valhalla_tiles.tar found. Downloading links: ${tile_urls}!"
    for url in ${tile_urls}; do
      fp=$CUSTOM_FILES/$(basename $url)
      download_file "${url}" "${fp}"
      files="${files} ${fp}"
    done
  fi
fi
files=$(echo $files | xargs)

# be careful how to write the config (mostly for restart scenarios where env vars are true all of a sudden)
if test -f "${CONFIG_FILE}"; then

  if [[ "${update_existing_config}" == "True" ]]; then 
    echo "INFO: Found existing valhalla.json. Updating possibly missing entries."

    # create temporary default config
    valhalla_build_config > ${TMP_CONFIG_FILE}

    # collect additions to make from the temp config (treating arrays as atomic values)
    additions=$(jq --slurpfile A "${TMP_CONFIG_FILE}" --slurpfile B "${CONFIG_FILE}" -n '
      def push(p;k): p+[k];
      def missing(a;b;p):
        if a|type=="object" then
          reduce (a|keys_unsorted[]) as $k ([];
            .+(
              if b|has($k)|not then [{path:push(p;$k), dot:(push(p;$k)|join(".")), value:a[$k]}]
              elif (a[$k]|type=="object") and (b[$k]|type=="object") then missing(a[$k];b[$k];push(p;$k))
              else [] end))
        else [] end;
      missing($A[0];$B[0];[])
    ')

    echo "$additions" | jq -r '.[] | "added \(.dot) with value \(.value)"'

    # add all additions
    jq --argjson additions "$additions" '
      reduce $additions[] as $a (.;
        setpath($a.path; $a.value)
      )
    ' "${CONFIG_FILE}" | sponge "${CONFIG_FILE}"

    rm ${TMP_CONFIG_FILE}
  fi

  jq --arg d "${TILE_DIR}" '.mjolnir.tile_dir = $d' "${CONFIG_FILE}"| sponge "${CONFIG_FILE}"
  jq --arg d "${TILE_TAR}" '.mjolnir.tile_extract = $d' "${CONFIG_FILE}"| sponge "${CONFIG_FILE}"
  jq --arg d "${ADMIN_DB}" '.mjolnir.admin = $d' "${CONFIG_FILE}"| sponge "${CONFIG_FILE}"
  jq --arg d "${TIMEZONE_DB}" '.mjolnir.timezone = $d' "${CONFIG_FILE}"| sponge "${CONFIG_FILE}"
  jq --arg d "${ELEVATION_PATH}" '.additional_data.elevation = $d' "${CONFIG_FILE}"| sponge "${CONFIG_FILE}"
  jq --arg d "${GTFS_DIR}" '.mjolnir.transit_feeds_dir = $d' "${CONFIG_FILE}"| sponge "${CONFIG_FILE}"
  jq --arg d "${TRANSIT_DIR}" '.mjolnir.transit_dir = $d' "${CONFIG_FILE}"| sponge "${CONFIG_FILE}"
  jq --arg d "${TRAFFIC_TAR}" '.mjolnir.traffic_extract = $d' "${CONFIG_FILE}"| sponge "${CONFIG_FILE}"
  jq --arg d "${server_threads}" '.mjolnir.concurrency = $d' "${CONFIG_FILE}"| sponge "${CONFIG_FILE}"
else
  additional_data_elevation="--additional-data-elevation $ELEVATION_PATH"
  mjolnir_admin="--mjolnir-admin ${ADMIN_DB}"
  mjolnir_timezone="--mjolnir-timezone ${TIMEZONE_DB}"
  transit_dir="--mjolnir-transit-dir ${TRANSIT_DIR}"
  gtfs_dir="--mjolnir-transit-feeds-dir ${GTFS_DIR}"
  traffic="--mjolnir-traffic-extract ${TRAFFIC_TAR}"
  threads="--mjolnir-concurrency ${server_threads}"
  valhalla_build_config \
    --mjolnir-tile-dir ${TILE_DIR} \
    --mjolnir-tile-extract ${TILE_TAR} \
    ${gtfs_dir} ${transit_dir} ${mjolnir_timezone} ${threads} \
    ${mjolnir_admin} ${additional_data_elevation} ${traffic} > ${CONFIG_FILE}
fi

# build the databases maybe
if [[ "${do_admins}" == "True" ]]; then
  maybe_create_dir $(dirname ${ADMIN_DB})
  echo ""
  echo "====================="
  echo "= Building admin db ="
  echo "====================="
  echo ""
  valhalla_build_admins --config "${CONFIG_FILE}" ${files}
fi
if [[ "${do_timezones}" == "True" ]]; then
  maybe_create_dir $(dirname ${TIMEZONE_DB})
  echo ""
  echo "========================"
  echo "= Building timezone db ="
  echo "========================"
  echo ""
  valhalla_build_timezones > ${TIMEZONE_DB}
fi
if [[ "${do_transit}" == "True" ]]; then
  echo ""
  echo "=========================="
  echo "= Building transit tiles ="
  echo "=========================="
  echo ""
  valhalla_ingest_transit -c ${CONFIG_FILE}
  valhalla_convert_transit -c ${CONFIG_FILE}
  # also do timezones if not done already
  if ! [[ -f ${TIMEZONE_DB} ]]; then
    valhalla_build_timezones > ${TIMEZONE_DB}
  fi
fi

# Finally build the tiles
if [[ "${do_build}" == "True" ]]; then
  echo "INFO: Running build tiles with: ${CONFIG_FILE} ${files}"

  # if we should build with elevation we need to build the tiles in stages
  echo ""
  echo "============================"
  echo "= Build the initial graph. ="
  echo "============================"

  valhalla_build_tiles -c ${CONFIG_FILE} -e build ${files}

  # Build the elevation data if requested
  if [[ $do_elevation == "True" ]]; then
    if [[ ${build_elevation} == "Force" ]] && ! test -d "${ELEVATION_PATH}"; then
      echo "WARNING: Rebuilding elevation tiles"
      rm -rf $ELEVATION_PATH
    fi
    maybe_create_dir ${ELEVATION_PATH}
    echo ""
    echo "================================"
    echo "= Download the elevation tiles ="
    echo "================================"
    valhalla_build_elevation --from-tiles --decompress -c ${CONFIG_FILE} -v
  fi

fi

# Use OSMSpeeds default_speeds

updated_default_speed_config=False
if [[ $use_default_speeds_config == "True" ]]; then
  if ! test -f "${DEFAULT_SPEEDS_CONFIG}"; then
    echo ""
    echo "======================================"
    echo "= Download the default speeds config ="
    echo "======================================"
    download_file "${default_speeds_config_url}" "${DEFAULT_SPEEDS_CONFIG}"
    updated_default_speed_config=True
  fi
  jq --arg d "${DEFAULT_SPEEDS_CONFIG}" '.mjolnir.default_speeds_config = $d' "${CONFIG_FILE}" | sponge "${CONFIG_FILE}"
fi

if [[ "${do_build}" == "True" ]] || [[ $updated_default_speed_config == "True" ]]; then
  echo ""
  echo "==============================="
  echo "= Enhancing the initial graph ="
  echo "==============================="
  valhalla_build_tiles -c ${CONFIG_FILE} -s enhance ${files} ||

  echo "INFO: Successfully built files: ${files}"
  add_hashes "${files}"
fi
