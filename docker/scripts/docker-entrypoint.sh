#!/usr/bin/env bash

set -o errexit -o pipefail -o nounset

# source the helpers for globals and functions
. /valhalla/scripts/helpers.sh

do_build_tar() {
  local build_tar_local=$1
  if ([[ "${build_tar_local}" == "True" && ! -f $TILE_TAR ]]) || [[ "${build_tar_local}" == "Force" ]]; then
    options="-c ${CONFIG_FILE} -v"
    if ! [[ -z ${traffic_name} ]]; then
      options="${options} -t"
    fi
    if [[ "${build_tar_local}" == "Force" ]]; then 
      options="${options} --overwrite"
    fi 
    valhalla_build_extract ${options} || exit 1
  fi
}

export server_threads=${server_threads:-$(nproc)}
if [[ "$force_rebuild" == "True" ]]; then
  build_tar="Force"
fi

# evaluate CMD
if [[ $1 == "build_tiles" ]]; then

  /valhalla/scripts/configure_valhalla.sh ${CONFIG_FILE} ${CUSTOM_FILES} ${TILE_DIR} ${TILE_TAR} || exit 1
  # tar tiles unless not wanted
  if [[ "$build_tar" == "True" ]] || [[ "$build_tar" == "Force" ]]; then
    do_build_tar "$build_tar"
  else
    echo "WARNING: Skipping tar building. Expect degraded performance while using Valhalla."
  fi

  # set 775/664 permissions on all created files
  find "${CUSTOM_FILES}" -type d -exec chmod 775 {} \;
  find "${CUSTOM_FILES}" -type f -exec chmod 664 {} \;

  if [[ ${serve_tiles} == "True" ]]; then
    if test -f ${CONFIG_FILE}; then
      echo "INFO: Found config file. Starting valhalla service!"
      exec valhalla_service ${CONFIG_FILE} ${server_threads}
    else
      echo "WARNING: No config found!"
    fi

    # Keep docker running easy
    exec "$@"
  else
    echo "INFO: Not serving tiles. Exiting."
  fi
elif [[ $1 == "tar_tiles" ]]; then
  do_build_tar "$build_tar"
else
  echo "ERROR: Unrecognized CMD: '$1'"
  exit 1
fi
