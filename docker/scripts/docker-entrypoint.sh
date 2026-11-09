#!/usr/bin/env bash

set -o errexit -o pipefail -o nounset

# source the helpers for globals and functions
. /valhalla/scripts/helpers.sh

# logging valhalla version
VALHALLA_VERSION=$(valhalla_service --version | tr -d '\n')
echo "Valhalla version: ${VALHALLA_VERSION}"
echo ""

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
    tile_tar_rebuilt="True"
  fi
}

# the extract is indexed by edge position, so it's stale whenever the tile tar was rebuilt
do_build_custom_attributes() {
  if [[ "${build_custom_attributes}" != "True" && "${build_custom_attributes}" != "Force" ]]; then
    return
  fi
  if [[ "${build_custom_attributes}" == "True" && -f "${CUSTOM_ATTRIBUTES_TAR}" && "${tile_tar_rebuilt}" == "False" ]]; then
    echo "INFO: Found ${CUSTOM_ATTRIBUTES_TAR}, skipping custom attributes extract."
    return
  fi
  if ! [[ -f "${TILE_TAR}" ]]; then
    echo "WARNING: Custom attributes extract requested, but no tile tar found at ${TILE_TAR}, skipping."
    return
  fi
  if [[ -z "${custom_attributes_defs}" ]]; then
    echo "ERROR: build_custom_attributes is ${build_custom_attributes}, but custom_attributes_defs is empty."
    exit 1
  fi
  echo ""
  echo "======================================"
  echo "= Building custom attributes extract ="
  echo "======================================"
  echo ""
  valhalla_build_custom_attributes_extract -c "${CONFIG_FILE}" \
    --attribute-defs "${custom_attributes_defs}" \
    --default-values "${custom_attributes_default_values}" || exit 1
}

tile_tar_rebuilt="False"

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
  do_build_custom_attributes

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
  do_build_custom_attributes
else
  echo "ERROR: Unrecognized CMD: '$1'"
  exit 1
fi
