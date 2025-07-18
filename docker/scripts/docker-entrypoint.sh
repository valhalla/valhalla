#!/usr/bin/env bash

set -o errexit -o pipefail -o nounset

# source the helpers for globals and functions
. /valhalla/scripts/helpers.sh

# we need to either run commands that create files with or without sudo (depends if the image was built with a UID/GID other than 0)
run_cmd() {
  if [[ $(id --user) == "59999" ]] && [[ $(id --group) == "59999" ]]; then
    # -E preserves the env vars, but some are still nulled for security reasons
    # use "env" to preserve them
    $cmd_prefix sudo -E env LD_LIBRARY_PATH=$LD_LIBRARY_PATH $1 || exit 1
  else
    $cmd_prefix $1 || exit 1
  fi
}

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

# find out the owner of the mapped volume and warn if it's root
dir_owner=$(stat --format '%U' "${CUSTOM_FILES}")
echo ""
echo "INFO: Running container with user $(whoami) UID $(id --user) and GID $(id --group)."
if [[ ${dir_owner} == "root" ]]; then
  if [[ $(id --user) != "59999" ]] || [[ $(id --group) != "59999" ]]; then
    echo "ERROR: If you run with custom UID or GID you have to create the mapped directory to the container's /custom_files manually before starting the image"
    exit 1
  fi
fi
echo ""

# the env vars with True default are set in the dockerfile, others are evaluated in configure_valhalla.sh
if [[ -z $server_threads ]]; then
  export server_threads=$(nproc)
fi

if [[ -z $build_tar ]]; then
  build_tar="True"
fi
if [[ -z $serve_tiles ]]; then
  serve_tiles="True"
fi
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
