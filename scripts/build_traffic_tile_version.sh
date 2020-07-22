#!/usr/bin/env bash
# Builds the traffic_tile_version.h header

set -o errexit -o pipefail -o nounset

readonly VALHALLA_SOURCE_DIR="${1:-NONE}"
readonly GENERATED_FILE="${2:-NONE}"

if [ "$VALHALLA_SOURCE_DIR" = NONE ]; then
  echo "Must specify path to root folder"
  exit 1
fi
if [ "$GENERATED_FILE" = NONE ]; then
  echo "Must specify path to target file"
  exit 1
fi

printf "const uint32_t TRAFFIC_TILE_VERSION = 0x%s;" $(
  # Grab only four bytes because space constraints
  sha1sum ${VALHALLA_SOURCE_DIR}/valhalla/baldr/traffictile.h | head -c4
) > "$GENERATED_FILE"
