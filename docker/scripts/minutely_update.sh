#!/bin/bash 
#This script is based on the script located on github:
#https://github.com/artemp/MapQuest-Render-Stack/blob/master/scripts/minutely_update.sh

export DATA_DIR=${DATA_DIR:-"/data/valhalla"}
export PLANET_FILE=${PLANET_FILE:-"planet-latest.osm.pbf"}
export BASE_DIR=${BASE_DIR:-"${DATA_DIR}/extracts"}
export WORKDIR_OSM=${WORKDIR_OSM:-"${BASE_DIR}/osmosis_work_dir.${PLANET_FILE}"}
export CHANGESET_DIR=${CHANGESET_DIR:-"${WORKDIR_OSM}/minutely"}
export URL=${URL:-"http://planet.us-east-1.mapzen.com/planet-latest.osm"}
PLANET=$BASE_DIR/$PLANET_FILE

OSMOSIS="/usr/bin/osmosis"
export JAVACMD_OPTIONS="-Djava.io.tmpdir=${DATA_DIR}/temp"

# check they're all present
if [ ! -e $OSMOSIS ]; then
  echo "[ERROR] osmosis ($OSMOSIS) not installed, but is required."
  exit 1
fi

if [ ! -e /usr/bin/osmconvert ]; then
  echo "[ERROR] osmconvert (/usr/bin/osmconvert) not installed, but is required."
  exit 1
fi

osmosis_fetch_changeset() {
  if [ ! -e $WORKDIR_OSM/state.txt ]; then
    echo "[ERROR] Osmosis state file not found - has the state been correctly initialized?"
    exit 1
  fi
  STATE_TIMESTAMP=$(grep '^timestamp=' $WORKDIR_OSM/state.txt | tail -n1 | cut -c 11-)
  CURRENT_TIMESTAMP=$(date -u "+%Y-%m-%d_%H:%M:%S")
  CHANGESET_FILE=$CHANGESET_DIR/changeset-$STATE_TIMESTAMP.ocs.gz

  echo "$CURRENT_TIMESTAMP:Downloading changeset $STATE_TIMESTAMP"
  cp $WORKDIR_OSM/state.txt $CHANGESET_DIR/state-$STATE_TIMESTAMP
  $OSMOSIS --read-replication-interval workingDirectory=$WORKDIR_OSM \
    --simplify-change --write-xml-change $CHANGESET_FILE
}

osmosis_cleanup() {
  rm -f $CHANGESET_DIR/changeset-$STATE_TIMESTAMP.ocs.gz
  rm -f $CHANGESET_DIR/state-$STATE_TIMESTAMP
}

update() {

  osmosis_fetch_changeset

  $OSMOSIS --rxc $CHANGESET_FILE --rb $PLANET --ac --wb $PLANET.new

  # exit if osmosis fails
  if [ $? -ne 0 ]; then
    echo "[ERROR] failed to apply $CHANGESET_DIR/changeset-$STATE_TIMESTAMP.ocs.gz"
    cp $CHANGESET_DIR/state-$STATE_TIMESTAMP $WORKDIR_OSM/state.txt
    exit 1
  else
    echo "[SUCCESS] applied $CHANGESET_DIR/changeset-$STATE_TIMESTAMP.ocs.gz"
    echo "[SUCCESS] Done"
  fi

  mv $PLANET.new $PLANET

  echo "[INFO] running osmconvert $PLANET --out-statistics."
  osmconvert $PLANET --out-statistics > $WORKDIR_OSM/current_stats.txt

  osmosis_cleanup
}

initialize() {

  # make the directories
  rm -rf ${DATA_DIR}/temp
  mkdir -p ${WORKDIR_OSM} ${CHANGESET_DIR} ${BASE_DIR} ${DATA_DIR}/temp

  if [ ! -e $WORKDIR_OSM/state.txt ]; then

    echo "[INFO] downloading osm data..."
    wget --quiet -O ${BASE_DIR}/${PLANET_FILE} ${URL}.pbf

    if [ $? -ne 0 ]; then
      echo "[ERROR] Planet file failed to download - cannot initialize without this."
      exit 1
    fi

    echo "[INFO] downloading osm md5sum..."
    wget --quiet -O ${BASE_DIR}/${PLANET_FILE}.md5 ${URL}.pbf.md5
    if [ $? -ne 0 ]; then
      echo "[ERROR] Planet file md5sum failed to download - cannot initialize without this."
      exit 1
    fi

    echo "[INFO] comparing md5sum values"
    MD5=`cat ${PLANET_FILE}.md5 | awk '{print $1}'`
    MD5_OUTPUT=`md5sum ${PLANET_FILE} | awk '{print $1}'`

    if [ "${MD5}" != "${MD5_OUTPUT}" ]; then
      echo "[ERROR] md5sums do not match."
      exit 1
    fi 

    $OSMOSIS --read-replication-interval-init workingDirectory=$WORKDIR_OSM

    baseUrl=https://planet.openstreetmap.org/replication/minute
    replacement="s@^\(baseUrl\s*=\s*\).*\$@\1${baseUrl}@"
    sed -i $replacement $WORKDIR_OSM/configuration.txt
    sed -i "s/^\(maxInterval\s*=\s*\).*\$/\10/" $WORKDIR_OSM/configuration.txt

    echo "[INFO] obtaining planet timestamp."
    planet_timestamp=$(osmconvert $PLANET --out-timestamp)

    wget "http://osm.personalwerk.de/replicate-sequences/?$planet_timestamp" -O $WORKDIR_OSM/state.txt;
    cat $WORKDIR_OSM/state.txt

  fi
}

initialize

update
