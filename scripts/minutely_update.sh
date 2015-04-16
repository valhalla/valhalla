#!/bin/bash 
#This script is based on the script located on github:
#https://github.com/artemp/MapQuest-Render-Stack/blob/master/scripts/minutely_update.sh
BASE_DIR="/data/osm_data"

# programs we're going to use
OSMOSIS="/usr/bin/osmosis"

# check they're all present
if [ ! -e $OSMOSIS ]; then
  echo "osmosis ($OSMOSIS) not installed, but is required."
  exit 1
fi

# directories used by the update process
WORKDIR_OSM="$BASE_DIR/osmosis_work_dir"
CHANGESET_DIR="$WORKDIR_OSM/minutely"
# make the directories
mkdir -p $WORKDIR_OSM $CHANGESET_DIR 

osmosis_fetch_changeset() {
  if [ ! -e $WORKDIR_OSM/state.txt ]; then
    echo "Osmosis state file not found - has the state been correctly initialized?"
    exit 1
  fi
  STATE_TIMESTAMP=`grep '^timestamp=' $WORKDIR_OSM/state.txt | tail -n1 | cut -c 11-`
  CURRENT_TIMESTAMP=`date -u "+%Y-%m-%d_%H:%M:%S"`
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

  PLANET=$BASE_DIR/planet/$1  

  $OSMOSIS --rxc $CHANGESET_FILE --rb $PLANET --ac --wb $PLANET.new

  # exit if osmosis fails
  if [ $? -ne 0 ]
  then
    echo "failed to apply $CHANGESET_DIR/changeset-$STATE_TIMESTAMP.ocs.gz"
    cp $CHANGESET_DIR/state-$STATE_TIMESTAMP $WORKDIR_OSM/state.txt
    exit 1
  else
    echo "applied $CHANGESET_DIR/changeset-$STATE_TIMESTAMP.ocs.gz"
    echo "Done"
  fi

  mv $PLANET.new $PLANET

  echo "running osmconvert $PLANET --out-statistics."
  osmconvert $PLANET --out-statistics > $WORKDIR_OSM/current_stats.txt

  osmosis_cleanup
}

initialize() {
  if [ -e $WORKDIR_OSM/state.txt ]; then
    echo "Osmosis state file found - has this already been initialized?"
    exit 1
  fi
  PLANET=$BASE_DIR/planet/$1
  if [ ! -e $PLANET ]; then
    echo "Planet file not found - cannot initialize without this."
    exit 1
  fi
  
  $OSMOSIS --read-replication-interval-init workingDirectory=$WORKDIR_OSM

  baseUrl=http://planet.openstreetmap.org/replication/minute
  replacement="s@^\(baseUrl\s*=\s*\).*\$@\1${baseUrl}@"  
  sed -i $replacement $WORKDIR_OSM/configuration.txt
  sed -i "s/^\(maxInterval\s*=\s*\).*\$/\10/" $WORKDIR_OSM/configuration.txt
  
  echo "obtaining planet timestamp."
  planet_timestamp=`osmconvert $PLANET --out-timestamp`

  #https://github.com/MaZderMind/replicate-sequences
  wget "http://osm.personalwerk.de/replicate-sequences/?$planet_timestamp" -O $WORKDIR_OSM/state.txt;
}

# make sure only one is running at any time...
LOCK_FILE="$BASE_DIR/locks/minutely.lock"
mkdir -p "$BASE_DIR/locks"

(set -C; : > $LOCK_FILE) 2> /dev/null

if [ $? != "0" ]; then
   echo "Lock File exists - exiting"
   exit 1
fi

trap 'rm $LOCK_FILE' EXIT 1 2 3 6

case "$1" in
  update)
   update $2;;

  initialize)
    initialize $2;;

  *)
    echo "Usage: $0 {update|initialize pbf_file}"
    echo 
    echo "  update:     Update the database using Osmosis and minutely diffs."
    echo
    echo "  initialize: Set up osmosis replication - relies on the location of the"
    echo "              planet file from the import step."
    echo
    exit 1;;
esac


