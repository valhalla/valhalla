#!/bin/bash

# Verify argument exists
if [ $# -ne 1 ]
then
  echo "Must supply run directory"
  exit 1
fi

# Verify directory exists
DIR=${1}
if [ ! -d "${DIR}" ]
then
  echo "${DIR} does not exist"
  exit 1
fi

# Switch to run directory
cd ${DIR}

# Verify statistics file exists
STATS_BASE_NAME="statistics"
STATS_EXT="csv"
STATS_FILENAME="${STATS_BASE_NAME}.${STATS_EXT}"
if [ ! -f "${STATS_FILENAME}" ]
then
  echo "${STATS_FILENAME} does not exist"
  exit 1
fi

### Example input
###1:orgLat, 2:orgLng, 3:destLat, 4:destLng, 5:result, 6:#Passes, 7:runtime, 8:trip time, 9:length, 10:arcDistance, 11:#Manuevers
###34.854443,40.608334,36.366665,36.983334,success,1,81,20031,273.763855,229.100693,44

# Write total stats header
TOTAL_STATS_FILENAME="total_${STATS_FILENAME}"
echo "ROUTE_COUNT,SUCCESS_COUNT,FAIL_COUNT,NUM_PASSES,RUN_TIME,TRIP_TIME,TRIP_LENGTH,NUM_MANEUVERS" > ${TOTAL_STATS_FILENAME}

# Assign ROUTE_COUNT (decement because of header)
ROUTE_COUNT=$(cat ${STATS_FILENAME} | wc -l)
((ROUTE_COUNT--))

# Assign SUCCESS_COUNT
SUCCESS_COUNT=$(cat ${STATS_FILENAME} | grep success | wc -l)

# Assign FAIL_COUNT
FAIL_COUNT=$(cat ${STATS_FILENAME} | grep fail | wc -l)

NUM_PASSES=0
RUN_TIME=0
TRIP_TIME=0
TRIP_LENGTH=0
NUM_MANEUVERS=0
{
  read; # Read header
  while IFS=, read IN_ORIG_LAT IN_ORIG_LNG IN_DEST_LAT IN_DEST_LNG IN_RESULT IN_NUM_PASSES IN_RUN_TIME IN_TRIP_TIME IN_TRIP_LENGTH IN_ARC_DISTAANCE IN_NUM_MANEUVERS IN_COST_SECONDS IN_COST
  do
    #echo "$IN_ORIG_LAT|$IN_ORIG_LNG|$IN_DEST_LAT|$IN_DEST_LNG|$IN_RESULT|$IN_NUM_PASSES|$IN_RUN_TIME|$IN_TRIP_TIME|$IN_TRIP_LENGTH|$IN_ARC_DISTAANCE|$IN_NUM_MANEUVERS"
    ((NUM_PASSES+=IN_NUM_PASSES))
    ((RUN_TIME+=IN_RUN_TIME))
    ((TRIP_TIME+=IN_TRIP_TIME))
    TRIP_LENGTH=$(python3 -c "print(${TRIP_LENGTH} + ${IN_TRIP_LENGTH})")
    ((NUM_MANEUVERS+=IN_NUM_MANEUVERS))
    #echo "NUM_PASSES=${NUM_PASSES}"
    #echo "RUN_TIME=${RUN_TIME}"
    #echo "TRIP_TIME=${TRIP_TIME}"
    #echo "TRIP_LENGTH=${TRIP_LENGTH}"
    #echo "NUM_MANEUVERS=${NUM_MANEUVERS}"
  done
} < ${STATS_FILENAME}

# Write total stats
echo "${ROUTE_COUNT},${SUCCESS_COUNT},${FAIL_COUNT},${NUM_PASSES},${RUN_TIME},${TRIP_TIME},${TRIP_LENGTH},${NUM_MANEUVERS}" >> ${TOTAL_STATS_FILENAME}

cd ..
exit

