#!/bin/bash

# Verify argument exists
if [ $# -ne 1 ]
then
  echo "Must supply a file with a list of directories"
  exit 1
fi

# Verify file exists
DIRS_FILE=${1}
if [ ! -f "${DIRS_FILE}" ]
then
  echo "${DIRS_FILE} does not exist"
  exit 1
fi

echo `date`

DIRS=$(cat ${DIRS_FILE})
for DIR in ${DIRS}
do
  echo "Processing ${DIR}..."
  ./total_run_stats.sh ${DIR}
done

# Write total stats header
TOTAL_STATS_FILENAME="total_statistics.csv"
TOTAL_MULTI_RUN_STATS_FILENAME="$(date +%Y%m%d_%H%M%S)_${TOTAL_STATS_FILENAME}"
echo "ROUTE_COUNT,SUCCESS_COUNT,FAIL_COUNT,NUM_PASSES,RUN_TIME,TRIP_TIME,TRIP_LENGTH,NUM_MANEUVERS" > ${TOTAL_MULTI_RUN_STATS_FILENAME}

# Initialize sum variables
ROUTE_COUNT=0
SUCCESS_COUNT=0
FAIL_COUNT=0
NUM_PASSES=0
RUN_TIME=0
TRIP_TIME=0
TRIP_LENGTH=0
NUM_MANEUVERS=0
for DIR in ${DIRS}
do
  {
    read; # Read header
    while IFS=, read IN_ROUTE_COUNT IN_SUCCESS_COUNT IN_FAIL_COUNT IN_NUM_PASSES IN_RUN_TIME IN_TRIP_TIME IN_TRIP_LENGTH IN_NUM_MANEUVERS
    do
      #echo "$IN_ROUTE_COUNT|$IN_SUCCESS_COUNT|$IN_FAIL_COUNT|$IN_NUM_PASSES|$IN_RUN_TIME|$IN_TRIP_TIME|$IN_TRIP_LENGTH|$IN_NUM_MANEUVERS"
      ((ROUTE_COUNT+=IN_ROUTE_COUNT))
      ((SUCCESS_COUNT+=IN_SUCCESS_COUNT))
      ((FAIL_COUNT+=IN_FAIL_COUNT))
      ((NUM_PASSES+=IN_NUM_PASSES))
      ((RUN_TIME+=IN_RUN_TIME))
      ((TRIP_TIME+=IN_TRIP_TIME))
      TRIP_LENGTH=$(python3 -c "print(${TRIP_LENGTH} + ${IN_TRIP_LENGTH})")
      ((NUM_MANEUVERS+=IN_NUM_MANEUVERS))
      #echo "ROUTE_COUNT=${ROUTE_COUNT}"
      #echo "SUCCESS_COUNT=${SUCCESS_COUNT}"
      #echo "FAIL_COUNT=${FAIL_COUNT}"
      #echo "NUM_PASSES=${NUM_PASSES}"
      #echo "RUN_TIME=${RUN_TIME}"
      #echo "TRIP_TIME=${TRIP_TIME}"
      #echo "TRIP_LENGTH=${TRIP_LENGTH}"
      #echo "NUM_MANEUVERS=${NUM_MANEUVERS}"
    done
  } < ${DIR}/${TOTAL_STATS_FILENAME}
done

# Write total stats
echo "${ROUTE_COUNT},${SUCCESS_COUNT},${FAIL_COUNT},${NUM_PASSES},${RUN_TIME},${TRIP_TIME},${TRIP_LENGTH},${NUM_MANEUVERS}" >> ${TOTAL_MULTI_RUN_STATS_FILENAME}

echo `date`
exit

