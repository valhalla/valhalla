#!/bin/bash

CONF_FILE=$1
DIR=$2
TEST_FILE=$3

pwd_dir=`pwd`
cd ${DIR}

#fill out the template with a date relative to now
sed "s/DATE_TIME_TAG/`date --date='08:00 next Tue' +%Y-%m-%dT%H:%M`/g" ${DIR}/${TEST_FILE} > {$DIR}/transit_routes.txt
route_count=$(wc -l ${DIR}/test_requests/${TEST_FILE} | awk '{print $1}')
#RAD those tests
./run.sh {$DIR}/transit_routes.txt ${CONF_FILE}
#check whats going on
succeed_count=`grep -ic success ${DIR}/results/*_transit_routes/statistics.csv`
#this doesnt look good
cd $pwd_dir
if [[ ${succeed_count} != ${route_count} ]]; then
  echo "[FAILURE] Tests failed."
  exit 1
fi
rm -rf ${DIR}/results
