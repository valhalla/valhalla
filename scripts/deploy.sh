#!/bin/bash
set -e

base_dir=$1
config=$2
src_dir=$3
extracts_dir=$4
log_dir=$5

# make sure only one is running at any time...
LOCK_FILE="${base_dir}/locks/deploy.lock"
mkdir -p "${base_dir}/locks"
(set -C; : > ${LOCK_FILE}) 2> /dev/null
if [ $? != "0" ]; then
   echo "Lock file exists"
   exit 1
fi
trap 'rm $LOCK_FILE' EXIT 1 2 3 6

# if they want updates then we need to install crontabs
if [ $WITH_UPDATES ]; then
  # remove cron jobs.
  crontab -r

  # add new cron job for updates.
  (crontab -l 2>/dev/null; echo "*/5 * * * * cd ${base_dir}; WITH_UPDATES=YES ${src_dir}/mjolnir/scripts/update_tiles.sh ${base_dir} ${config} ${src_dir} ${extracts_dir} >> ${log_dir}/update_cron.log 2>&1") | crontab -

  # add new cron job for log clean up.
  (crontab -l 2>/dev/null; echo "0 0 * * 0 rm ${log_dir}/*.log") | crontab -
# they just want us to cut tiles once
else
  ${src_dir}/mjolnir/scripts/update_tiles.sh ${base_dir} ${config} ${src_dir} ${extracts_dir}
fi
