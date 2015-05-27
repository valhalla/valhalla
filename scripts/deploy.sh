#!/bin/bash
set -e

base_dir=$1
config=$2
src_dir=$3
extracts_dir=$4
log=$5

# make sure only one is running at any time...
LOCK_FILE="${base_dir}/locks/mjolnir.lock"
mkdir -p "${base_dir}/locks"

(set -C; : > ${LOCK_FILE}) 2> /dev/null

if [ $? != "0" ]; then
   echo "Lock File exists - exiting"
   exit 1
fi

conf_dir=$(dirname "${src_dir}/mjolnir/conf")
config_dir=$(dirname "${config}")
lua_files=`find ${conf_dir} -type f -name "*.lua"`

for file in ${lua_files} ; do
  file_name=`find ${file} -type f -name "*.lua" -printf '%f '`
  ln -s ${file} ${config_dir}/${file_name}
done

extracts=`find ${extracts_dir} -type f -name "*.pbf"`
files=`find ${extracts_dir} -type f -name "*.pbf" -printf '%f '`

# update each pbf
for file in ${files} ; do
  ${src_dir}/mjolnir/scripts/minutely_update.sh update ${extracts_dir} ${file}
done

tile_dir=`cat ${config} | jq -r '.mjolnir.hierarchy.tile_dir'`
mjolnir_tile_dir=$(echo ${tile_dir} | sed 's/tiles/mjolnir_tiles/g')

sed -i 's/tiles/mjolnir_tiles/g' ${config}

# clean mjolnir tiles
rm -rf ${mjolnir_tile_dir}/*

export PATH=$PATH:/usr/local/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

# create admins
pbfadminbuilder -c ${config} ${extracts}

# move the admin log
mv ${log} ${log}.admins

log_dir=$(dirname "${log}")

# cut tiles from the data
pbfgraphbuilder -c ${config} ${extracts}

# clean tile dir 
rm -rf ${tile_dir}/*

# move the newly created tiles to the tile dir
mv ${mjolnir_tile_dir}/* ${tile_dir}/

# cp admin db
db_name=`cat ${config} | jq -r '.mjolnir.admin.db_name'`
cp ${tile_dir}/${db_name} ${mjolnir_tile_dir}/${db_name}

# clean up
rm -rf ${base_dir}/*.bin

# remove cron jobs.
crontab -r

# add new cron job for updates.
(crontab -l 2>/dev/null; echo "*/5 * * * * cd ${base_dir}; ${src_dir}/mjolnir/scripts/update_tiles.sh ${base_dir} ${config} ${src_dir} ${extracts_dir} >> ${log_dir}/update_cron.log 2>&1") | crontab -

# add new cron job for log clean up.
(crontab -l 2>/dev/null; echo "0 0 * * 0 rm ${log_dir}/*.log") | crontab -

rm ${LOCK_FILE}

