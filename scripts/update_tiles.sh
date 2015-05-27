#!/bin/bash
set -e

base_dir=$1
config=$2
src_dir=$3
extracts_dir=$4

# make sure only one is running at any time...
LOCK_FILE="${base_dir}/locks/mjolnir.lock"
mkdir -p "${base_dir}/locks"
(set -C; : > ${LOCK_FILE}) 2> /dev/null

# try again later when lock is removed 
if [ $? != "0" ]; then
   echo "Lock file exists"
   exit 0
fi

export PATH=$PATH:/usr/local/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

# only if they want updates will we update the pbfs
if [ $WITH_UPDATES ]; then
  files=`find ${extracts_dir} -type f -name "*.pbf" -printf '%f '`
  for file in ${files}; do
    ${src_dir}/mjolnir/scripts/minutely_update.sh update ${extracts_dir} ${file}
  done
fi

# get all the lua programs for data import
cp -rp ${src_dir}/mjolnir/conf/*.lua $(dirname ${config})

# make the dir where this will go
tile_dir=$(jq -r '.mjolnir.hierarchy.tile_dir' ${config})
cur_tile_dir=$(dirname ${tile_dir})/tiles_$(date +%Y_%m_%d-%H_%M_%S)/

# if we dont have admins we must create them
extracts=$(find ${extracts_dir} -type f -name "*.pbf")
admin_file=$(jq -r '.mjolnir.admin.admin_dir' ${config})/$(jq -r '.mjolnir.admin.db_name' ${config})
if [ ! -e $admin_file ]; then
  pbfadminbuilder -c ${config} ${extracts}
fi

# cut tiles from the data
pbfgraphbuilder -c ${config} $(find ${extracts_dir} -type f -name "*.pbf")
rm -rf *.bin

# generate connectivity map geojson, tile dir is as good a place as any
# we can ship the whole tile dir to s3 anyway, admin connectivity and all
pushd ${tile_dir}
connectivitymap -c ${config}
popd

# backup files and tile dirs, keep the admin stuff though
mkdir -p ${cur_tile_dir}
mv ${tile_dir}/* ${cur_tile_dir}
cp -rp ${cur_tile_dir}/$(basename admin_file) ${tile_dir}

# trim backed up tile directories to a certain number
count=0
for dir in $(ls -d tiles_.* | sort -r); do
   if [ $count -ge 3 ]; then
      rm -rf $dir
   fi
   let count=count+1
done

rm ${LOCK_FILE}

