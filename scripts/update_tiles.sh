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

#wait until lock is removed 
if [ $? != "0" ]; then
   echo "Lock file exists"
   exit 0
fi

extracts=`find ${extracts_dir} -type f -name "*.pbf"`
files=`find ${extracts_dir} -type f -name "*.pbf" -printf '%f '`

# update each pbf
for file in ${files}; do
  ${src_dir}/mjolnir/scripts/minutely_update.sh update ${extracts_dir} ${file}
done

mjolnir_tile_dir=`cat ${config} | jq -r '.mjolnir.hierarchy.tile_dir'`
tile_dir=$(echo ${mjolnir_tile_dir} | sed 's/mjolnir_tiles/tiles/g')

export PATH=$PATH:/usr/local/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

# cut tiles from the data
pbfgraphbuilder -c ${config} ${extracts}

# backup tiles
date_time=`date +%Y_%m_%d_%H%M%S`
mv ${tile_dir} ${tile_dir}.$date_time
mkdir -p ${tile_dir}

# clean tile dir 
rm -rf ${tile_dir}/*

# move the newly created tiles to the tile dir
mv ${mjolnir_tile_dir}/* ${tile_dir}/

# cp admin db
db_name=`cat ${config} | jq -r '.mjolnir.admin.db_name'`
cp -rp ${tile_dir}/${db_name} ${mjolnir_tile_dir}/${db_name}

# generate connectivity map geojson, tile dir is as good a place as any
# we can ship the whole tile dir to s3 anyway, admin connectivity and all
pushd ${tile_dir}/
connectivitymap -c ${config}
popd

# clean backup tiles
${src_dir}/mjolnir/scripts/clean_tiles.sh ${tile_dir} 2

# clean up
rm -rf ${base_dir}/*.bin

rm ${LOCK_FILE}

