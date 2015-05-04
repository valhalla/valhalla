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
   exit 0
fi

extracts=`find ${extracts_dir} -type f -name "*.pbf"`
files=`find ${extracts_dir} -type f -name "*.pbf" -printf '%f '`

# update each pbf
#for file in ${files}; do
#  ${src_dir}/mjolnir/scripts/minutely_update.sh update ${extracts_dir} ${file} || exit $?
#done

mjolnir_tile_dir=`cat ${config} | jq '.mjolnir.hierarchy.tile_dir' | sed 's/^"\(.*\)"$/\1/'` || exit $?
tile_dir=$(echo ${mjolnir_tile_dir} | sed 's/mjolnir_tiles/tiles/g') || exit $?

export PATH=$PATH:/usr/local/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

# cut tiles from the data
pbfgraphbuilder -c ${config} ${extracts} || exit $?

# backup tiles
${src_dir}/mjolnir/scripts/backup_tiles.sh ${tile_dir} || exit $?

# clean tile dir 
rm -rf ${tile_dir}/* || exit $?

exit

# move the newly created tiles to the tile dir
mv ${mjolnir_tile_dir}/* ${tile_dir}/ || exit $?

# cp admin db
db_name=`cat ${config} | jq '.mjolnir.admin.db_name' | sed 's/^"\(.*\)"$/\1/'` || exit $?
cp ${tile_dir}/${db_name} ${mjolnir_tile_dir}/${db_name} || exit $?

# clean backup tiles
${src_dir}/mjolnir/scripts/clean_tiles.sh ${tile_dir} 2 || exit $?

# clean up
rm -rf ${base_dir}/*.bin || exit $?

rm ${LOCK_FILE} || exit $?

