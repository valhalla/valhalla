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

cd ${extracts_dir}
# update each pbf
for file in *.pbf ; do
  ${src_dir}/mjolnir/scripts/minutely_update.sh update ${extracts_dir} ${file} || exit $?
done

mjolnir_tile_dir=`cat ${config} | jq '.mjolnir.hierarchy.tile_dir' | sed 's/^"\(.*\)"$/\1/'` || exit $?
tile_dir=$(echo ${mjolnir_tile_dir} | sed 's/mjolnir_tiles/tiles/g') || exit $?

# clean mjolnir tiles
rm -rf ${mjolnir_tile_dir}/* || exit $?

# cut tiles from the data
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib pbfgraphbuilder -c ${config} ${extracts} || exit $?

# backup tiles
${src_dir}/mjolnir/scripts/backup_tiles.sh ${tile_dir} || exit $?

# clean tile dir 
rm -rf ${tile_dir}/* || exit $?

# move the newly created tiles to the tile dir
mv ${mjolnir_tile_dir}/* ${tile_dir}/ || exit $?

# clean backup tiles
${src_dir}/mjolnir/scripts/clean_tiles.sh ${tile_dir} 2 || exit $?

# clean up
rm -rf ${base_dir}/*.bin || exit $?

rm ${LOCK_FILE} || exit $?



