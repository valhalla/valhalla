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

extracts=`find ${extracts_dir} -type f -name "*.pbf"`

#cd ${extracts_dir}
# update each pbf
#for file in *.pbf ; do
#  ${src_dir}/mjolnir/scripts/minutely_update.sh update ${extracts_dir} ${file} || exit $?
#done

mjolnir_tile_dir=`cat ${config} | jq '.mjolnir.hierarchy.tile_dir' | sed 's/^"\(.*\)"$/\1/'` || exit $?
tile_dir=$(echo ${mjolnir_tile_dir} | sed 's/mjolnir_tiles/tiles/g') || exit $?

# clean mjolnir tiles
rm -rf ${mjolnir_tile_dir}/* || exit $?

# create admins
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib pbfadminbuilder -c ${config} ${extracts} || exit $?

# move the admin log
mv ${log} ${log}.admins || exit $?

# cut tiles from the data
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib pbfgraphbuilder -c ${config} ${extracts} || exit $?

# clean tile dir 
rm -rf ${tile_dir}/* || exit $?

# move the newly created tiles to the tile dir
mv ${mjolnir_tile_dir}/* ${tile_dir}/ || exit $?

# clean up
rm -rf ${base_dir}/*.bin || exit $?

# remove cron jobs.
crontab -r

# add new cron job for updates.
(crontab -l 2>/dev/null; echo "*/5 * * * * ${src_dir}/mjolnir/scripts/update_tiles.sh ${base_dir} ${config} ${src_dir} ${extracts_dir}") | crontab -

rm ${LOCK_FILE} || exit $?

