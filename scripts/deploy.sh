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

cd ${extracts_dir}
# update each pbf
for file in *.pbf ; do
  ${src_dir}/mjolnir/scripts/minutely_update.sh update ${extracts_dir} ${file} || exit $?
done

cd ${base_dir}

tile_dir=`cat ${config} | jq '.mjolnir.hierarchy.tile_dir' | sed 's/^"\(.*\)"$/\1/'` || exit $?
mjolnir_tile_dir=$(echo ${tile_dir} | sed 's/tiles/mjolnir_tiles/g') || exit $?

sed -i 's/tiles/mjolnir_tiles/g' ${config} || exit $?

# clean mjolnir tiles
rm -rf ${mjolnir_tile_dir}/* || exit $?

export PATH=$PATH:/usr/local/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

# create admins
pbfadminbuilder -c ${config} ${extracts} || exit $?

# move the admin log
mv ${log} ${log}.admins || exit $?

log_dir=$(dirname "${log}") || exit $?

# cut tiles from the data
pbfgraphbuilder -c ${config} ${extracts} || exit $?

# clean tile dir 
rm -rf ${tile_dir}/* || exit $?

# move the newly created tiles to the tile dir
mv ${mjolnir_tile_dir}/* ${tile_dir}/ || exit $?

# cp admin db
db_name=`cat ${config} | jq '.mjolnir.admin.db_name' | sed 's/^"\(.*\)"$/\1/'` || exit $?
cp ${tile_dir}/${db_name} ${mjolnir_tile_dir}/${db_name} || exit $?

# clean up
rm -rf ${base_dir}/*.bin || exit $?

# remove cron jobs.
crontab -r

# add new cron job for updates.
(crontab -l 2>/dev/null; echo "*/5 * * * * cd ${base_dir}; ${src_dir}/mjolnir/scripts/update_tiles.sh ${base_dir} ${config} ${src_dir} ${extracts_dir} >> ${log_dir}/update_cron.log 2>&1") | crontab -

rm ${LOCK_FILE} || exit $?

