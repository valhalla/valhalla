function error_exit
{
  echo "$1" 1>&2
  exit 1
}

if [ -z "$1" ]; then
    echo "No config supplied.  Usage: ./create_tz_db.sh /data/valhalla/mjolnir/conf/valhalla.json"
    exit 1
fi

rm -rf world
rm -f ./tz_world_mp.zip

config=$1
if [ ! -f $config ]; then
    echo "Config file not found $config"
    exit 1
fi
tz_dir=$(jq -r '.mjolnir.timezone.timezone_dir' $config)
if [ ! -d $tz_dir ]; then
    echo "Timezone directory not found $tz_dir"
    exit 1
fi
tz_file=$tz_dir/$(jq -r '.mjolnir.timezone.db_name' $config)
rm -f $tz_file
url="http://efele.net/maps/tz/world/tz_world_mp.zip"
wget $url || error_exit "wget failed for " $url
unzip ./tz_world_mp.zip || error_exit "unzip failed"
spatialite_tool -i -shp ./world/tz_world_mp -d $tz_file -t tz_world -s 4326 -g geom -c UTF8 || error_exit "spatialite_tool import failed"
spatialite $tz_file "SELECT CreateSpatialIndex('tz_world', 'geom');" || error_exit "SpatialIndex failed" 

rm -rf world
rm -f ./tz_world_mp.zip

