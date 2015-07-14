db=$1
calendar_type=$3

if [ -f schedule.txt ]; then
  rm schedule.txt
fi
if [ -f schedule.tmp ]; then
  rm schedule.tmp
fi

for f in schedule.*.tmp;
do
  if [ -f $f ]; then
    rm $f
  fi 
  if [ -f $f.txt ]; then
    rm $f.txt
  fi 
done 

if [[ "$2" ==  "pg" ]]; then
  dbuser=$4
fi

./split.sh $calendar_type $db $2 $dbuser

for f in schedule.*.tmp;
do
   ./build.sh $calendar_type $f $f.txt &
done
wait

echo "origin_stop_key,dest_stop_key,trip_key,route_key,service_key,shape_key,departure_time,arrival_time,start_date,end_date,dow_mask,has_subtractions,block_id,headsign,wheelchair_accessible,bikes_allowed" >> schedule.txt

for f in schedule.*.tmp;
do

  if [ -f $f.txt ]; then
    cat $f.txt >> schedule.txt
    rm $f.txt
  fi

  if [ -f $f ]; then
    rm $f
  fi

done

if [[ "$2" ==  "pg" ]]; then
  psql -U $dbuser $db -c "copy schedule_tmp(origin_stop_key,dest_stop_key,trip_key,route_key,service_key,shape_key,departure_time,arrival_time,start_date,end_date,dow_mask,has_subtractions,block_id,headsign,wheelchair_accessible,bikes_allowed) from '$PWD/schedule.txt' with delimiter ',' csv header;"

  if [[ "$calendar_type" ==  "exceptions" ]]; then
    psql -U $dbuser $db -c "delete from calendar_dates_tmp where exception_type = 1;"
    psql -U $dbuser $db -c "update schedule_tmp set has_subtractions = 1 where schedule_tmp.service_key = (select distinct(calendar_dates_tmp.service_key) from calendar_dates_tmp where schedule_tmp.service_key = calendar_dates_tmp.service_key);"
  fi

  psql -U $dbuser $db -c "update trips_tmp set block_id = '0' where block_id = '' or block_id is null;"
  psql -U $dbuser $db -c "update schedule_tmp set block_id = '0' where block_id = '' or block_id is null;"
  psql -U $dbuser $db -c "VACUUM ANALYZE;"
elif [[ "$2" ==  "sqlite" ]]; then

  termsql -a -i $PWD/schedule.txt -c 'origin_stop_key,dest_stop_key,trip_key,route_key,service_key,shape_key,departure_time,arrival_time,start_date,end_date,dow_mask,has_subtractions,block_id,headsign,wheelchair_accessible,bikes_allowed' -1 -d ',' -t schedule_tmp -o $db &> /dev/null
  #echo -e '.separator ","\n.import '$PWD'/schedule.txt schedule_tmp' | sqlite3 $db

  if [[ "$calendar_type" ==  "exceptions" ]]; then
    spatialite $db "delete from calendar_dates_tmp where exception_type = 1;"
    spatialite $db "update schedule_tmp set has_subtractions = 1 where schedule_tmp.service_key = (select distinct(calendar_dates_tmp.service_key) from calendar_dates_tmp where schedule_tmp.service_key = calendar_dates_tmp.service_key);"
  fi  

  spatialite $db "update trips_tmp set block_id = '0' where block_id = '' or block_id is null;"
  spatialite $db "update schedule_tmp set block_id = '0' where block_id = '' or block_id is null;"
  spatialite $db "VACUUM ANALYZE;"
fi
