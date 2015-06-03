db=$1

if [ -f schedule.txt ]; then
  rm schedule.txt
fi
if [ -f schedule.tmp ]; then
  rm schedule.tmp
fi

if [[ "$2" ==  "pg" ]]; then
  dbuser=$3
  psql -U $dbuser -c "copy (select stop_key,trip_key,trip_id,route_key,service_key,departure_time,arrival_time,start_date,end_date,sunday,monday,tuesday,wednesday,thursday,friday,saturday,has_subtractions,block_id,headsign from s_tmp order by trip_id, stop_sequence) to STDOUT csv" $db > ./schedule.tmp
elif [[ "$2" ==  "sqlite" ]]; then 
  sqlite3 $db -csv "select stop_key,trip_key,trip_id,route_key,service_key,departure_time,arrival_time,start_date,end_date,sunday,monday,tuesday,wednesday,thursday,friday,saturday,has_subtractions,block_id,headsign from s_tmp order by trip_id, stop_sequence;" > ./schedule.tmp
fi

tr -d '"' < ./schedule.tmp > ./schedule.tmp.new
mv ./schedule.tmp.new ./schedule.tmp

NONE=0
SUNDAY=1
MONDAY=2
TUESDAY=4
WEDNESDAY=8
THURSDAY=16
FRIDAY=32
SATURDAY=64

dow_mask=$NONE
dest=false
origin=""
departure=""
trip=""
startdate=""

echo "origin_stop_key,dest_stop_key,trip_key,route_key,service_key,departure_time,arrival_time,start_date,end_date,dow_mask,has_subtractions,block_id,headsign" >> schedule.txt

#Creating the stop pairs and calendar bitmask.
while IFS=, read stop_key trip_key trip_id route_key service_key departure_time arrival_time start_date end_date sun mon tue wed thu fri sat has_subtractions block_id headsign; do

    if [[ "$trip_id" ==  "$trip" && "$start_date" == "$startdate" ]]; then
       echo "$origin,$stop_key,$trip_key,$route_key,$service_key,$departure,$arrival_time,$start_date,$end_date,$dow_mask,$has_subtractions,$block_id,$headsign" >> schedule.txt
    fi

    startdate=$start_date
    trip=$trip_id
    origin=$stop_key
    departure=$departure_time

    dow_mask=$NONE

    if [[ $sun -eq 1 ]]; then dow_mask=$(($dow_mask|$SUNDAY));fi
    if [[ $mon -eq 1 ]]; then dow_mask=$(($dow_mask|$MONDAY));fi
    if [[ $tue -eq 1 ]]; then dow_mask=$(($dow_mask|$TUESDAY));fi
    if [[ $wed -eq 1 ]]; then dow_mask=$(($dow_mask|$WEDNESDAY));fi
    if [[ $thu -eq 1 ]]; then dow_mask=$(($dow_mask|$THURSDAY));fi
    if [[ $fri -eq 1 ]]; then dow_mask=$(($dow_mask|$FRIDAY));fi
    if [[ $sat -eq 1 ]]; then dow_mask=$(($dow_mask|$SATURDAY));fi

done < ./schedule.tmp

if [[ "$2" ==  "pg" ]]; then
  psql -U $dbuser $db -c "copy schedule_tmp(origin_stop_key,dest_stop_key,trip_key,route_key,service_key,departure_time,arrival_time,start_date,end_date,dow_mask,has_subtractions,block_id,headsign) from '$PWD/schedule.txt' with delimiter ',' csv header;"
  psql -U $dbuser $db -c "update trips_tmp set block_id = '0' where block_id = '' or block_id is null;"
  psql -U $dbuser $db -c "update schedule_tmp set block_id = '0' where block_id = '' or block_id is null;"
  psql -U $dbuser $db -c "VACUUM ANALYZE;"
elif [[ "$2" ==  "sqlite" ]]; then

  termsql -a -i $PWD/schedule.txt -c 'origin_stop_key,dest_stop_key,trip_key,route_key,service_key,departure_time,arrival_time,start_date,end_date,dow_mask,has_subtractions,block_id,headsign' -1 -d ',' -t schedule_tmp -o $db
  spatialite $db "update trips_tmp set block_id = '0' where block_id = '' or block_id is null;"
  spatialite $db "update schedule_tmp set block_id = '0' where block_id = '' or block_id is null;"
  spatialite $db "VACUUM ANALYZE;"
fi
