db=$1

if [ -f schedule.txt ]; then
  rm schedule.txt
fi
if [ -f schedule.tmp ]; then
  rm schedule.tmp
fi

if [[ "$2" ==  "pg" ]]; then
  dbuser=$3
  psql -U $dbuser -c "copy (select stop_key,trip_key,trip_id,route_key,service_key,departure_time,arrival_time,start_date,end_date,has_subtractions,block_id,headsign from s_dates_tmp order by trip_id,start_date, stop_sequence) to STDOUT csv" $db > ./schedule.tmp
elif [[ "$2" == "sqlite" ]]; then 
  sqlite3 $db -csv "select stop_key,trip_key,trip_id,route_key,service_key,departure_time,arrival_time,start_date,end_date,has_subtractions,block_id,headsign from s_dates_tmp order by trip_id,start_date, stop_sequence;" > ./schedule.tmp
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
while IFS=, read stop_key trip_key trip_id route_key service_key departure_time arrival_time start_date end_date has_subtractions block_id headsign; do

    if [[ "$trip_id" == "$trip" && "$start_date" == "$startdate" ]]; then
       echo "$origin,$stop_key,$trip_key,$route_key,$service_key,$departure,$arrival_time,$start_date,$end_date,$dow_mask,$has_subtractions,$block_id,$headsign" >> schedule.txt
    fi
    
    startdate=$start_date
    trip=$trip_id
    origin=$stop_key
    departure=$departure_time

    dow_mask=$NONE

    dow=`date --date="${start_date}" "+%u"`

    if [[ $dow -eq 7 ]]; then dow_mask=$(($dow_mask|$SUNDAY));fi
    if [[ $dow -eq 1 ]]; then dow_mask=$(($dow_mask|$MONDAY));fi
    if [[ $dow -eq 2 ]]; then dow_mask=$(($dow_mask|$TUESDAY));fi
    if [[ $dow -eq 3 ]]; then dow_mask=$(($dow_mask|$WEDNESDAY));fi
    if [[ $dow -eq 4 ]]; then dow_mask=$(($dow_mask|$THURSDAY));fi
    if [[ $dow -eq 5 ]]; then dow_mask=$(($dow_mask|$FRIDAY));fi
    if [[ $dow -eq 6 ]]; then dow_mask=$(($dow_mask|$SATURDAY));fi

done < ./schedule.tmp

if [[ "$2" == "pg" ]]; then
  psql -U $dbuser $db -c "copy schedule_tmp(origin_stop_key,dest_stop_key,trip_key,route_key,service_key,departure_time,arrival_time,start_date,end_date,dow_mask,has_subtractions,block_id,headsign) from '$PWD/schedule.txt' with delimiter ',' csv header;"
  psql -U $dbuser $db -c "delete from calendar_dates_tmp where exception_type = 1;"
  psql -U $dbuser $db -c "update schedule_tmp set has_subtractions = 1 from calendar_dates_tmp where schedule_tmp.service_key = calendar_dates_tmp.service_key;"
  psql -U $dbuser $db -c "update trips_tmp set block_id = '0' where block_id = '' or block_id is null;"
  psql -U $dbuser $db -c "update schedule_tmp set block_id = '0' where block_id = '' or block_id is null;"
  psql -U $dbuser $db  -c "VACUUM ANALYZE;"
elif [[ "$2" == "sqlite" ]]; then

  termsql -a -i $PWD/schedule.txt -c 'origin_stop_key,dest_stop_key,trip_key,route_key,service_key,departure_time,arrival_time,start_date,end_date,dow_mask,has_subtractions,block_id,headsign' -1 -d ',' -t schedule_tmp -o $db

  spatialite $db "delete from calendar_dates_tmp where exception_type = 1;"
  spatialite $db "update schedule_tmp set has_subtractions = 1 from calendar_dates_tmp where schedule_tmp.service_key = calendar_dates_tmp.service_key;"
  spatialite $db "update trips_tmp set block_id = '0' where block_id = '' or block_id is null;"
  spatialite $db "update schedule_tmp set block_id = '0' where block_id = '' or block_id is null;"
  spatialite $db "VACUUM ANALYZE;"
fi
