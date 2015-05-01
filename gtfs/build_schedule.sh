db=$1

if [ -f schedule.txt ]; then
  rm schedule.txt
fi
if [ -f schedule.tmp ]; then
  rm schedule.tmp
fi

if [[ "$2" ==  "pg" ]]; then
  dbuser=$3
  psql -U $dbuser -c "copy (select stop_key,trip_key,trip_id,route_key,service_key,departure_time,arrival_time,start_date,end_date,sunday,monday,tuesday,wednesday,thursday,friday,saturday,headsign from s_tmp order by trip_id, stop_sequence) to STDOUT csv" $db > ./schedule.tmp
elif [[ "$2" ==  "sqlite" ]]; then 
  sqlite3 $db -csv "select stop_key,trip_key,trip_id,route_key,service_key,departure_time,arrival_time,start_date,end_date,sunday,monday,tuesday,wednesday,thursday,friday,saturday,headsign from s_tmp order by trip_id, stop_sequence;" > ./schedule.tmp
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

echo "origin_stop_key,dest_stop_key,trip_key,route_key,service_key,departure_time,arrival_time,start_date,end_date,dow_mask,headsign" >> schedule.txt

#Creating the stop pairs and calendar bitmask.
while IFS=, read stop_key trip_key trip_id route_key service_key departure_time arrival_time start_date end_date sun mon tue wed thu fri sat headsign; do

      if [[ "$trip_id" ==  "$trip" ]]; then
         echo "$origin,$stop_key,$trip_key,$route_key,$service_key,$departure,$arrival_time,$start_date,$end_date,$dow_mask,$headsign" >> schedule.txt
      fi
    
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
  psql -U $dbuser gtfs -c "copy schedule_tmp(origin_stop_key,dest_stop_key,trip_key,route_key,service_key,departure_time,arrival_time,start_date,end_date,dow_mask,headsign) from '$PWD/schedule.txt' with delimiter ',' csv header;"
  psql -U $dbuser gtfs -c "VACUUM ANALYZE;"
elif [[ "$2" ==  "sqlite" ]]; then

  termsql -a -i $PWD/schedule.txt -c 'origin_stop_key,dest_stop_key,trip_key,route_key,service_key,departure_time,arrival_time,start_date,end_date,dow_mask,headsign' -1 -d ',' -t schedule_tmp -o $db
  sqlite3 $db "VACUUM ANALYZE;"
fi
