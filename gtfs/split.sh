#It is much faster to split the schedule into multiple csv files, process the files, and cat the results 
#versus having one huge csv file. 

db=$2
calendar_type=$1

procs=`cat /proc/cpuinfo | awk '/^processor/{print $3}' | tail -1`

if [[ "$3" ==  "pg" ]]; then
  dbuser=$4
  
  if [[ "$calendar_type" !=  "exceptions" ]]; then
    start_trip_key=`psql -Atc -U $dbuser -d $db --c "select min(trip_key) from s_tmp;"`
    max=`psql -Atc -U $dbuser -d $db --c "select max(trip_key) from s_tmp;"`
  else
    start_trip_key=`psql -Atc -U $dbuser -d $db --c "select min(trip_key) from s_dates_tmp;"`
    max=`psql -Atc -U $dbuser -d $db --c "select max(trip_key) from s_dates_tmp;"`
  fi 

  total_recs=$((max-start_trip_key))
  rows=$((($total_recs/$procs) + ($total_recs % $procs > 0)))
  end_trip_key=$((start_trip_key+rows))
  counter=1

  while [  $start_trip_key -lt $max ]; do

#selecting and dumping a range
    if [[ "$calendar_type" !=  "exceptions" ]]; then
      psql -U $dbuser -c "copy (select stop_key,trip_key,trip_id,route_key,service_key,shape_key,departure_time,arrival_time,start_date,end_date,sunday,monday,tuesday,wednesday,thursday,friday,saturday,has_subtractions,block_id,headsign,wheelchair_accessible,bikes_allowed from s_tmp where (trip_key >= '$start_trip_key' and trip_key < '$end_trip_key' ) order by trip_id, stop_sequence) to STDOUT csv" $db > ./schedule.${counter}.tmp
    else
      psql -U $dbuser -c "copy (select stop_key,trip_key,trip_id,route_key,service_key,shape_key,departure_time,arrival_time,start_date,end_date,has_subtractions,block_id,headsign,wheelchair_accessible,bikes_allowed from s_dates_tmp where (trip_key >= '$start_trip_key' and trip_key < '$end_trip_key' ) order by trip_id,start_date, stop_sequence) to STDOUT csv" $db > ./schedule.${counter}.tmp
    fi

    tr -d '"' < ./schedule.${counter}.tmp > ./schedule.${counter}.tmp.new
    mv ./schedule.${counter}.tmp.new ./schedule.${counter}.tmp

    let start_trip_key=end_trip_key
    let end_trip_key=end_trip_key+rows

    if [  $end_trip_key -eq $max ]; then
      let end_trip_key+=1
    fi

    let counter+=1
  done

elif [[ "$3" ==  "sqlite" ]]; then

  if [[ "$calendar_type" !=  "exceptions" ]]; then
    start_trip_key=$( sqlite3 $db "select min(trip_key) from s_tmp;")
    max=$( sqlite3 $db "select max(trip_key) from s_tmp;")
  else
    start_trip_key=$( sqlite3 $db "select min(trip_key) from s_dates_tmp;")
    max=$( sqlite3 $db "select max(trip_key) from s_dates_tmp;")
  fi

  total_recs=$((max-start_trip_key))
  rows=$((($total_recs/$procs) + ($total_recs % $procs > 0)))
  end_trip_key=$((start_trip_key+rows))
  counter=1

  while [  $start_trip_key -lt $max ]; do

#selecting and dumping a range
    if [[ "$calendar_type" !=  "exceptions" ]]; then
      sqlite3 $db -csv "select stop_key,trip_key,trip_id,route_key,service_key,shape_key,departure_time,arrival_time,start_date,end_date,sunday,monday,tuesday,wednesday,thursday,friday,saturday,has_subtractions,block_id,headsign,wheelchair_accessible,bikes_allowed from s_tmp where (trip_key >= '$start_trip_key' and trip_key < '$end_trip_key' ) order by trip_id, stop_sequence;" > ./schedule.${counter}.tmp
    else
      sqlite3 $db -csv "select stop_key,trip_key,trip_id,route_key,service_key,shape_key,departure_time,arrival_time,start_date,end_date,has_subtractions,block_id,headsign,wheelchair_accessible,bikes_allowed from s_dates_tmp where (trip_key >= '$start_trip_key' and trip_key < '$end_trip_key' ) order by trip_id,start_date, stop_sequence;" > ./schedule.${counter}.tmp
    fi 

    tr -d '"' < ./schedule.${counter}.tmp > ./schedule.${counter}.tmp.new
    mv ./schedule.${counter}.tmp.new ./schedule.${counter}.tmp

    let start_trip_key=end_trip_key
    let end_trip_key=end_trip_key+rows

    if [  $end_trip_key -eq $max ]; then
      let end_trip_key+=1    
    fi

    let counter+=1
  done
fi
