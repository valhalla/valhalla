calendar_type=$1
inputfile=$2
outputfile=$3

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

if [[ "$calendar_type" !=  "exceptions" ]]; then
#Creating the stop pairs and calendar bitmask.
  while IFS=, read stop_key trip_key trip_id route_key service_key shape_key departure_time arrival_time start_date end_date sun mon tue wed thu fri sat has_subtractions block_id headsign wheelchair_accessible bikes_allowed; do

    if [[ "$trip_id" ==  "$trip" && "$start_date" == "$startdate" ]]; then
       echo "$origin,$stop_key,$trip_key,$route_key,$service_key,$shape_key,$departure,$arrival_time,$start_date,$end_date,$dow_mask,$has_subtractions,$block_id,$headsign,$wheelchair_accessible,$bikes_allowed" >> $outputfile
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

  done < ./$inputfile
else
#Creating the stop pairs and calendar bitmask.
  while IFS=, read stop_key trip_key trip_id route_key service_key shape_key departure_time arrival_time start_date end_date has_subtractions block_id headsign wheelchair_accessible bikes_allowed; do

    if [[ "$trip_id" == "$trip" && "$start_date" == "$startdate" ]]; then
       echo "$origin,$stop_key,$trip_key,$route_key,$service_key,$shape_key,$departure,$arrival_time,$start_date,$end_date,$dow_mask,$has_subtractions,$block_id,$headsign,$wheelchair_accessible,$bikes_allowed" >> $outputfile
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

  done < ./$inputfile
fi
