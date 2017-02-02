
PivoteDate=`grep kPivotDate ../../baldr/valhalla/baldr/graphconstants.h | awk -F'=' '{print $2}' | awk -F';' '{print $1}' | tr -d "\"" | tr -d " "`

if [[ "$1" == "pg" ]]; then
  db=gtfs
  dbuser=dbuser
elif [[ "$1" == "sqlite" ]]; then
  db=transit.sqlite
fi

#clean up
cat /dev/null > data.sql
cat /dev/null > schedule.txt

for f in shapes.txt agency.txt stops.txt routes.txt trips.txt stop_times.txt calendar.txt calendar_dates.txt transfers.txt
do
  if [ -f $f ]; then

    #remove the ^M
    tr -d '\r' < $f > $f.bk
    sed '/^$/d' $f.bk > $f
    rm $f.bk
    
    #fix up some of the feeds
    if [[ "$f" ==  "stops.txt" ]]; then
      sed -i 's/wheelchair_accessible/wheelchair_boarding/g' stops.txt
    fi

    if [[ "$f" ==  "trips.txt" ]]; then
      sed -i 's/wheelchair_boarding/wheelchair_accessible/g' trips.txt
    fi

    #columns that we need/want
    headers=$(head -1 $f.header)

    #columns we have in the .txt file
    cols=$(head -1 $f)

    cols=$(echo $cols | tr -d \")

    #determine where the columns are that we want.
    COL=$(awk -F, -v headers="${headers}" -v cols="${cols}" '
    BEGIN {
      n=split(headers,header)
      m=split(cols,col)
      for (i=1; i<=n; i++) {
        for (j=1; j<=m; j++) {
          if (col[j] == header[i] || col[j] == "header[i]")    
            print j-1;
        }
      }
    }')

    COL=$(echo $COL | tr " " ,)

    #dump out the columns we want
    csvfilter -f $COL $f --out-delimiter="|" > $f.bk
    mv $f.bk $f

    #remove the ^M again due to csvfilter
    tr -d '\r' < $f > $f.bk
    sed '/^$/d' $f.bk > $f
    rm $f.bk
   
    cols=$(head -1 $f)
    cols=$(echo $cols | tr "|" ,) 

    if [[ "$1" ==  "pg" ]]; then
      echo "copy ${f%%.*}_tmp(${cols}) from '$PWD/$f' with delimiter '|' csv header;" >> data.sql
      chmod 755 ./data.sql
    elif [[ "$1" ==  "sqlite" ]]; then
      echo "termsql -a -i $PWD/$f -c '${cols}' -1 -d '|' -t ${f%%.*}_tmp -o ${db} &> /dev/null &" >> data.sql
      chmod 755 ./data.sql
    fi
  fi

done

if [[ "$1" ==  "pg" ]]; then

  #wipe db as needed.
  if [[ "$2" ==  "clean" ]]; then
    psql -U $dbuser -f ./create_tables_pg.sql ${db} || exit $?
  fi 

  psql -U $dbuser -f ./data.sql ${db} || exit $?
  psql -U $dbuser -f ./mid_updates.sql ${db} || exit $?

  psql -U $dbuser $db -c "delete from calendar_tmp where CAST(end_date as integer) < CAST('${PivoteDate}' as integer);"
  psql -U $dbuser $db -c "delete from calendar_dates_tmp where CAST(date as integer) < CAST('${PivoteDate}' as integer);"

  calendar_type="normal"
  ./build_schedule.sh $db $1 $calendar_type $dbuser
  calendar_type="exceptions"
  ./build_schedule.sh $db $1 $calendar_type $dbuser

  psql -U $dbuser $db -c "insert into shapes select * from shapes_tmp;"
  psql -U $dbuser $db -c "insert into shape select * from shape_tmp;"
  psql -U $dbuser $db -c "insert into agency select * from agency_tmp;"
  psql -U $dbuser $db -c "insert into stops select * from stops_tmp;"
  psql -U $dbuser $db -c "insert into routes select * from  routes_tmp;"
  psql -U $dbuser $db -c "insert into trips select * from trips_tmp;"
  psql -U $dbuser $db -c "insert into stop_times select * from stop_times_tmp;"
  psql -U $dbuser $db -c "insert into calendar select * from calendar_tmp;"
  psql -U $dbuser $db -c "insert into calendar_dates select * from calendar_dates_tmp;"
  psql -U $dbuser $db -c "insert into transfers select * from transfers_tmp;"
  psql -U $dbuser $db -c "insert into schedule select * from schedule_tmp;"
  
  psql -U $dbuser $db -c "TRUNCATE TABLE shapes_tmp;"
  psql -U $dbuser $db -c "TRUNCATE TABLE shape_tmp;"
  psql -U $dbuser $db -c "TRUNCATE TABLE agency_tmp;"
  psql -U $dbuser $db -c "TRUNCATE TABLE stops_tmp;"
  psql -U $dbuser $db -c "TRUNCATE TABLE routes_tmp;"
  psql -U $dbuser $db -c "TRUNCATE TABLE trips_tmp;"
  psql -U $dbuser $db -c "TRUNCATE TABLE stop_times_tmp;"
  psql -U $dbuser $db -c "TRUNCATE TABLE calendar_tmp;"
  psql -U $dbuser $db -c "TRUNCATE TABLE calendar_dates_tmp;"
  psql -U $dbuser $db -c "TRUNCATE TABLE cal_dates_tmp;"
  psql -U $dbuser $db -c "TRUNCATE TABLE transfers_tmp;"
  psql -U $dbuser $db -c "TRUNCATE TABLE schedule_tmp;"
  psql -U $dbuser $db -c "DROP Table s_tmp;"
  psql -U $dbuser $db -c "DROP Table s_dates_tmp;"

  if [[ "$2" ==  "clean" ]]; then
    psql -U $dbuser $db -c "CREATE INDEX shapes_index ON shapes USING GIST (geom);"
    psql -U $dbuser $db -c "CREATE INDEX shape_index ON shape USING GIST (geom);"
    psql -U $dbuser $db -c "CREATE INDEX stops_index ON stops USING GIST (geom);"
    psql -U $dbuser $db -c "CREATE INDEX stop_key_index ON stop_times USING btree (stop_key);"
  fi

elif [[ "$1" ==  "sqlite" ]]; then

  echo "wait" >> data.sql
     
  #wipe db as needed.
  if [[ "$2" == "clean" ]]; then
    rm $db
    cat ./create_tables_sqlite.sql | spatialite $db || exit $?
    ./data.sql $db || exit $?
    cat ./geom.sql | spatialite $db || exit $?
  else
    cat ./shapes.sql | spatialite $db || exit $?
    ./data.sql $db || exit $? 
    cat ./geom.sql | spatialite $db || exit $?
  fi

  spatialite $db "delete from calendar_tmp where CAST(end_date as integer) < CAST('${PivoteDate}' as integer);" 
  spatialite $db "delete from calendar_dates_tmp where CAST(date as integer) < CAST('${PivoteDate}' as integer);"

  cat ./mid_updates_sqlite.sql | spatialite $db || exit $?

  calendar_type="normal"
  ./build_schedule.sh $db $1 $calendar_type 
  calendar_type="exceptions"
  ./build_schedule.sh $db $1 $calendar_type

  spatialite $db "insert into shapes select * from shapes_tmp;"
  spatialite $db "insert into shape select * from shape_tmp;"

  cols=$(head -1 stops.txt)
  cols=$(echo $cols | tr "|" ,)  
  spatialite $db "insert into stops(${cols},parent_station_key,geom) select ${cols},parent_station_key,NULL from stops_tmp;"
  spatialite $db "update stops set geom = SetSRID(MakePoint(stop_lon, stop_lat),4326) where geom is null;"
  
  spatialite $db "insert into agency select * from agency_tmp;"
  spatialite $db "insert into stops select * from stops_tmp;"
  spatialite $db "insert into routes select * from  routes_tmp;"
  spatialite $db "insert into trips select * from trips_tmp;"
  spatialite $db "insert into stop_times select * from stop_times_tmp;"
  spatialite $db "insert into calendar select * from calendar_tmp;"
  spatialite $db "insert into calendar_dates select * from calendar_dates_tmp;"
  spatialite $db "insert into transfers select * from transfers_tmp;"
  spatialite $db "insert into schedule select * from schedule_tmp;"

  spatialite $db "DELETE from shapes_tmp;"
  spatialite $db "DELETE from shape_tmp;"
  spatialite $db "DELETE from agency_tmp;"
  spatialite $db "DELETE from stops_tmp;"
  spatialite $db "DELETE from routes_tmp;"
  spatialite $db "DELETE from trips_tmp;"
  spatialite $db "DELETE from stop_times_tmp;"
  spatialite $db "DELETE from calendar_tmp;"
  spatialite $db "DELETE from calendar_dates_tmp;"
  spatialite $db "DELETE from cal_dates_tmp;"
  spatialite $db "DELETE from transfers_tmp;"
  spatialite $db "DELETE from schedule_tmp;"
  spatialite $db "DROP Table s_tmp;"
  spatialite $db "DROP Table s_dates_tmp;"
  spatialite $db "VACUUM ANALYZE;"
 
  if [[ "$2" ==  "clean" ]]; then
    echo "SELECT CreateSpatialIndex('shapes', 'geom');" | spatialite $db
    echo "SELECT CreateSpatialIndex('shape', 'geom');" | spatialite $db
    echo "SELECT CreateSpatialIndex('stops', 'geom');" | spatialite $db
    echo "CREATE INDEX stops_key_index ON stops (stop_key);" | spatialite $db
  fi

fi

rm *.txt

exit 0

