
dbuser=<your db username>

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
    csvfilter -f $COL $f > $f.bk
    mv $f.bk $f

    #remove the ^M again due to csvfilter
    tr -d '\r' < $f > $f.bk
    sed '/^$/d' $f.bk > $f
    rm $f.bk

    cols=$(head -1 $f)
        
    echo "copy ${f%%.*}_tmp(${cols}) from '$PWD/$f' with delimiter ',' csv header;" >> data.sql
  fi

done

db=gtfs

#wipe db as needed.
if [[ "$1" ==  "clean" ]]; then
  psql -U $dbuser -f ./create_tables.sql ${db} || exit $?
fi 

psql -U $dbuser -f ./data.sql ${db} || exit $?
psql -U $dbuser -f ./mid_updates.sql ${db} || exit $?

./build_schedule.sh

psql -U $dbuser gtfs -c "insert into shapes select * from shapes_tmp;"
psql -U $dbuser gtfs -c "insert into agency select * from agency_tmp;"
psql -U $dbuser gtfs -c "insert into stops select * from stops_tmp;"
psql -U $dbuser gtfs -c "insert into routes select * from  routes_tmp;"
psql -U $dbuser gtfs -c "insert into trips select * from trips_tmp;"
psql -U $dbuser gtfs -c "insert into stop_times select * from stop_times_tmp;"
psql -U $dbuser gtfs -c "insert into calendar select * from calendar_tmp;"
psql -U $dbuser gtfs -c "insert into calendar_dates select * from calendar_dates_tmp;"
psql -U $dbuser gtfs -c "insert into transfers select * from transfers_tmp;"
psql -U $dbuser gtfs -c "insert into schedule select * from schedule_tmp;"

psql -U $dbuser gtfs -c "TRUNCATE TABLE shapes_tmp;"
psql -U $dbuser gtfs -c "TRUNCATE TABLE agency_tmp;"
psql -U $dbuser gtfs -c "TRUNCATE TABLE stops_tmp;"
psql -U $dbuser gtfs -c "TRUNCATE TABLE routes_tmp;"
psql -U $dbuser gtfs -c "TRUNCATE TABLE trips_tmp;"
psql -U $dbuser gtfs -c "TRUNCATE TABLE stop_times_tmp;"
psql -U $dbuser gtfs -c "TRUNCATE TABLE calendar_tmp;"
psql -U $dbuser gtfs -c "TRUNCATE TABLE calendar_dates_tmp;"
psql -U $dbuser gtfs -c "TRUNCATE TABLE transfers_tmp;"
psql -U $dbuser gtfs -c "TRUNCATE TABLE schedule_tmp;"
psql -U $dbuser gtfs -c "DROP Table s_tmp;"

if [[ "$1" ==  "clean" ]]; then
  psql -U $dbuser gtfs -c "CREATE INDEX shapes_index ON shapes USING GIST (geom);"
  psql -U $dbuser gtfs -c "CREATE INDEX shape_index ON shape USING GIST (geom);"
  psql -U $dbuser gtfs -c "CREATE INDEX stop_key_index ON stop_times USING btree (stop_key);"
fi

rm *.txt
rm schedule.tmp

exit 0

