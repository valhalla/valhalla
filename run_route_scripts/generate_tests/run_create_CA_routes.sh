#!/bin/bash

### This script is CA-specific but it can be used
### as a template in generating other location files



rm loc_seed_*.txt
rm CA_*.txt

#Manually collect city points in CA and generate a random number of points within a radius
# lat | lon | # of random generated pts | radius in meters
./create_random_points_within_radius.py 34.052025 -118.244225 30 10000 > loc_seed_1.txt  #LA 10km 
./create_random_points_within_radius.py 34.107766 -117.289439 20 10000 > loc_seed_2.txt  #SanBernardino 10km
./create_random_points_within_radius.py 33.837541 -117.913470 20 10000 > loc_seed_3.txt  #Anaheim 10km
./create_random_points_within_radius.py 35.372965 -119.019924 10 10000 > loc_seed_4.txt  #Bakersfield 10km
./create_random_points_within_radius.py 36.735042 -119.785714 10 10000 > loc_seed_5.txt  #Fresno 10km
#./create_random_points_within_radius.py 38.587161 -121.488578 30 10000 > loc_seed_6.txt  #Sacramento 10km


#merge into 1 CA location file
for fname in loc_seed_*.txt; do
  #sed -e "s/\(\-\?[0-9]\+\.[0-9]\+, \-\?[0-9]\+\.[0-9]\+\)/\n\1\n/g" $fname | grep -E '[0-9]$' | sed -e "s/^/[/g" -e "s/$/],/g" >> CA_locations.txt  #if JSON is needed
  sed -e "s/\(\-\?[0-9]\+\.[0-9]\+, \-\?[0-9]\+\.[0-9]\+\)/\n\1\n/g" $fname | grep -E '[0-9]$' | sed -e 's/, \+/,/g'>> CA_locations.txt
done	

#Choose location descriptor, costing, time type(depart or arriveby) and datetime
#Generates the CA routes from above generated locations
#Create routes with datetime or without

mkdir -p ../requests/predicted_traffic/
./create_test_request_routes.py CA auto > CA_routes.txt
head -n-1 CA_routes.txt  > ../requests/predicted_traffic/CA_routes_no_datetime_remote.txt   #removes the last line so that we dont have a the final location routing to itself
#./create_test_request_routes.py CA auto 1 "2018-09-03T08:30"> CA_routes.txt
#head -n-1 CA_routes.txt  > ../requests/predicted_traffic/CA_routes_datetime_remote.txt     #removes the last line so that we dont have a the final location routing to itself
