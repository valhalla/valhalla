wget http://onestop-feed-cache.transit.land.s3.amazonaws.com/f-dr5r-nyctsubway.artifact.zip
unzip f-dr5r-nyctsubway.artifact.zip
./import.sh sqlite clean
rm f-dr5r-nyctsubway.artifact.zip

wget http://onestop-feed-cache.transit.land/f-9q9-bayarearapidtransit.artifact.zip
unzip f-9q9-bayarearapidtransit.artifact.zip
./import.sh sqlite 
rm f-9q9-bayarearapidtransit.artifact.zip

