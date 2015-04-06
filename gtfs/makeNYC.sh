wget http://web.mta.info/developers/data/nyct/subway/google_transit.zip
unzip google_transit.zip
./import.sh clean
rm google_transit.zip

wget https://transit-admin.511ny.org/feeds/JFK_Airtrain.zip
unzip JFK_Airtrain.zip
./import.sh
rm JFK_Airtrain.zip

wget http://web.mta.info/developers/data/lirr/google_transit.zip
unzip google_transit.zip
./import.sh 
rm google_transit.zip

wget http://web.mta.info/developers/data/mnr/google_transit.zip
unzip google_transit.zip
./import.sh 
rm google_transit.zip

