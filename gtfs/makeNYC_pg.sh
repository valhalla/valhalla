wget http://web.mta.info/developers/data/nyct/subway/google_transit.zip
unzip google_transit.zip
./import.sh pg clean
rm google_transit.zip

#wget https://transit-admin.511ny.org/feeds/JFK_Airtrain.zip
wget -O JFK_Airtrain.zip http://transitfeeds.com/p/jfk-airtrain/433/20150310/download
unzip JFK_Airtrain.zip
./import.sh pg
rm JFK_Airtrain.zip

wget http://web.mta.info/developers/data/lirr/google_transit.zip
unzip google_transit.zip
./import.sh pg
rm google_transit.zip

wget http://web.mta.info/developers/data/mnr/google_transit.zip
unzip google_transit.zip
./import.sh pg
rm google_transit.zip

