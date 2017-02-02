# How to run  
Run the valhalla_run_route application using the `requests/demo_routes.txt` route request file and the `../../conf/valhalla.json` config file:
```
#Usage:
./run.sh <ROUTE_REQUEST_FILE> <CONFIG_FILE>
#Example#1:
./run.sh requests/demo_routes.txt ../../conf/valhalla.json
```
The results will be stored in the `<TIMESTAMP>_<ROUTE_REQUEST_FILE_BASENAME>` directory, e.g. `20160112_181443_demo_routes`

[Create and save diffs](results/README.md) in the `results` directory.

Run the valhalla_run_route application using all of the country specific route request files in the `requests/city_to_city` directory:  
```
#Example:
./run_city_routes.sh
```
