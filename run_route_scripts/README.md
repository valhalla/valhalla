# How to run a route request file through the `valhalla_run_route` application
Run the valhalla_run_route application using the `../test_requests/demo_routes.txt` route request file and the `../../conf/valhalla.json` config file:
```
##Usage:
./run.sh <ROUTE_REQUEST_FILE> <CONFIG_FILE>
##Example#1:
./run.sh ../test_requests/demo_routes.txt ../../conf/valhalla.json
```
The results will be stored in the `<TIMESTAMP>_<ROUTE_REQUEST_FILE_BASENAME>` directory, e.g. `20160112_181443_demo_routes`

[Create and save diffs](results/README.md) in the `results` directory.

# How to run multiple route request files through the `valhalla_run_route` application
You can start with the `request_files.txt.TEMPLATE` file or create you own `request_files.txt`
```
cp request_files.txt.TEMPLATE request_files.txt

##Usage:
./multi_run.sh <FILE_OF_REQUESTS> <CONFIG_FILE>
##Example#1:
./multi_run.sh
##Example#2:
./multi_run.sh my_own_files.txt
##Example#3:
./multi_run.sh request_files.txt ../../conf/valhalla.json
```

# How to create a path pbf that will be used as input for pinpoint tests
- Create a one line route request and save in the target pinpoint test directory - for example: `../test/pinpoints/turn_lanes/right_active_pinpoint.txt`
- Run the `create_path_pbf.sh` script that will read the specified route request and config and save a corresponding path pbf file - for example: `./create_path_pbf.sh ../test/pinpoints/turn_lanes/right_active_pinpoint.txt ../valhalla.json`
- Use the generated pbf file as the input path for a directions pinpoint test - example pbf file: `../test/pinpoints/turn_lanes/right_active_pinpoint.pbf`
