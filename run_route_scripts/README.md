# How to run the the `valhalla_run_route` application
Run the valhalla_run_route application using the `../test_requests/demo_routes.txt` route request file and the `../../conf/valhalla.json` config file:
```
#Usage:
./run.sh <ROUTE_REQUEST_FILE> <CONFIG_FILE>
#Example#1:
./run.sh ../test_requests/demo_routes.txt ../../conf/valhalla.json
```
The results will be stored in the `<TIMESTAMP>_<ROUTE_REQUEST_FILE_BASENAME>` directory, e.g. `20160112_181443_demo_routes`

[Create and save diffs](results/README.md) in the `results` directory.

Run the valhalla_run_route application using all of the country specific route request files in the `requests/city_to_city` directory:
```
#Example:
./run_city_routes.sh
```

# How to create a path pbf that will be used as input for pinpoint tests
- Create a one line route request and save in the target pinpoint test directory - for example: `../test/pinpoints/turn_lanes/right_active_pinpoint.txt`
- Run the `create_path_pbf.sh` script that will read the specified route request and config and save a corresponding path pbf file - for example: `./create_path_pbf.sh ../test/pinpoints/turn_lanes/right_active_pinpoint.txt ../valhalla.json`
- Use the generated pbf file as the input path for a directions pinpoint test - example pbf file: `../test/pinpoints/turn_lanes/right_active_pinpoint.pbf`
