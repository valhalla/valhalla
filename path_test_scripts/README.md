Run the path_test application using the demo_routes.txt route request file:  
USAGE: `./run.sh <ROUTE_REQUEST_FILE> [<PREVIOUS_ROUTE_RESULTS_DIRECTORY>]`  
EXAMPLE#1: `./run.sh demo_routes.txt`  
The results will be stored in the `<TIMESTAMP>_<ROUTE_REQUEST_FILE_BASENAME>` directory, e.g. `20160112_181443_demo_routes`  
  
EXAMPLE#2: `./run.sh demo_routes.txt 20160112_181443_demo_routes`  
The results will be stored in the `<TIMESTAMP>_<ROUTE_REQUEST_FILE_BASENAME>` directory, e.g. `20160113_152056_demo_routes`  
The <PREVIOUS_ROUTE_RESULTS_DIRECTORY> and the current route results will be compared and the diff results will be stored in the `<PREVIOUS_ROUTE_RESULTS_DIRECTORY>_<CURRENT_ROUTE_RESULTS_DIRECTORY>_diff` directory, e.g. `20160112_181443_demo_routes_20160113_152056_demo_routes_diff`
  
Run the path_test application using all of the country specific route request files in the `requests/city_to_city` directory:  
USAGE: `./run_city_routes.sh.sh`  
 
