# Diffs  
Diff two route results directories and save diff results in a new directory. The diff directory shall be named as follows: `<PREVIOUS_ROUTE_RESULTS_DIRECTORY>_<CURRENT_ROUTE_RESULTS_DIRECTORY>_diff`  
USAGE: `./diff_results.sh <PREVIOUS_ROUTE_RESULTS_DIRECTORY> <CURRENT_ROUTE_RESULTS_DIRECTORY>`  
EXAMPLE:  `./diff_results.sh 20160112_181443_demo_routes 20160113_152056_demo_routes`  
The diff results are stred in the `TBD` directory.  
  
Indorder to list and inspect the narrative differences - switch to the diff directory:  
`cd <DIFF_DIRECTORY>`  
  
Output the count of narrative differences:  
`../cnd`  
  
List the narrative differences:  
`../lnd`  
  
View the narrative differences:  
`../vnd`  
  
Output all of the narrative differences:  
`../catnd`  
  
# Stats  
To sum all of the route results stats and store in the `<ROUTE_RESULTS_DIRECTORY>/total_statistics.csv` file:  
USAGE: `./total_run_stats.sh <ROUTE_RESULTS_DIRECTORY>`  
EXAMPLE: `./total_run_stats.sh 20160113_152056_demo_routes`  
  
To sum all of the city to city route results stats and store in the `<TIMESTAMP>_total_statistics.csv` file:  
USAGE: `./total_multi_run_stats.sh <DIRECTORY_LIST_FILE>`  
EXAMPLE: `./total_multi_run_stats.sh OUTDIRS.txt`  
  
