# Create and save diffs  
Diff two route results directories and save diff results in a new directory. The diff directory shall be named as follows: `<PREVIOUS_ROUTE_RESULTS_DIRECTORY>_<CURRENT_ROUTE_RESULTS_DIRECTORY>_diff`
```
#Usage:
./diff_results.sh <PREVIOUS_ROUTE_RESULTS_DIRECTORY> <CURRENT_ROUTE_RESULTS_DIRECTORY>
#Example:
./diff_results.sh 20160112_181443_demo_routes 20160113_152056_demo_routes
```
The diff results are stored in the `20160112_181443_demo_routes_20160113_152056_demo_routes_diff` directory.  
  
Indorder to list and inspect the narrative differences - switch to the diff directory:
```
cd <DIFF_DIRECTORY>
```

Output the count of narrative differences:  
```
../cnd
```

List the narrative differences:  
```
../lnd
```
  
View the narrative differences:  
```
../vnd
```

Output all of the narrative differences:  
```
../catnd
```

# Create and save stats  
To sum all of the route results stats and store in the `<ROUTE_RESULTS_DIRECTORY>/total_statistics.csv` file:  
```
#Usage:
./total_run_stats.sh <ROUTE_RESULTS_DIRECTORY>
#Example:
./total_run_stats.sh 20160113_152056_demo_routes
```

To sum all of the city to city route results stats and store in the `<TIMESTAMP>_total_statistics.csv` file:  
```
#Usage:
./total_multi_run_stats.sh <DIRECTORY_LIST_FILE>
#Example:
./total_multi_run_stats.sh OUTDIRS.txt
```
