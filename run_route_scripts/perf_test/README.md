# How to run  

The following command will run `siege`, a multi-threaded http load testing and benchmarking utility, with 4 concurrent simulated users, 2550 repetitions per user, and the `url_de_benchmark_routes.txt` requests file.
```
siege -c 4 -b -r 2550 -f url_de_benchmark_routes.txt
```

