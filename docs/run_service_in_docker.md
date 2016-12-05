# Running Meili service in Docker

Make sure you have `docker`, `git` and `wget` installed.

## Build Meili Docker Image

Clone meili and build it
```sh
git clone --depth=1 https://github.com/valhalla/meili.git
sudo docker build -t valhalla/meili meili/docker
```

## Run Meili Service

Run the service:

```sh
sudo docker run -it \
     --publish 8002:8002 \
     valhalla/meili \
     valhalla_map_match_service /etc/valhalla-server.json
```

Now the service is up and listening on `localhost:8002`. You may refer
to the
[service API](https://github.com/valhalla/meili/blob/master/docs/service_api.md)
for more information.


## Set up Demo

Open
http://valhalla.github.io/demos/map_matching/index-internal.html. Zoom
to San Francisco. Simulate a trace by clicking on the map. You will
see the trace is map matched immediately.
