# Running Meili service in Docker

Before we start, make sure you have `docker`, `git` and `wget`
installed.

1. Clone `meili`:
   ```sh
   git clone --depth=1 https://github.com/valhalla/meili.git
   ```

2. Build the Docker image `valhalla/meili`:
   ```sh
   sudo docker build -t valhalla/meili meili/docker
   ```

3. We need a work directory, let's say `~/tiles` (must be absolute
   path), to place configuration files, OSM data and tiles:
   ```sh
   WORK_DIR=~/tiles
   mkdir -p "${WORK_DIR}"
   ```

4. Download the OSM of your city to the work directory:
   ```sh
   wget --directory-prefix "${WORK_DIR}" http://download.geofabrik.de/europe/germany/berlin-latest.osm.pbf
   ```

5. Clone Valhalla configuration which is needed by the tiles data
   creator `mjolnir` later:
   ```sh
   git clone --depth=1 https://github.com/valhalla/conf.git "${WORK_DIR}/conf"
   ```

6. Run `mjolnir` to create tiles:
   ```sh
   sudo docker run -it \
        --volume  "${WORK_DIR}":/data \
        --workdir /data \
        valhalla/meili \
        pbfgraphbuilder --conf conf/valhalla.json berlin-latest.osm.pbf
   ```

   This process takes a while, from a few minutes to a few hours,
   depending on the OSM size.

7. Copy the Meili configuration file to the work directory so that the
   service can read it

   ```sh
   cp meili/conf/mm.json "${WORK_DIR}"
   ```

8. Run the service:

   ```sh
   sudo docker run -it \
        --volume "${WORK_DIR}":/data \
        --publish 8001:8001 \
        valhalla/meili \
        valhalla_map_match_service /data/mm.json
   ```

   Now the service is up. It is listening on `localhost:8001` for all
   coming coordinates in format of GeoJSON. You may refer to the
   [service API](https://github.com/valhalla/meili/blob/master/docs/service_api.md)
   documentation for details.

9. If you need a web interface to play with, clone our demos:
   ```sh
   git clone --depth=1 --branch=gh-pages https://github.com/mapillary/demos.git
   ```

   Open the web page with your favorite web browser:
   ```sh
   google-chrome-stable demos/routing/index.html
   ```

   Zoom to the city you just processed. Hopefully you will see there
   are a number of Mapillary sequences ready to play with. Before you
   are able to click on these sequences, you need to switch to MM by
   clicking `Go Map Matching`, and then select `localhost` as current
   environment.
