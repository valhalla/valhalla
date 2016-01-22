# Running MM service in Docker

Before we start, make sure you have `docker`, `git` and `wget` installed.

1. Clone `map_matching_plus`:
   ```sh
   git clone --depth=1 https://github.com/mapillary/map_matching_plus.git
   ```

2. Build the Docker image `mapillary/mmp`:
   ```sh
   sudo docker build -t mapillary/mmp map_matching_plus/docker
   ```

3. We need a work directory, let's say `~/tiles` (must be absolute
   path), to place configuration files, OSM data and tiles:
   ```sh
   WORK_DIR=~/tiles
   mkdir -p "${WORK_DIR}"
   ```

4. Download the OSM of your city to the work directory:
   ```sh
   wget --directory-prefix "${WORK_DIR}"
        http://download.geofabrik.de/europe/germany/berlin-latest.osm.pbf
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
        mjnolor --conf conf/valhalla.json *.pbf
   ```

   This process takes a while, from a few minutes to a few hours,
   depends on the OSM size.

7. Copy the MM configuration file to the work directory so that the
   service can read it

   ```sh
   cp map_matching_plus/conf/mm.json "${WORK_DIR}"
   ```

8. Run the service:

   ```sh
   sudo docker run -it \
        --volume "${WORK_DIR}":/data \
        --publish 8001:8001 \
        mapillary/mmp \
        mmp_service mm.json
   ```

   Now the service is up. It is listening on `localhost:8001` for all
   coming coordinates in format of GeoJSON.

9. If you need a web interface to play with, clone our demos:
   ```sh
   git clone --depth=1 --branch=gh-pages https://github.com/mapillary/demos.git
   ```

   Open the web page with your favorite web browser:
   ```sh
   google-chrome-stable demos/routing/index.html
   ```

   Zoom to the city you just processed. Hopefully you will see there
   are a number of Mapillary sequences there ready for you to play
   with. Before clicking on any of these sequences, you need to click
   `Go Map Matching`, and then select `localhost` as current
   environment.
