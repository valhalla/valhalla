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
   path), to serve as a shared folder between host and the docker
   container. We will place OSM data, configuration files, and
   generated tiles there:

   ```sh
   WORK_DIR=~/tiles
   mkdir -p "${WORK_DIR}"
   ```

4. Search your city and download the OSM data at
   [Mapzen metro extracts](https://mapzen.com/data/metro-extracts/). Here
   we take Berlin as example:
    ```sh
   wget --directory-prefix "${WORK_DIR}" https://s3.amazonaws.com/metro-extracts.mapzen.com/berlin_germany.osm.pbf
   ```

5. Clone the Valhalla conf to the work directory:
   ```sh
   git clone --depth=1 https://github.com/valhalla/conf.git "${WORK_DIR}/conf"
   ```

6. Run `mjolnir` to create tiles:
   ```sh
   sudo docker run -it \
        --volume  "${WORK_DIR}":/data \
        --workdir /data \
        valhalla/meili \
        valhalla_build_tiles --conf conf/valhalla.json berlin_germany.osm.pbf
   ```

   This process takes time from a few minutes to hours, depending on
   the OSM size. It creates tiles under `${WORK_DIR}/valhalla`.

7. Run the Meili service:

   ```sh
   sudo docker run -it \
        --volume "${WORK_DIR}":/data \
        --workdir /data \
        --publish 8002:8002 \
        valhalla/meili \
        valhalla_map_match_service conf/valhalla.json
   ```

   Now the service is up and listening on `localhost:8002` for all
   coming coordinates in GeoJSON format. You may refer to the
   [service API](https://github.com/valhalla/meili/blob/master/docs/service_api.md)
   documentation for details.

8. If you need a web interface to play with, clone our demos:
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
