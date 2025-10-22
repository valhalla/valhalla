[![Test & Publish Docker image](https://github.com/valhalla/valhalla/actions/workflows/docker-build.yml/badge.svg)](https://github.com/valhalla/valhalla/actions/workflows/docker-build.yml)

# Valhalla Docker images

We provide Docker images for various architectures of two images:
- [**base image**](https://github.com/valhalla/valhalla/pkgs/container/valhalla): reflects the "plain" image, which contains the whole library & all executables, but no `docker-entrypoint` script
- [**scripted image**](https://github.com/valhalla/valhalla/pkgs/container/valhalla-scripted): is based on the "base image" but lets the user configure the whole tile build parameters per environment variables (with sensible defaults) auto-magically, also removes some debugging executables

The "base image" is more catered towards individuals knowing how to configure a Valhalla tile build correctly and wanting to implement non-supported (in the "scripted image") use cases.

The following guide is written for the "scripted image".

## Quick start

```bash
# download a file to custom_files and start valhalla
mkdir custom_files
wget -O custom_files/andorra-latest.osm.pbf https://download.geofabrik.de/europe/andorra-latest.osm.pbf
docker run -dt --name valhalla -p 8002:8002 -v $PWD/custom_files:/custom_files ghcr.io/valhalla/valhalla-scripted:latest

# or let the container download the file for you
docker run -dt --name valhalla -p 8002:8002 -v $PWD/custom_files:/custom_files -e tile_urls=https://download.geofabrik.de/europe/andorra-latest.osm.pbf ghcr.io/valhalla/valhalla-scripted:latest
```

Once built, you can easily change Valhalla's configuration: the underlying OSM graphs are built from, accompanying data (like admin or timezone DBs, elevation tiles etc) or even other pre-built graph tiles. Upon `docker restart <container>` those changes are taken into account via **hashed files**, and, if necessary, new graph tiles will be built automatically.

## Features

- Easily switch graphs by mapping different volumes to containers.
- Stores all relevant data (tiles, config, admin & timezone DBs, elevation) in the mapped volume.
- ~~Load and build from **multiple URLs** pointing to valid pbf files.~~ ([not recommended](https://github.com/valhalla/valhalla/issues/3925))
- Load local data through volume mapping.
- **Supports auto rebuild** on OSM file changes through md5

## Build the image

If you want to build the image yourself, be aware that you might need to adapt the base image in the `Dockerfile` to reflect the version of Valhalla you'd like to build. You can find the tags of the `ghcr.io/valhalla/valhalla` images here: https://github.com/valhalla/valhalla/pkgs/container/valhalla.

Then it's a simple

```shell script
# optionally build the base image yourself before or download it by default
# docker build -t ghcr.io/valhalla/valhalla:latest .
docker build  -f docker/Dockerfile-scripted -t ghcr.io/valhalla/valhalla-scripted:latest .
```

The `--build-arg`s are:
 - `VALHALLA_BUILDER_IMAGE`: specify the base image, default `ghcr.io/valhalla/valhalla:latest`

## Environment variables

Containers of this image have the following custom environment variables being passed:

| Environment Variable | Default | Description |
|--|--|--|
| `tile_urls` |  | Supports single or multiple (space-separated) URL(s), e.g. https://download.geofabrik.de/europe/andorra-latest.osm.pbf |
| `use_tiles_ignore_pbf` | `True` | `True` uses a local tile.tar file if available and skips building. |
| `force_rebuild` | `False` | `True` forces a rebuild of the routing tiles and sets `build_tar` to `Force`. |
| `build_elevation` | `False` | `True` downloads elevation tiles which are covering the routing graph. <br> `Force` will do the same, but first delete any existing elevation tiles. |
| `build_admins` | `True` | `True` builds the admin db needed for things like driving side and border-crossing penalties. <br> `Force` will do the same, but first delete the existing db. |
| `build_time_zones`| `True` | `True` builds the timezone db which is needed for time-dependent routing. <br> `Force` will do the same, but first delete the existing db. |
| `build_transit` | `False` | `True` will attempt to build transit tiles if none exist yet.  <br>`Force` will remove existing transit **and** routing tiles. |
| `build_tar` | `True` | `True` creates a tarball of the tiles including an index which allows for faster graph loading after reboots. <br> `Force` will do the same, but first delete the existing tarball. |
| `server_threads` | value of `nproc` | How many threads `valhalla_build_tiles` will use and `valhalla_service` will run with. <br> If valhalla gets killed when building tiles, lower this number.  |
| `path_extension` | `''` | This path will be appended to the container-internal `/custom_files` (and by extension to the docker volume mapped to that path) and will be the directory where all files will be created. <br> Can be very useful in certain deployment scenarios. No leading/trailing path separator allowed. |
| `serve_tiles` | `True` | `True` starts the valhalla service. |
| `tileset_name` | `valhalla_tiles` | The name of the resulting graph on disk.<br>Very useful in case you want to build multiple datasets in the same directory. |
| `traffic_name` | `""` | The name of the `traffic.tar`.<br>Setting this to be empty (i.e. `""`) will cause no traffic archive to be built.<br>Again, useful for serving multiple traffic archives from the same directory. |
| `update_existing_config` | `True` | `True` updates missing keys in existing `valhalla.json`.<br>Useful for updating stale config files to include newly introduced config parameters. |
| `use_default_speeds_config` | `True` | `True` loads a JSON file used to enhance default speeds (or falls back to an existing `custom_files/default_speeds.json`) and sets the respective config entry. Read more [here](https://github.com/OpenStreetMapSpeeds/schema). |
| `default_speeds_config_url` | [this url](https://raw.githubusercontent.com/OpenStreetMapSpeeds/schema/master/default_speeds.json) | Remote location of the `default_speeds_config` JSON. |

## Container recipes

For the following instructions to work, you'll need to have the image locally available already, either from [Github Docker registry](https://github.com/valhalla/valhalla/pkgs/container/valhalla-scripted) or from [local](#build-the-image).

Start a background container from that image:

```bash
docker run -dt -v $PWD/custom_files:/custom_files -p 8002:8002 --name valhalla ghcr.io/valhalla/valhalla-scripted:latest
```

The important part here is, that you map a volume from your host machine to the container's **`/custom_files`**. The container will dump all relevant Valhalla files to that directory.

At this point Valhalla is running, but there is no graph tiles yet. Follow the steps below to customize your Valhalla instance.

> [!NOTE]
> Alternatively you could create `custom_files` on your host before starting the container with all necessary files you want to be respected, e.g. the OSM PBF files.

#### Build Valhalla with transit

Valhalla supports reading raw GTFS feeds to build transit into its graph, see the [docs](https://valhalla.github.io/valhalla/api/turn-by-turn/api-reference/#sample-json-payloads-for-multimodal-requests-with-transit) for more details.

Put the unzipped GTFS feeds as subfolders in the main gtfs folder, e.g. `gtfs_feeds/berlin/`, otherwise the files will not be found.

To enable `multimodal` routing, you'll need to map the directory which contains all the GTFS feeds to the container's `/gtfs_feeds` directory, e.g.

```
docker run -dt -v gtfs_feeds:/gtfs_feeds -v $PWD/custom_files:/custom_files -p 8002:8002 --name valhalla valhalla/valhalla-scripted:latest
```

#### Build Valhalla with arbitrary OSM data

Just dump OSM PBF file(s) to your mapped `custom_files` directory, restart the container and Valhalla will start building the graphs:

```bash
cd custom_files
# Download Andorra & Faroe Islands
wget http://download.geofabrik.de/europe/faroe-islands-latest.osm.pbf http://download.geofabrik.de/europe/andorra-latest.osm.pbf
docker restart valhalla
```

If you change the PBF files by either adding new ones or deleting any, Valhalla will build new tiles on the next restart unless told not to (e.g. setting `use_tiles_ignore_pbf=True`).

#### Build Valhalla Graph with custom elevation tiles

Elevation tiles need to be in HGT (file format of the SRTM dataset) format and need to be named like `NXXEYYY.hgt`. [More info about format](https://github.com/tilezen/joerd/blob/master/docs/formats.md#skadi).

You need to store elevation tiles in the `<base_path>/elevation_data` directory (by default `custom_files/elevation_data/`). Tiles need to be grouped in folders by latitude , for example:

```
custom_files/elevation_data/
  N53/
    N53E016.hgt
    N53E017.hgt
  N54/
    N54E016.hgt
```

If you had an existing graph before you acquired new elevation data, you'll need to rebuild the graph for the new data to become available, e.g. by starting a new container with `force_rebuild=True`. If you want to use new elevation data for the `/height` service, you just need to restart the container.

#### Customize Valhalla configuration

If you need to customize Valhalla's configuration to e.g. increase the allowed maximum distance for the `/route` service, just edit `custom_files/valhalla.json` and restart the container. It won't rebuild the tiles in this case, unless you tell it to do so via environment variables.

#### Run Valhalla with pre-built tiles

In the case where you have a pre-built `valhalla_tiles.tar` package from another Valhalla instance, you can also dump that to `/custom_files/` and they're loaded upon container restart if you set the following environment variables: `use_tiles_ignore_pbf=True`, `force_rebuild=False`. Also, don't forget to set the md5 sum for your `valhalla_tiles.tar` in `.file_hashes.txt`.

## Tests

If you want to verify that the image is working correctly, there's a small test script in `./tests`.

```shell script
./tests/test.sh
```

> [!TIP]
> It might require `sudo`, since it touches a few things generated by the container's `valhalla` user
