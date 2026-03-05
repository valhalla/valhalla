# Getting started

## Prerequisites

Basic familiarity with:

- Command line
- Docker
- Package management

## Requirements

- [Docker](https://www.docker.com/)

## Install

> See _Installation instructions_ for detailed information.

> TODO: Use Python library instead?

The simplest way to install Valhalla and the tools is with Docker. We'll use the [official _base_ image](https://github.com/valhalla/valhalla/pkgs/container/valhalla):

```bash
docker pull ghcr.io/valhalla/valhalla:latest
```

Now we can start the container:

```bash
docker run \
    --name valhalla \
    --publish 127.0.0.1:8002:8002 \
    --mount type=volume,src=valhalla-data,dst=/data \
    --workdir /data \
    --interactive \
    --tty \
    --rm \
    ghcr.io/valhalla/valhalla:latest
```

In addition, this command:

- Publishes a port so we could communicate from outside the container.
- Creates a volume named `valhalla-data` and mounts it at `/data` directory inside the container to persist generated files.

Let's make sure that everything is okay - the following command should return the version:

```console
$ valhalla_build_tiles --version
3.6.3-95fb6ddd6
```

> TODO More info about the tools?

> All Valhalla tools support `--help` option to show usage.

From now on, we'll run all commands _inside_ the container, in `/data` directory with a mounted volume:

```bash
cd /data
```

Finally, update the list of available software packages and install some helpful programs:

```bash
apt update
apt install jq less tree wget
```

## Configuration file

> Move into [Data](#prepare-the-data) section?

> See [Configuration](guides/configuration.md) for detailed information.

All Valhalla tools use a common JSON configuration file. It contains settings for data generation, location search, service process and so on.

> TODO: Set correct paths for admin and tz databases

Let's create it:

```bash
valhalla_build_config \
    --mjolnir-tile-dir "${PWD}"/tiles/ \
    --mjolnir-tile-extract "${PWD}"/tiles.tar \
    > config.json
```

> TODO What do the options mean?

The command generates a configuration with reasonable defaults. Take a look:

```bash
less config.json
```

## Prepare the data

> See [Data guide](guides/data.md) for detailed information.

Valhalla needs some data in order to work.

The fundamental / core / main / minimal component is a set of **routing tiles** (TODO link) - a directory of files in a specific format with information about the roads, restrictions and so on.

The core _data source_ for this is [OpenStreetMap](https://www.openstreetmap.org/) (OSM): Valhalla takes OSM map data in [PBF format](https://wiki.openstreetmap.org/wiki/PBF_Format) and uses it to create a set of tiles.

> IDEA: Talk a little more about _data_: content and format (PBF)? What is an extract?

> IDEA: Link to learn more about OSM data format: [Beginners' Guide](https://wiki.openstreetmap.org/wiki/Beginners%27_guide), see [Downloading data](https://wiki.openstreetmap.org/wiki/Downloading_data) for more options.

> Tiles are a representation of a [graph](<https://en.wikipedia.org/wiki/Graph_(abstract_data_type)>) data structure.

Here's what we need to do to get a minimal working version of the system:

1. Download [OSM data extract](https://download.geofabrik.de/technical.html) in `.osm.pbf` file format
1. Build _admin_ database\*
   > Recommendation is to build this from the _whole planet_, which is a large file and may take some time.
1. Build _time zones_ database
1. Build the tiles

### Download OpenStreetMap data extract

> IDEA: Maybe use just **one** extract? Note about an option to work with multiple extracts and the limitations. Specific version of OSM data extracts?

We can use [Geofabrik's free server](https://download.geofabrik.de/) to download [data extracts](<(https://download.geofabrik.de/technical.html)>) to `osm/` directory:

```bash
wget \
    --directory-prefix osm/ \
    https://download.geofabrik.de/europe/andorra-latest.osm.pbf \
    https://download.geofabrik.de/europe/liechtenstein-latest.osm.pbf
```

!!! warning

    Valhalla can work with _multiple_ data extracts, but this is discouraged - some problems. See linked issue (TODO).

!!! info

    Suggestion - use whole planet. Give some information about the size and time it takes.

### Build admin database

> TODO: What is this? Why do we need it?

```bash
valhalla_build_admins -c config.json switzerland-latest.osm.pbf
```

### Build time zones database

> TODO: What is this? Why do we need it?

```bash
valhalla_build_timezones > timezones.sqlite
```

### Build routing tiles

Finally, we are ready to build the tiles:

```bash
valhalla_build_tiles --config config.json osm/andorra-latest.osm.pbf
```

We'll see a log with a bunch of messages. It's okay to ignore the warnings.

When finished, we can see generated data in `tiles/` directory (the one from configuration file). At the time of writing it looks like this:

```console
$ tree tiles/
tiles/
|-- 0
|   `-- 003
|       `-- 015.gph
|-- 1
|   `-- 047
|       `-- 701.gph
`-- 2
    `-- 000
        |-- 762
        |   |-- 485.gph
        |   `-- 486.gph
        `-- 763
            |-- 925.gph
            |-- 926.gph
            `-- 927.gph

9 directories, 7 files
```

> IDEA: A button to show the result (like in [`jq` tutorial](https://jqlang.org/tutorial/)?)

TODO: A little info about what this directory & files mean. Structure of `tiles/` directory: on top, directories represent _hierarchy levels_. For example, in directory `tiles/0/` we have the tiles with hierarchy level 0. The rest of the path encodes a _tile index_. For example, a tile at path `tiles/1/047/701.gph` has:

- Level: 1 from `1/`
- Tile index: 47701 from `047/701.gph`

> Why are we doing this?

### Inspect the edges

We can export the edges into a CSV file:

```bash
valhalla_export_edges -c config.json > edges.csv
```

And take a look:

```console
$ head -n 1 edges.csv
uebapAcyi{AcEcDm\mUmJiG_FaDmNuM}rBc}B}MiPeEoEqC{CCarrer Prada Motxilla
```

> TODO: Other options for tile inspection? Website to visualize polylines?

## Start the service

> See [Operations Guide](guides/operations.md) for detailed information.

> TODO Info about using it as a library?

There are multiple ways to run Valhalla. Here's one of them:

```bash
valhalla_service config.json 1
```

Valhalla service is an HTTP server process - it accepts requests and returns responses. By default, it is available on port `8002`, but we could change that in the configuration file.

> Remember that we published a port when we ran a Docker container? Now we can send requests to Valhalla using <http://localhost:8002> address from outside the container.

Interface / public API - RESTful - set of paths (`/status`, `/route`, etc) for different operations, either GET or POST requests. For both, we pass the content either via query parameter or as a body | JSON content. Formats depend on the specific operation.

> TODO: Some details on how the service works? Accept the request, correlate locations to the routing graph to find the correct nodes, run the algorithm, create response in a nice format.

> TODO: Technical details. Graph algorithms, sequential with one core, multiple cores = multiple requests at the same time, loads tiles from disk, caches them in memory. Depending on the data layout, either each process has its own cache, or they share data via memory-mapped tar archive. Suggestion: lots of memory, fast disk, many cores.

> TODO: Can I use this in production? What's performance like?

Let's make a request to check the status:

```console
$ curl http://localhost:8002/status | jq '.'
{
  "version": "3.6.3-95fb6ddd6",
  "tileset_last_modified": 1772753449,
  "available_actions": [
    "tile",
    "status",
    "centroid",
    "expansion",
    "transit_available",
    "trace_attributes",
    "trace_route",
    "isochrone",
    "optimized_route",
    "sources_to_targets",
    "height",
    "route",
    "locate"
  ]
}
```

This command will do this and that.

> We use `curl` as it is installed inside the container, but you could use any other CLI tool (httpie, etc) or API client (Bruno, etc) to talk to the service.
