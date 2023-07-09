## Valhalla Incidents

### General

This project aims at turning TomTom Incidents data into a Valhalla-friendly format so that the live traffic infrastructure can update affected edges to be closed (or open previously closed ones).

To that end, we developed a new service `valhalla_incidents_service` which can be run as a HTTP API, optionally within a docker container.

The flow is:
- POST request to `valhalla_incidents_service`'s endpoints (s. below)
- with multiple threads, decode the OpenLR strings and match their location reference points to a sequence of graph edges
- update the `traffic.tar` for all affected edges; depending on the endpoint it'll either close road segments, open closed ones or reset the entire `traffic.tar`

**NOTE**, this HTTP API is **not** meant to run on **more than 1 thread**. Internally, each job will use multithreading to decode and match the OpenLR strings.

### Configuration/Requirements

The most important configuration requirements are:
- `mjolnir.tile_extract`: needs the path of the Valhalla graph tar file. This is necessary because we need to route & map-match to correlate the OpenLR "geometry" to the Valhalla graph edges
- `mjolnir.traffic_extract`: this is the path to the `traffic.tar` file which will be written to by `valhalla_incidents_service` is shared with the routing Valhalla instance
- `mjolnir.concurrency`: the amount of threads with which we'll process a single POST request; don't use more than machine specs

### Endpoints

- `/update`: **updates** traffic entries with `0` speed for all matched edges
- `/delete`: **removes** traffic entries for all matched edges, so they won't be considered "closed" anymore
- `/reset`: **updates** the passed traffic entries (if any) and **removes** all other entries, i.e. a combination of `/update` and `/delete`

These endpoints return GeoJSON and are very useful for debugging:

- `/geojson/matches`: Returns the exact geometry of the edges which will be updated
- `/geojson/openlr`: Returns the interpolated line between a OpenLR's linear reference points

**NOTE**, we included `./scripts/incident_xml_parser.py` to produce a payload from XML files (requires `lxml` Python package to be installed). `./scripts/incident_xml_parser.py -h` for options.

#### POST request

All endpoints take the same parameters: an array of base64-encoded OpenLR strings, e.g. 

```json
[
	"CwpRoyOdoCORAf/V/7EjAQ==",
	"Cww8UiNMMyONDQC1/WMjAg=="
]
```

### Build & run

To build the Docker image:

```
# clone and fetch all submodules
git clone --recurse-submodules https://gitlab.solvertech.cz/gis-ops/incidents
docker build -t solvertech/valhalla-incidents:latest .
```

To run:

Spin up a container and map the local directory where the graph tiles for entire Czech Republic are located (it's important that you expose the graph directory, not the tar file)

```
# after this, you should have ./valhalla_incidents/0, ./valhalla_incidents/1, ./valhalla_incidents/2
mkdir valhalla_incidents && cp graph/ valhalla_incidents

# run the container with volume & open port
docker run --name valhalla_incidents -dt -v $PWD/valhalla_incidents:/data -p 8003:8002 solvertech/valhalla-incidents:latest

# exec into it
docker exec -it valhalla_incidents bash
```

Once inside the container:

```
# First build the config to give the right paths
valhalla_build_config \
  --mjolnir-concurrency 16 \
  --mjolnir-tile-dir /data \
  --mjolnir-tile-extract /data/czech.tar \
  --mjolnir-traffic-extract /data/czech_traffic.tar > /data/valhalla.json

# Now we can build the graph.tar & the accompanying traffic.tar
valhalla_build_extract --config /data/valhalla.json --with-traffic

# Run the service
valhalla_incidents_service --config /data/valhalla.json
```

Then you should be able to query with e.g.

```
curl --request POST \
  --url http://localhost:8003/geojson/matches \
  --header 'Content-Type: application/json' \
  --data '[
	"Cwo8wiOfqxJeDv5pAdISZxMR",
	"Cww8UiNMMyONDQC1/WMjAg=="
]'
```
