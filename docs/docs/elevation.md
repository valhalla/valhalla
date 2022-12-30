# Elevation Lookup Service

This document describes how to run an elevation lookup service using Valhalla. Documentation for querying this service can be found here: [elevation-api-docdocs](api/elevation/api-reference.md).

## Running

The easiest way to start an elevation service is to run valhalla via Docker, download elevation data, and start the service. Or you can build Valhalla from source. See the main [README](https://github.com/valhalla/valhalla/blob/master/README.md) for installation instructions.

The following bash should be enough to make some get some elevation data and start a server using it:

```bash
# Be sure that the parallel and curl dependencies are installed
sudo apt-get install parallel
sudo apt-get install curl
# grab data for the whole world (it's about 1.6TB) or a smaller bounding box
valhalla_build_elevation -180 180 -90 90 ./elevation_tiles $(nproc)
#configure the server
valhalla_build_config --additional-data-elevation ./elevation_tiles > config.json
#start up the server with the config and number of threads
valhalla_service config.json 1
#curl it directly if you like:
curl http://localhost:8002/height --data '{"range":true,"shape":[{"lat":40.712431,"lon":-76.504916},{"lat":40.712275,"lon":-76.605259},{"lat":40.712122,"lon":-76.805694},{"lat":40.722431,"lon":-76.884916},{"lat":40.812275,"lon":-76.905259},{"lat":40.912122,"lon":-76.965694}]' | jq '.'

#HAVE FUN!
```

## See Also

- [API docs](api/elevation/api-reference.md) for `/height` endpoint
- [Skadi module](skadi.md) for working with digital elevation model (DEM) data
- [Terrain tiles](https://registry.opendata.aws/terrain-tiles/) for download from Amazon Web Services Public Datasets program
