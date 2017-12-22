Elevation Lookup Service
---------
This document describes how to run an elevation lookup service using Valhalla. Documentation for this service can be found here: [elevation-api-docdocs](https://github.com/valhalla/valhalla-docs/tree/master/elevation/elevation-service.md). The easiest way to start an elevation service is to install Valhalla from PPA, download elevation data, and start the service. If you prefer to buld Valhalla from source please refer to the Valhalla [README](../README.md).

Get Valhalla from Personal Package Archive (PPA)
------------------------------------------------
If you are running Ubuntu (trusty or xenial) Valhalla can be installed quickly and easily via PPA. Try the following:

```bash
# grab all of the valhalla software from ppa
sudo add-apt-repository -y ppa:valhalla-core/valhalla
sudo apt-get update
sudo apt-get install -y valhalla-bin
```

Running
-------

The following bash should be enough to make some get some elevation data and start a server using it:

```bash
# grab data for the whole world (its about 1.6TB) or a smaller bounding box
valhalla_build_elevation -180 180 -90 90 ./elevation_tiles $(nproc)
#configure the server
valhalla_build_config --additional-data-elevation ./elevation_tiles > config.json
#start up the server with the config and number of threads
valhalla_elevation_service config.json 1
#curl it directly if you like:
curl http://localhost:8002/height --data '{"range":true,"shape":[{"lat":40.712431,"lon":-76.504916},{"lat":40.712275,"lon":-76.605259},{"lat":40.712122,"lon":-76.805694},{"lat":40.722431,"lon":-76.884916},{"lat":40.812275,"lon":-76.905259},{"lat":40.912122,"lon":-76.965694}]' | jq '.'

#HAVE FUN!
```

Contributing
------------

We welcome contributions to valhalla. If you would like to report an issue, or even better fix an existing one, please use the [valhalla issue tracker](https://github.com/valhalla/valhalla/issues) on GitHub.

If you would like to make an improvement to the code, please be aware that all valhalla projects are written mostly in C++11, in the K&R (1TBS variant) with two spaces as indentation. We welcome contributions as pull requests to the [repository](https://github.com/valhalla/valhalla) and highly recommend that your pull request include a test to validate the addition/change of functionality.
