## THIS IS A WORK IN PROGRESS!

### DockerHub

This repository is now built under the [valhalla dockerhub org](https://hub.docker.com/r/valhalla/valhalla/). The previous location under the Mapzen org will remain in place so there is continued access to old builds, but all new images will be built under valhalla/valhalla.

Builds are automated: pushing a tag in the form of {major}.{minor}.{patch} will trigger a build of both the ppa and source Dockerfiles, and result in images with the tags `valhalla/valhalla:run-{major}.{minor}.{patch}` or `valhalla/valhalla:build-{major}.{minor}.{release}` (dependencies for building from source). In addition to the tagged releases we also push the `-latest` variants which contain whatever is in `master`.

### Run valhalla (local development)

TODO: update the docker compose resources, these no longer work

`VALHALLA_DOCKER_DATAPATH=/some/path/to/data docker-compose -f docker-compose-{ppa|source}.yml up`

The routing engine will listen on and expose port 8002, and load any tile data found in `${VALHALLA_DOCKER_DATAPATH}`.

### To build/publish images manually
* `docker build -f Dockerfile-[build] --tag valhalla/valhalla:[build]-[version_tag] --no-cache --force-rm .`
* `docker push valhalla/valhalla:[build]-[version_tag]`
