## THIS IS A WORK IN PROGRESS!

### DockerHub

This repository is now built under the [valhalla dockerhub org](https://hub.docker.com/r/valhalla/docker/). The previous location under the Mapzen org will remain in place so there is continued access to old builds, but all new images will be built under valhalla/docker.

Builds are automated: pushing a tag in the form of {major}.{minor}.{release} will trigger a build of both the ppa and source Dockerfiles, and result in images with the tags `valhalla/docker:source-{major}.{minor}.{release}` or `valhalla/docker:ppa-{major}.{minor}.{release}`

### Run valhalla (local development)
`VALHALLA_DOCKER_DATAPATH=/some/path/to/data docker-compose -f docker-compose-{ppa|source}.yml up`

The routing engine will listen on and expose port 8002, and load any tile data found in `${VALHALLA_DOCKER_DATAPATH}`.

### To build/publish images manually
* `docker build -f Dockerfile-[build|build-x86] --tag valhalla/valhalla:[build|build-x86]-[version_tag] --no-cache --force-rm .`
* `docker push valhalla/valhalla:[build|build-x86]-[version_tag]`
