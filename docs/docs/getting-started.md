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
