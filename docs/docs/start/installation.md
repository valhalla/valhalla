# Installation

There are several ways to install Valhalla, depending on whether you want the HTTP service, the command line tools or the library bindings.

## From source

If you want to build Valhalla from source, follow the [build documentation](building.md).

## With Docker

[![Test & Publish Docker image](https://github.com/valhalla/valhalla/actions/workflows/docker-build.yml/badge.svg)](https://github.com/orgs/valhalla/packages?repo_name=valhalla)

To run Valhalla locally or your own server, we recommend using one of our [Docker images](https://github.com/orgs/valhalla/packages), see the [README](https://github.com/valhalla/valhalla/blob/master/docker/README.md).

## Via Python bindings

[![pyvalhalla version](https://img.shields.io/pypi/v/pyvalhalla?label=pyvalhalla)](https://pypi.org/project/pyvalhalla/)

We publish our (very) high-level Python bindings to PyPI with [`pyvalhalla`](https://pypi.org/project/pyvalhalla/).

The Python packages don't only contain the Python bindings, they also provide access to the C++ executables, e.g. in the form of `python -m valhalla valhalla_build_tiles -h`. For more details, see the [Python README](../bindings/python/index.md).

To install the native C++ executables one doesn't even need to have root permissions or even have Python installed. Simply download the desired wheel from [PyPI](https://pypi.org/project/pyvalhalla), extract it with e.g. `unzip` and run the included `valhalla/bin/<binary>` directly.

## Via NodeJS bindings

[![npm version](https://img.shields.io/npm/v/@valhallajs/valhallajs/latest?label=@valhallajs/valhallajs@latest)](https://www.npmjs.com/package/@valhallajs/valhallajs) [![npm version](https://img.shields.io/npm/v/@valhallajs/valhallajs/weekly?label=@valhallajs/valhallajs@weekly)](https://www.npmjs.com/package/@valhallajs/valhallajs)

We provide high-level NodeJS binding:

```bash
npm install @valhallajs/valhallajs
```

For more details, see the [NodeJS README](../bindings/nodejs.md).
