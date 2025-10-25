# Valhalla Node.js Bindings

Node.js bindings for [Valhalla](https://github.com/valhalla/valhalla), an open-source routing engine.

## Installation

```bash
npm install @valhalla/valhalla
```

## Quick Start

### 1. Build Valhalla Tiles

First, download OSM data and build routing tiles using the Valhalla CLI tools:

```bash
# Download OSM data (e.g. Berlin)
wget https://download.geofabrik.de/europe/germany/berlin-latest.osm.pbf

# Build config file
npx valhalla build_config --mjolnir-tile-dir valhalla_tiles --mjolnir-tile-extract valhalla_tiles.tar > config.json

# Build tiles from OSM data
npx valhalla build_tiles -c config.json berlin-latest.osm.pbf
```

### 2. Use Valhalla for Routing

```javascript
import { Actor, getConfig } from '@valhalla/valhalla';

async function main() {
  // Create an actor with config generated on previous step
  const actor = await Actor.fromConfigFile('config.json');

  // Calculate a route
  const result = actor.route({
    locations: [
      { lat: 52.5200, lon: 13.4050 },
      { lat: 52.5300, lon: 13.4150 }
    ],
    costing: 'auto'
  });

  console.log(result);
}

main();
```

## API

### Actor

The `Actor` class provides access to Valhalla's routing services:

- `route(query)` - Get directions between locations
- `matrix(query)` - Compute time-distance matrix
- `isochrone(query)` - Generate isochrone polygons
- `optimizedRoute(query)` - Solve traveling salesman problem
- `traceRoute(query)` - Map-match GPS traces to routes
- `traceAttributes(query)` - Get attributes along a trace
- `locate(query)` - Find nearest roads to coordinates
- `height(query)` - Get elevation data
- `expansion(query)` - Get route expansion for visualization
- `status(query)` - Check service status

All methods accept either a query object or JSON string and return parsed results.

## CLI Tools

The package includes access to all Valhalla C++ command-line tools via `npx valhalla`:

```bash
# Build routing tiles
npx valhalla build_tiles -c config.json data.osm.pbf

# Run routing from command line
npx valhalla run_route -c config.json -j '{"locations":[...]}'

# Start Valhalla HTTP service
npx valhalla valhalla_service config.json

# See all available commands
npx valhalla --help
```

Note: Commands can be used with or without the `valhalla_` prefix (e.g., both `build_tiles` and `valhalla_build_tiles` work).

## License

MIT

