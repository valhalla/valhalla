# Valhalla Node.js Bindings

Node.js bindings for [Valhalla](https://github.com/valhalla/valhalla), an open-source routing engine.

## Installation

```bash
npm install @valhallajs/valhallajs
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
import { Actor, getConfig } from '@valhallajs/valhallajs';

async function main() {
  // Create an actor with config generated on previous step
  const actor = await Actor.fromConfigFile('config.json');

  // Calculate a route
  const result = await actor.route({
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

## Compatibility

### Node.js Versions
- Node.js v16.0.0 and all later versions
- Node.js v15.12.0+
- Node.js v14.17.0+
- Node.js v12.22.0+

### Platforms
- Linux (arm64, x64)
- macOS (arm64)
