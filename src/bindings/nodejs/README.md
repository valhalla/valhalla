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

### 3. Working with `GraphId` (no built graph needed)

The `GraphId` class represents Valhalla's internal graph identifiers, which encode a hierarchy level, tile ID, and element ID.

```javascript
import { GraphId } from '@valhallajs/valhallajs';

// Construct from components: (tileid, level, id)
const gid = new GraphId(100, 2, 5);
console.log(gid.level());   // 2
console.log(gid.tileid());  // 100
console.log(gid.id());      // 5
console.log(gid.is_valid()); // true

// Construct from string "level/tileid/id"
const gid2 = new GraphId('2/100/5');
console.log(gid.equals(gid2)); // true

// String and JSON representations
console.log(gid.toString());    // "2/100/5"
console.log(JSON.stringify(gid)); // {"level":2,"tileid":100,"id":5,"value":...}

// Get tile base (same tile, id zeroed out)
const base = gid.tile_base();
console.log(base.toString()); // "2/100/0"

// Advance the id by an offset (returns a new GraphId)
const next = gid.add(3);
console.log(next.toString()); // "2/100/8"

// Roundtrip through the raw numeric value
const restored = new GraphId(gid.value);
console.log(restored.equals(gid)); // true
```

## Protocol Buffer Support

For performance-critical applications, Valhalla supports Protocol Buffer (pbf) format for requests and responses. This provides faster serialization/deserialization and smaller payload sizes compared to JSON.

### Supported APIs

The following methods support `format: 'pbf'`:
- `route`
- `matrix`
- `isochrone`
- `expansion`
- `traceRoute`
- `traceAttributes`

### Using Protocol Buffer Format

When you request `format: 'pbf'`, the method returns a Node.js Buffer containing serialized protobuf data instead of a parsed JSON object.

```javascript
import { Actor } from '@valhallajs/valhallajs';
import protobuf from 'protobufjs';
import { fileURLToPath } from 'url';
import { dirname, join } from 'path';

const __dirname = dirname(fileURLToPath(import.meta.url));

async function main() {
  const actor = await Actor.fromConfigFile('config.json');

  // Load the protobuf definition
  const protoPath = join(__dirname, 'node_modules/@valhallajs/valhallajs/proto/api.proto');
  const root = await protobuf.load(protoPath);
  const Api = root.lookupType('valhalla.Api');

  // Request in pbf format
  const buffer = await actor.route({
    locations: [
      { lat: 52.5200, lon: 13.4050 },
      { lat: 52.5300, lon: 13.4150 }
    ],
    costing: 'auto',
    format: 'pbf'  // Returns Buffer instead of JSON object
  });

  // Decode the protobuf buffer
  const message = Api.decode(buffer);
  const response = Api.toObject(message, {
    longs: String,
    enums: String,
    bytes: String
  });

  console.log(response.trip);
  console.log(response.directions);
}

main();
```

### Installing protobufjs

To decode protobuf responses, install protobufjs:

```bash
npm install protobufjs
```

### Benefits of Protocol Buffer Format

- **Performance**: 3-10x faster serialization/deserialization
- **Size**: 20-50% smaller payload sizes
- **Bandwidth**: Reduced network transfer costs
- **Type Safety**: Strongly-typed schema definitions

### Proto Files Location

The protocol buffer definitions are included in the package at `node_modules/@valhallajs/valhallajs/proto/`. The main entry point is `api.proto`, which imports all other necessary definitions.

## Compatibility

### Node.js Versions
- Node.js v16.0.0 and all later versions
- Node.js v15.12.0+
- Node.js v14.17.0+
- Node.js v12.22.0+

### Platforms
- Linux (arm64, x64)
- macOS (arm64)
