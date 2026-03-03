# CLAUDE.md — Valhalla Project Guide

## Project Overview

Valhalla is an open-source C++ routing engine for OpenStreetMap data. It provides turn-by-turn routing, time-distance matrices, isochrones, map matching, elevation queries, and optimized routes (TSP). All modules are Norse-mythology-themed.

C++20, CMake build system, ~10 library modules compiled as CMake OBJECT libraries linked into a single `libvalhalla` shared library.

## Build and Test Commands

```bash
mkdir build && cd build

# Build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && cmake --build . -j 12

# Build + run a single unit test
cd build && cmake --build . -j 12 --target directededge && ./test/directededge

# Build + run a single gurka integration test
cd build && cmake --build . -j 12 --target gurka_filter && ./test/gurka/gurka_filter

# Run a specific GoogleTest case
./test/gurka/gurka_access --gtest_filter="*YourTestName*"

# Build + run multiple related tests
cmake --build . -j 12 --target gurka_access --target gurka_route && \
  ./test/gurka/gurka_access && ./test/gurka/gurka_route

# Run all tests (slow, avoid during development, might have false positives on arm64)
make -C build check

# Format (use clang-format-11 specifically, newer versions produce different output)
clang-format-11 -i src/**/*.h src/**/*.cc test/**/*.h test/**/*.cc
```

**arm64 (Apple Silicon) warning:** Some tests fail on arm64 due to numeric instability — all tests were originally tuned for x86_64. Always run the relevant test(s) **before** making code changes to establish a baseline.

### Key CMake Options

| Option | Default | Purpose |
|--------|---------|---------|
| `ENABLE_DATA_TOOLS` | ON | Tile building tools and gurka tests |
| `ENABLE_SERVICES` | ON | HTTP service via prime_server |
| `ENABLE_HTTP` | ON | cURL for remote tile fetching |
| `ENABLE_PYTHON_BINDINGS` | ON | Python bindings (nanobind, in `src/bindings/python/`) |
| `ENABLE_TESTS` | ON | Build test targets |
| `ENABLE_SANITIZERS` | OFF | All sanitizers (Debug builds) |
| `ENABLE_ADDRESS_SANITIZER` | OFF | ASan only |
| `ENABLE_COVERAGE` | OFF | Coverage instrumentation (`make coverage`) |
| `LOGGING_LEVEL` | `""` (INFO) | NONE, ALL, ERROR, WARN, INFO, DEBUG, TRACE |

## Architecture

### Module Hierarchy

```
Request → Tyr → Loki → Thor → Odin → Tyr → Response
                  ↑        ↑
                Baldr     Sif
                  ↑        ↑
               Midgard   Skadi

Meili (map matching) → Loki + Thor
Mjolnir (tile builder) → Baldr + Skadi + Midgard
```

Headers in `valhalla/<module>/`, implementations in `src/<module>/`.

| Module | Purpose |
|--------|---------|
| **Midgard** | Foundational geometry: Point2/PointLL, vectors, bounding boxes, polyline encoding, tiling, logging. Used by all other modules. |
| **Baldr** | Graph data structures: GraphId, GraphTile, NodeInfo, DirectedEdge, EdgeInfo, tile reading/caching. |
| **Sif** | Dynamic costing. `DynamicCost` interface (`Allowed()`, `EdgeCost()`, `TransitionCost()`). Models: auto, bicycle, pedestrian, truck, bus, taxi, motor_scooter, motorcycle, multimodal, bikeshare. Created via `CostFactory` registry in `valhalla/sif/costfactory.h`. |
| **Skadi** | Elevation data (DEM) access for Sif's elevation costing and the `/height` API. |
| **Mjolnir** | Builds routing tiles from OSM PBF extracts via Lua tag parsing (`lua/graph.lua`). Only compiled when `ENABLE_DATA_TOOLS=ON`. |
| **Loki** | Location-to-graph correlation. Maps lat/lon to graph edges using per-tile bin index. |
| **Meili** | Map matching via HMM + Viterbi search. Outputs Thor-compatible data so Odin can generate directions. |
| **Thor** | Path computation: BidirectionalAStar (main), TimeDepForward/Reverse, AStar (trivial), MultiModal (transit). Also matrices and isochrones. |
| **Odin** | Maneuver generation and narrative text. 30+ languages via Transifex translations in `locales/`. |
| **Tyr** | API serialization and orchestration. Outputs Valhalla JSON, OSRM JSON, GPX, or Protocol Buffers. |

### Core Graph Data Structures

All in `valhalla/baldr/`:

- **GraphId** — 64-bit ID: 3-bit hierarchy level + 22-bit tile index + 21-bit node/edge index. Same format for nodes and edges — context determines which.
- **GraphTile** — A single tile. Contains arrays of nodes, directed edges, edge info, and supplementary data.
- **NodeInfo** — Intersection node. Points to outgoing edges via `edge_index` and `edge_count` (within same tile).
- **DirectedEdge** — Directed edge. `endnode()` returns a GraphId that may point to a different tile. `opp_index` points to the reverse edge.
- **EdgeInfo** — Shared metadata for forward + reverse edge pair: shape (polyline), names, way ID.

The graph uses an adjacency list format split into tiles. Each node points to its outgoing edges via `edge_index` and `edge_count` (indexes within the same tile). Each `DirectedEdge` has an `endnode()` GraphId that may point to a different tile. Each edge also has an `opp_index` pointing to the reverse direction edge.

### Tile Hierarchy

| Level | Tile Size | Road Classes |
|-------|-----------|-------------|
| 0 | 4° | Highways (motorway, trunk, primary) |
| 1 | 1° | Arterials (secondary, tertiary) |
| 2 | 0.25° | Local (residential, service, unclassified) |

Higher levels contain shortcut edges for fast long-distance routing. Tiles are loaded from a memory-mapped tar extract (production) or individual files (development). `GraphReader` (`valhalla/baldr/graphreader.h`) manages tile loading with an in-memory cache (default 1 GB, configurable via `mjolnir.max_cache_size`).

### Request Pipeline

Three workers extend `service_worker_t` (`valhalla/worker.h`):
- `loki_worker_t` — input validation, location correlation
- `thor_worker_t` — routing algorithms, CostFactory
- `odin_worker_t` — maneuver generation, narrative

**Deployment modes:**
- **HTTP server** (`valhalla_service config.json [concurrency]`) — prime_server with ZMQ, N worker threads per stage
- **Distributed workers** (`valhalla_loki_worker`, `valhalla_thor_worker`, `valhalla_odin_worker`) — each stage as a separate process via ZMQ
- **One-shot CLI** (`valhalla_service config.json action request.json`) — uses `tyr::actor_t` in-process, no HTTP
- **Library** (`tyr::actor_t` from `valhalla/tyr/actor.h`) — embedded use, also used by gurka tests and Python bindings

**Per-action pipelines:**

| Action | Pipeline |
|--------|----------|
| route | `loki.route()` → `thor.route()` → `odin.narrate()` → serialize |
| matrix | `loki.matrix()` → `thor.matrix()` → serialize |
| isochrone | `loki.isochrone()` → `thor.isochrone()` → serialize |
| trace_route | `loki.trace()` → `thor.trace_route()` → `odin.narrate()` → serialize |
| trace_attributes | `loki.trace()` → `thor.trace_route()` → serialize |
| locate, height, status | Loki-only → serialize |
| expansion | `loki.route()` → `thor.expansion()` → serialize |

### OSM Data Pipeline

Orchestrated by `build_tile_set()` in `src/mjolnir/util.cc`:

```
OSM PBF
  → lua/graph.lua (tag transformation — configurable without recompilation)
  → pbfgraphparser.cc: way() / relation() / node() (three ordered passes)
  → GraphBuilder::BuildEdges (ways split at intersections)
  → GraphBuilder::Build (edges → DirectedEdge + NodeInfo in tiles)
  → GraphEnhancer → GraphFilter → HierarchyBuilder → ShortcutBuilder
  → ElevationBuilder → RestrictionBuilder → GraphValidator
  → Ready for routing
```

Intermediate data flows through `OSMData` (container for `OSMWay`, `OSMWayNode`, etc.) and temporary `.bin` files. The Lua layer (`lua/graph.lua`) controls tag-to-attribute mapping — key tables: `highway`, `road_class`, `default_speed`, `restriction`.

## Configuration

JSON format, loaded into `boost::property_tree::ptree` via `valhalla::config()`. Generate defaults with:
```bash
scripts/valhalla_build_config --mjolnir-tile-dir ./tiles --mjolnir-tile-extract ./tiles.tar > valhalla.json
```

Key sections: `mjolnir` (tile paths, concurrency), `loki`/`thor` (service limits), `meili` (map matching params), `httpd` (server port), `service_limits` (per-endpoint limits). Access via ptree path traversal: `config.get<std::string>("mjolnir.tile_dir")`.

## Error Handling

Errors are thrown as `valhalla_exception_t` (`valhalla/exceptions.h`) with numeric code, message, HTTP status, and OSRM error string. All codes defined in `src/exceptions.cc`. Code ranges: 100s = Loki/parsing, 200s = Odin, 300s = Skadi, 400s = Thor, 500s = Tyr/serialization. Non-fatal issues use `add_warning(api, code)`.

## Testing

### Unit Tests vs Gurka Integration Tests

**Unit tests** (`test/*.cc`) — test individual functions or modules in isolation. Target name matches filename. Link against `valhalla_test` (shared library from `test/test.cc` + `test/test.h`). Many use pre-built tilesets from `test/data/` (utrecht, whitelion, roma, etc.). Use for testing a specific function, parser, serializer, or algorithm.

**Gurka tests** (`test/gurka/test_*.cc`) — end-to-end: define ASCII road maps → build OSM + tiles → run full API requests → verify results. File `test_<name>.cc` produces target `gurka_<name>`. Use for testing routing behavior, access restrictions, turn restrictions, costing, maneuvers — anything requiring multiple modules.

### Gurka Test Pattern

```cpp
TEST(MyFeature, BasicCase) {
  const std::string ascii_map = R"(
      A----B----C
           |
           D
  )";
  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}}},
      {"BC", {{"highway", "residential"}}},
      {"BD", {{"highway", "service"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/my_test");
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "C"}, "auto");
  gurka::assert::raw::expect_path(result, {"AB", "BC"});
}
```

**Key helpers:** `gurka::buildtiles()`, `gurka::do_action()`, `gurka::findEdge()`, `gurka::findEdgeByNodes()`, `gurka::findNode()`, `gurka::assert::raw::expect_path()`, `gurka::assert::raw::expect_maneuvers()`, `gurka::assert::osrm::expect_steps()`.

**Test utilities** (`test/test.h`): `test::make_config()`, `test::build_live_traffic_data()`, `test::customize_live_traffic_data()`, `test::customize_historical_traffic()`, `test::customize_edges()`, `test::make_clean_graphreader()`.

### Test Conventions

- `TEST(SuiteName, TestName)` or `TEST_F(FixtureClass, TestName)`. PascalCase, no underscores in test names.
- Prefer `EXPECT_*` over `ASSERT_*`.

## Development Workflow

### 1. Find the Right Test

- `test/gurka/` for integration tests (e.g., `test_access.cc`, `test_route.cc`, `test_restrictions.cc`)
- `test/` for unit tests (e.g., `directededge.cc`, `graphparser.cc`, `factory.cc`)

### 2. Baseline It

Always run the test before any code changes to establish a baseline, for example to run `test/gurka/test_access.cc`:
```bash
cd build && cmake --build . -j 12 --target gurka_access && ./test/gurka/gurka_access
```

### 3. Add a Failing Test

Write a `TEST` that demonstrates the expected behavior. For bug fixes, it should reproduce the bug and fail:
```bash
cmake --build . -j 12 --target gurka_access && ./test/gurka/gurka_access --gtest_filter="*YourNewTest*"
```

### 4. Trace the Pipeline and Fix

- **Tag parsing / data?** → `lua/graph.lua` or `src/mjolnir/pbfgraphparser.cc`
- **Edge/node attributes?** → `src/mjolnir/graphbuilder.cc` or `src/mjolnir/graphenhancer.cc`
- **Access / costing?** → `src/sif/` (the relevant cost model, e.g., `autocost.cc`)
- **Routing algorithm?** → `src/thor/` (`bidirectional_astar.cc`, `astar.cc`, `timedep_*.cc`)
- **Location correlation?** → `src/loki/search.cc`
- **Maneuvers / narrative?** → `src/odin/maneuversbuilder.cc`, `src/odin/narrativebuilder.cc`
- **Serialization?** → `src/tyr/` (`route_serializer_*.cc`, `matrix_serializer.cc`)

### 5. Iterate Until Green

```bash
cmake --build . -j 12 --target gurka_access && ./test/gurka/gurka_access --gtest_filter="*YourNewTest*"
```

### 6. Verify Related Tests

Always finish with high-level gurka tests to catch regressions:
```bash
cmake --build . -j 12 --target gurka_access --target gurka_route --target gurka_filter && \
  ./test/gurka/gurka_access && ./test/gurka/gurka_route && ./test/gurka/gurka_filter
```

Avoid running the entire suite — it is extremely slow and produces false positives on arm64.

## Code Style

- **C++20**, 2-space indent, 102-col limit, left-aligned pointers
- Use clang-format-11 specifically (newer versions produce different output)
- clang-tidy checks: bugprone-*, performance-*, modernize-*, clang-analyzer-*

## API Endpoints

| Endpoint | Description |
|----------|-------------|
| `/route` | Turn-by-turn routing with maneuvers |
| `/sources_to_targets` | Time-distance matrix |
| `/isochrone` | Reachability polygons by time or distance |
| `/trace_route` | Map matching → route directions |
| `/trace_attributes` | Map matching → edge attributes |
| `/height` | Elevation at points or along path |
| `/optimized_route` | TSP-optimized route |
| `/locate` | Graph metadata for nearby edges/nodes |
| `/expansion` | Debug: edges visited during search |
| `/status` | Health check and tileset info |

All accept JSON via POST. Protobuf definitions in `proto/` — root message is `Api` in `api.proto`.

## File Layout

```
valhalla/          # Public headers (valhalla/<module>/*.h)
src/               # Implementations + CLI tools (valhalla_*.cc)
  bindings/        # Python (nanobind) and Node.js bindings
proto/             # Protobuf definitions (*.proto)
test/              # Unit tests + test helpers (test.h/test.cc)
  gurka/           # Integration test framework (~136 tests)
  data/            # Test fixtures: OSM PBFs, admin DBs, traffic CSVs
lua/               # OSM tag parsing scripts (graph.lua, admin.lua)
locales/           # Translation JSON files (~30 languages)
third_party/       # Vendored deps: rapidjson, date, googletest, etc.
scripts/           # Dev scripts: format.sh, valhalla_build_config, CI
date_time/         # CLDR timezone mapping (compiled into binary)
docs/              # MkDocs documentation
```

## Key Executables

- `valhalla_service` — HTTP server (multi-threaded) or one-shot CLI mode
- `valhalla_loki_worker` / `valhalla_thor_worker` / `valhalla_odin_worker` — standalone worker processes
- `valhalla_build_tiles` — Build routing tiles from OSM PBF
- `valhalla_build_admins` / `valhalla_build_timezones` — Build admin/timezone databases
- `valhalla_build_extract` — Create tar extract from tiles

## Important Notes

- **Costs are dynamic, not baked into tiles.** Costing is computed at runtime via `DynamicCost` (`valhalla/sif/dynamiccost.h`), enabling custom costing without rebuilding data.
- **Lua tag parsing** (`lua/graph.lua`) controls OSM tag-to-attribute mapping without recompilation.
- **Multimodal costing** is only supported for `/route` and `/isochrone`.
- **Live and predictive traffic:** Live traffic in separate binary tile files; historical/predicted speeds per edge. Test helpers `customize_live_traffic_data()` and `customize_historical_traffic()` in `test/test.h`.
- **Date/time routing:** `depart_at`/`arrive_by` for time-dependent routing. Timezone data from `tz.sqlite` (built by `valhalla_build_timezones`).
- **Loki search pipeline:** bin search (tiles by distance) → point-to-segment projection → heading/side/road-class filtering → reachability check → `PathLocation` with `PathEdge`s.
- **Tile bin index:** Each tile has a 5x5 spatial bin index (built by Mjolnir) for fast edge lookup.
- **Shortcut edges** on higher hierarchy levels collapse lower-level subpaths for fast BidirectionalAStar.
- **OSRM compatibility:** Tyr can serialize responses in OSRM-compatible JSON format.
