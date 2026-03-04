# CLAUDE.md — Valhalla Project Guide

Open-source C++ routing engine for OpenStreetMap data: turn-by-turn routing, time-distance matrices, isochrones, map matching, elevation queries, optimized routes (TSP). C++20, CMake, single `libvalhalla` shared library from ~10 Norse-mythology-themed modules.

## Critical Constraints

Read this section first. Violating these constraints causes real damage at planet scale.

**Tile format is frozen.** The routing graph is serialized into binary tiles containing `DirectedEdge`, `NodeInfo`, `GraphTileHeader`, `EdgeInfo`, `Sign`, `AdminInfo`, `TransitDeparture`, and other bit-packed structs. Never change their binary layout — no field reordering, no bitfield width changes. The last tile version bump was ~7 years ago. Thousands of users run pre-built tilesets they cannot rebuild (~7-hour planet build). Extending `DirectedEdge` or `NodeInfo` is not an option — there are only 4 spare bits in `DirectedEdge` and 1 in `NodeInfo`. The preferred way to add new per-edge data is via `TaggedValue` entries in `EdgeInfo`'s variable-length name/tag list (see `TaggedValue` enum in `valhalla/baldr/graphconstants.h`).

**Every addition is multiplied at planet scale.** The graph contains ~1 billion `DirectedEdge`s, ~500 million `NodeInfo`s, and ~3 billion `OSMWayNode`s (the intermediate `way_node.bin` alone is ~110 GiB). Adding a `TaggedValue` to `EdgeInfo` is safe for the format but still grows every affected tile. Parsing new OSM way types creates more edges and nodes across those billions. Always consider the multiplicative cost of any graph-level change.

**Costing functions are the hottest path.** `EdgeCost()`, `TransitionCost()`, `Allowed()` in `src/sif/` are called millions of times per request.

**arm64 (Apple Silicon) instability.** Some tests fail on Apple Silicon due to numeric differences from x86_64. Always run relevant tests **before** making changes to establish a baseline.

## Build and Test Commands

```bash
# Build (from repo root)
cd build && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && cmake --build . -j$(nproc)

# Unit test — target name = filename without extension (test/directededge.cc → directededge)
cd build && cmake --build . -j$(nproc) --target directededge && ./test/directededge

# Gurka integration test — target gurka_<name> from test/gurka/test_<name>.cc
cd build && cmake --build . -j$(nproc) --target gurka_filter && ./test/gurka/gurka_filter

# Single GoogleTest case
./test/gurka/gurka_access --gtest_filter="*YourTestName*"

# Multiple related tests
cmake --build . -j$(nproc) --target gurka_access --target gurka_route && \
  ./test/gurka/gurka_access && ./test/gurka/gurka_route

# Format — or use clang-format-11 directly `clang-format-11 -i src/**/*.h src/**/*.cc test/**/*.h test/**/*.cc`
./scripts/format.sh
```

**Build parallelism:** `-j$(nproc)` works on Linux; on macOS use `-j$(sysctl -n hw.logicalcpu)` or install `coreutils` for `nproc`. Alternatively, configure CMake with Ninja (`cmake -G Ninja ..` or `CMAKE_GENERATOR=Ninja`), which parallelizes automatically without needing `-j`.

**IMPORTANT:** Avoid `make check` — extremely slow and produces false positives on arm64. Run only the relevant tests.

### Key CMake Options

| Option | Default | Purpose |
|--------|---------|---------|
| `ENABLE_DATA_TOOLS` | ON | Tile building tools and gurka tests |
| `ENABLE_SERVICES` | ON | HTTP service via prime_server |
| `ENABLE_TESTS` | ON | Build test targets |
| `ENABLE_SANITIZERS` | OFF | All sanitizers (Debug builds) |
| `ENABLE_ADDRESS_SANITIZER` | OFF | ASan only |
| `LOGGING_LEVEL` | INFO | NONE, ALL, ERROR, WARN, INFO, DEBUG, TRACE |

## Architecture

### Module Map

```
Request → Tyr → Loki → Thor → Odin → Tyr → Response
                  ↑        ↑
                Baldr     Sif
                  ↑        ↑
               Midgard   Skadi

Meili (map matching) → Loki + Thor
Mjolnir (tile builder) → Baldr + Skadi + Midgard
```

Headers: `valhalla/<module>/`. Source: `src/<module>/`.

| Module | What It Does |
|--------|-------------|
| **Midgard** | Geometry primitives (Point2/PointLL, vectors, bounding boxes, polyline encoding). Foundation for everything. |
| **Baldr** | Graph data structures: GraphId, GraphTile, NodeInfo, DirectedEdge, EdgeInfo, tile reading/caching. |
| **Sif** | Runtime costing via `DynamicCost` (`Allowed()`, `EdgeCost()`, `TransitionCost()`). Costs are never baked into tiles. Models: auto, bicycle, pedestrian, truck, bus, taxi, motor_scooter, motorcycle, multimodal, bikeshare. Factory: `valhalla/sif/costfactory.h`. |
| **Skadi** | Elevation (DEM) data access. |
| **Mjolnir** | Tile builder. OSM PBF → Lua tag parsing (`lua/graph.lua`) → tiles. Only when `ENABLE_DATA_TOOLS=ON`. |
| **Loki** | Location correlation: lat/lon → graph edges via per-tile 5x5 spatial bin index. |
| **Meili** | Map matching via HMM + Viterbi. |
| **Thor** | Pathfinding: BidirectionalAStar (primary), TimeDepForward/Reverse, MultiModal. Also matrices and isochrones. |
| **Odin** | Maneuver generation and narrative in 30+ languages (`locales/`). |
| **Tyr** | API orchestration + serialization (Valhalla JSON, OSRM JSON, GPX, Protobuf). |

### The Graph (Essential Knowledge)

All in `valhalla/baldr/`. Understanding these relationships is essential for almost any Valhalla task:

- **GraphId** — 64-bit: 3-bit level + 22-bit tile index + 21-bit local index. Same format for nodes and edges — context determines which.
- **NodeInfo** — An intersection. Points to its outgoing directed edges via `edge_index` + `edge_count` (within the same tile).
- **DirectedEdge** — One direction of a road segment. `endnode()` returns a GraphId that may cross tiles. `opp_index` points to the reverse-direction edge.
- **EdgeInfo** — Shared by forward + reverse edge pair: geometry (polyline), street names, elevation, OSM way ID.
- **GraphTile** — Container for all of the above, plus signs, restrictions, admin info, etc.

Three hierarchy levels:

| Level | Tile Size | Contains |
|-------|-----------|----------|
| 0 | 4° | Highways + shortcut edges |
| 1 | 1° | Arterials + shortcut edges |
| 2 | 0.25° | Local roads |

**Shortcut edges** on levels 0–1 collapse chains of base edges through contractible nodes. Built by `ShortcutBuilder` (`src/mjolnir/shortcutbuilder.cc`). Each shortcut is a `DirectedEdge` with `is_shortcut()` true; each replaced base edge is marked `superseded()`. Shortcuts speed up BidirectionalAStar; UnidirectionalAStar (time-dependent) does not use them. `GraphReader::RecoverShortcut()` expands them back to base edges for the final path.

Tiles are memory-mapped from a tar extract in production. `GraphReader` manages a 1 GB in-memory cache — only a fraction of ~205k planet tiles fit at once.

### Request Flow

| Action | Pipeline |
|--------|----------|
| route | `loki.route()` → `thor.route()` → `odin.narrate()` → serialize |
| matrix | `loki.matrix()` → `thor.matrix()` → serialize |
| isochrone | `loki.isochrone()` → `thor.isochrone()` → serialize |
| trace_route | `loki.trace()` → `thor.trace_route()` → `odin.narrate()` → serialize |
| trace_attributes | `loki.trace()` → `thor.trace_route()` → serialize |
| locate, height, status | Loki-only → serialize |
| expansion | `loki.route()` → `thor.expansion()` → serialize |

Actors communicate via Protobuf (`Api` message in `proto/api.proto`). The final "serialize" step in Tyr converts the pbf into the output format — JSON (Valhalla or OSRM-compatible), GPX, or pbf. Not all endpoints support pbf output.

Deployment modes: HTTP server (`valhalla_service`), distributed ZMQ workers (`valhalla_*_worker`), one-shot CLI, embedded library (`tyr::actor_t`).

### OSM Data Pipeline

`build_tile_set()` in `src/mjolnir/util.cc`:

```
OSM PBF → lua/graph.lua (tag transformation)
  → pbfgraphparser.cc (three passes: way/relation/node)
  → GraphBuilder (ways split at intersections → DirectedEdge + NodeInfo)
  → GraphEnhancer → GraphFilter → HierarchyBuilder → ShortcutBuilder
  → ElevationBuilder → RestrictionBuilder → GraphValidator
```

The Lua layer (`lua/graph.lua`) controls tag-to-attribute mapping without recompilation — key tables: `highway`, `road_class`, `default_speed`, `restriction`. Intermediate data flows through `OSMData` and temporary `.bin` files. See `docs/docs/mjolnir/tag_parsing.md` for the full Lua ↔ C++ tag flow and debugging tips.

## Where to Look

This is the most important navigation aid. Large files like `pbfgraphparser.cc` (5400+ lines) and `bidirectional_astar.cc` make knowing where to start essential.

| You're Working On | Start Here |
|-------------------|-----------|
| OSM tag parsing, which tags produce which attributes | `lua/graph.lua`, `src/mjolnir/pbfgraphparser.cc` |
| How edges/nodes get their properties during tile build | `src/mjolnir/graphbuilder.cc`, `src/mjolnir/graphenhancer.cc` |
| Adding new per-edge data to tiles | `TaggedValue` enum in `valhalla/baldr/graphconstants.h`, stored in `EdgeInfo` name/tag list (`valhalla/baldr/edgeinfo.h`) |
| Whether a vehicle type can use an edge, costing weights | `src/sif/` — each model has its own file (e.g., `autocost.cc`, `bicyclecost.cc`). See `docs/docs/sif/dynamic-costing.md` |
| Routing algorithm behavior | `src/thor/bidirectional_astar.cc`, `astar.cc`, `timedep_forward.cc`, `timedep_reverse.cc`. See `docs/docs/thor/path-algorithm.md` |
| How lat/lon maps to graph edges | `src/loki/search.cc` (bin search → projection → filtering → reachability) |
| Turn-by-turn maneuver generation | `src/odin/maneuversbuilder.cc`, `src/odin/narrativebuilder.cc` |
| API response serialization (pbf → JSON/GPX/pbf output) | `src/tyr/` — `route_serializer_valhalla.cc`, `route_serializer_osrm.cc`, `matrix_serializer.cc`, and other `*_serializer.cc`. New output fields must be added to the `.proto` definition first, then to the serializer |
| Error codes and HTTP status mapping | `src/exceptions.cc` (100s=Loki, 200s=Odin, 300s=Skadi, 400s=Thor, 500s=Tyr) |
| Protobuf message definitions | `proto/` — root message is `Api` in `api.proto` |
| Live/historical traffic | Separate binary tile files; test via `test::customize_live_traffic_data()` and `test::customize_historical_traffic()` in `test/test.h` |
| Time-dependent routing | `depart_at`/`arrive_by` params; timezone data from `tz.sqlite` |
| Route API request/response format | `docs/docs/api/turn-by-turn/api-reference.md` |
| Speed assignment (maxspeed, highway defaults, density) | `docs/docs/speeds.md` |
| Domain terminology (cost vs penalty vs factor) | `docs/docs/terminology.md` |

## Performance at Planet Scale

| Metric | Value |
|--------|-------|
| OSM planet PBF | ~85 GiB |
| Output tileset | ~PBF size + 10% (~95 GiB for planet) |
| Total tiles | ~205,000 |
| DirectedEdge objects | ~1 billion |
| NodeInfo objects | ~500 million |
| Parsed OSM nodes | ~3 billion |
| `way_node.bin` intermediate | ~110 GiB |
| Full planet build time | ~7 hours on m8gd.8xlarge (32 vCPU, 128 GiB, NVMe) |

**Why this matters for every change:**
- **Data structure size** → +1 byte on DirectedEdge = +1 GiB tileset. Bit-packing is intentional.
- **Tile format stability** → binary layout changes break every user's pre-built tileset. Not an option.
- **Build intermediates** → `way_node.bin` at 110 GiB means changes to `OSMWayNode` or node processing directly inflate build time and disk.
- **Tile I/O** → memory-mapped in production. Larger tiles = more memory pressure, slower cold starts.
- **Cache budget** → 1 GB cache holds only a fraction of ~205k tiles. Cache-friendly access patterns matter.

## Testing

### Unit Tests vs Gurka Integration Tests

**Unit tests** (`test/*.cc`) — target name = filename. Test individual functions/modules. Many use pre-built tilesets from `test/data/` (utrecht, whitelion, roma, etc.).

**Gurka integration tests** (`test/gurka/test_*.cc`) — target `gurka_<name>`. Build ASCII road maps → generate tiles → run full API → verify. Use for testing routing behavior, access restrictions, turn restrictions, costing, maneuvers — anything requiring multiple modules. See `docs/docs/test/gurka.md` for the full framework reference including map construction, relations, assertions, and debugging with GeoJSON.

### Gurka Test Pattern

Read any existing gurka test as a template. The basic pattern:

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

### Key Test Helpers

**Gurka:** `gurka::buildtiles()`, `gurka::do_action()`, `gurka::findEdge()`, `gurka::findEdgeByNodes()`, `gurka::findNode()`, `gurka::assert::raw::expect_path()`, `gurka::assert::raw::expect_maneuvers()`, `gurka::assert::osrm::expect_steps()`.

**Utilities** (`test/test.h`): `test::make_config()`, `test::build_live_traffic_data()`, `test::customize_live_traffic_data()`, `test::customize_historical_traffic()`, `test::customize_edges()`, `test::make_clean_graphreader()`.

### Conventions

- `TEST(SuiteName, TestName)` or `TEST_F(FixtureClass, TestName)`. PascalCase, no underscores in test names.
- Prefer `EXPECT_*` over `ASSERT_*` (test continues on failure, giving more diagnostic info).
- Always baseline tests before changes — especially on arm64 where some tests have pre-existing failures.

## Development Workflow

### 1. Baseline Before Changing

**IMPORTANT:** Always run the relevant test(s) before any code changes:
```bash
cd build && cmake --build . -j$(nproc) --target gurka_access && ./test/gurka/gurka_access
```
This catches pre-existing arm64 failures and establishes what "passing" looks like.

### 2. Add a Failing Test

Write a `TEST` that demonstrates the expected behavior. For bug fixes, it should reproduce the bug or requires missing routing feature:
```bash
cmake --build . -j$(nproc) --target gurka_access && ./test/gurka/gurka_access --gtest_filter="*YourNewTest*"
```

### 3. Trace the Pipeline and Fix

Use the "Where to Look" table above to find the right file. The pipeline flows left to right: tag parsing → graph building → costing → routing → maneuvers → serialization.

### 4. Iterate Until Green

```bash
cmake --build . -j$(nproc) --target gurka_access && ./test/gurka/gurka_access --gtest_filter="*YourNewTest*"
```

### 5. Verify Related Tests

**IMPORTANT:** Always finish with related gurka tests to catch regressions:
```bash
cmake --build . -j$(nproc) --target gurka_access --target gurka_route --target gurka_filter && \
  ./test/gurka/gurka_access && ./test/gurka/gurka_route && ./test/gurka/gurka_filter
```

Never skip this step. Avoid running the entire suite — it is extremely slow and has false positives on arm64.

## Code Style

- **C++20**, 2-space indent, 102-col limit, left-aligned pointers (`int* p`, not `int *p`)
- **MUST** use either script `./scripts/format.sh` or clang-format-11 directly — newer versions produce different output
- clang-tidy checks: `bugprone-*`, `performance-*`, `modernize-*`, `clang-analyzer-*`
- Errors thrown as `valhalla_exception_t` with numeric code — ranges in `src/exceptions.cc`
- Configuration via `boost::property_tree::ptree`, JSON format

## Running a Route Locally

`valhalla_service` supports a one-shot CLI mode (no HTTP server needed): `valhalla_service config.json <action> '<json_request>'`. This is the fastest way to test routing against real tiles. The same mechanism powers `tyr::actor_t` used in gurka tests and Python bindings.

### Quick Setup: Build Tiles and Route

```bash
# Download a small OSM extract (Liechtenstein is ~2 MB, builds in seconds)
wget https://download.geofabrik.de/europe/liechtenstein-latest.osm.pbf

# Generate config
mkdir -p valhalla_tiles
valhalla_build_config --mjolnir-tile-dir ${PWD}/valhalla_tiles \
  --mjolnir-tile-extract ${PWD}/valhalla_tiles.tar \
  --mjolnir-timezone ${PWD}/valhalla_tiles/timezones.sqlite \
  --mjolnir-admin ${PWD}/valhalla_tiles/admins.sqlite > valhalla.json

# Build supporting databases and routing tiles
valhalla_build_timezones > valhalla_tiles/timezones.sqlite
valhalla_build_admins -c valhalla.json liechtenstein-latest.osm.pbf
valhalla_build_tiles -c valhalla.json liechtenstein-latest.osm.pbf

# Create tar extract for faster tile loading
valhalla_build_extract -c valhalla.json -v

# Route from Vaduz to Schaan (one-shot, no server)
valhalla_service valhalla.json route '{"locations":[{"lat":47.141,"lon":9.521},{"lat":47.165,"lon":9.510}],"costing":"auto"}'

# Pipe through jq for readable output
valhalla_service valhalla.json route '{"locations":[{"lat":47.141,"lon":9.521},{"lat":47.165,"lon":9.510}],"costing":"auto"}' 2>/dev/null | jq '.trip.summary'
```

The third argument can also be a path to a JSON file. The `test_requests/` directory contains ~250 files with example requests for various scenarios (demo routes, truck routes, bicycle routes, transit, restrictions, etc.) — useful as templates for crafting test requests.

### Route Geometry: Polyline6 Encoding

Valhalla encodes route geometries as polyline strings with **6 digits of precision** (polyline6), not the 5-digit precision from Google's original spec. This is critical — using 5-digit precision will place points in the ocean.

When a code change affects route geometry, paste the encoded `shape` string from the response into the [Valhalla polyline decoder](https://valhalla.github.io/demos/polyline/?unescape=true&polyline6=true) to visually verify the route on a map.

See `docs/docs/decoding.md` for decode implementations in C++, Python, JavaScript, Go, and Rust.

## Reference

### API Endpoints

All accept JSON via POST. Protobuf definitions in `proto/` — root message `Api` in `api.proto`.

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

### File Layout

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
```

### Key Executables

- `valhalla_service` — HTTP server (multi-threaded) or one-shot CLI mode
- `valhalla_build_tiles` — Build routing tiles from OSM PBF
- `valhalla_build_admins` / `valhalla_build_timezones` — Build admin/timezone databases
- `valhalla_build_extract` — Create tar extract from tiles
- `valhalla_build_config` — Generate default configuration JSON

### In-Repo Documentation

The `docs/docs/` directory contains detailed documentation. The most useful for development:

| Document | What It Covers |
|----------|---------------|
| `route_overview.md` | End-to-end route computation pipeline (Loki → Thor → Odin → Tyr) |
| `terminology.md` | Domain glossary: cost vs penalty vs factor, edge, maneuver, trip |
| `sif/dynamic-costing.md` | Costing design: EdgeCost, TransitionCost, turn penalties, name consistency |
| `thor/path-algorithm.md` | A*, BidirectionalA*, MultiModal, hierarchy levels, edge labeling, shortcuts |
| `tiles.md` | Tile math: GraphId layout, lat/lon ↔ tile index, bounding box queries |
| `speeds.md` | Speed assignment: maxspeed tags, highway defaults, urban/rural density |
| `mjolnir/tag_parsing.md` | OSM tag flow: Lua → C++ marshalling, debugging route quality issues |
| `test/gurka.md` | Gurka framework: ASCII maps, ways/nodes/relations, assertions, GeoJSON debugging |
| `decoding.md` | Polyline6 encoding/decoding with examples in multiple languages |
| `api/turn-by-turn/api-reference.md` | Route API: request format, costing options, response structure |
| `building.md` | Building from source and running Valhalla server on all platforms |

## Maintaining This Document

This is a living document. Valhalla has 10+ years of history and receives continuous improvements — no static guide can cover everything. When working on a task, you may discover knowledge that would save future agents significant time. Update this file when appropriate, but respect its role as an AI-first document — structured for how agents parse and act on information.

**When to add something:**
- A non-obvious gotcha that caused a wrong approach or wasted build/test cycle
- A pattern or convention not documented here that required reading multiple files to discover
- A new test helper, build target, or tool that is broadly useful
- A "Where to Look" entry for a problem domain not yet covered

**When NOT to add:**
- Information already covered here (check first — duplicates dilute the document)
- Details specific to a single task or bug fix (belongs in code comments or commit messages)
- Anything available in `docs/docs/` that can be reached via the In-Repo Documentation table above
- Speculative guidance not validated by actually building and testing

**Style rules to preserve:**
- Tables over prose. A row in "Where to Look" beats a paragraph.
- Bold warnings (`**IMPORTANT:**`) for things that cause silent failures or wasted time.
- Concrete over abstract. File paths, command lines, struct names — not "the relevant module."
- No filler. Every sentence should answer a question an agent would actually have.
- Keep sections self-contained. An agent may be pointed at a single section, not the whole file.
- Critical constraints stay at the top. Performance and compatibility warnings must never drift below the fold.
