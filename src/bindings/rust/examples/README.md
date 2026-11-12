# Examples

Each example is an independent crate, so its dependencies are clear. Run from this `examples/`
directory; every example documents its own arguments:

```sh
cargo run -p <example> -- --help
cargo run -p valhalla-tools --bin valhalla_build_elevation -- --help
cargo run -p valhalla-tools --bin valhalla_build_config -- --help
```

| Example | Demonstrates | Key APIs |
|---|---|---|
| [ferry-lines](ferry-lines/src/main.rs) | scan a whole tileset in parallel, stitch routes across tiles, name places without a geocoder | `GraphReader::tiles`, `edgeinfo`, `opp_index`, `admin_info`, `TimeZoneInfo`, logging to std_err |
| [reachability](reachability/src/main.rs) | snap a coordinate the way the router does, then run your own search under Valhalla's access rules | `Actor::locate`, `Access`, `node_transitions` |
| [match-polyline](match-polyline/src/main.rs) | map-match with typed protobuf and walk the response | `Actor::trace_attributes`, `proto::Options`, `proto::Api` |
| [traffic_debug](traffic_debug/src/main.rs) | read and write a live `traffic.tar` through a typed API | `LiveTraffic`, `TrafficTile`, `ConfigBuilder` |
| [valhalla-tools](valhalla-tools/src/main.rs) | drop-in Rust versions of three scripts, without libvalhalla: `valhalla_build_extract` tars a tile set with its `index.bin`, optionally with an empty `traffic.tar`; [`valhalla_build_elevation`](valhalla-tools/src/bin/valhalla_build_elevation.rs) downloads the elevation tiles of a bbox, GeoJSON polygons or a graph, as gzip, raw `.hgt` or LZ4; [`valhalla_build_config`](valhalla-tools/src/bin/valhalla_build_config.rs) writes a config, optionally merged into an existing one, byte for byte as the script does, with the defaults and help texts its `build.rs` reads from `scripts/valhalla_build_config` | none, it only reads tile headers and paths |
| [valhalla-service](valhalla-service/src/main.rs) | Rust version of Valhalla's [`valhalla_service`](https://github.com/valhalla/valhalla/blob/master/src/valhalla_service.cc) with Actor API over HTTP, JSON and protobuf | `Actor`, all Valhalla endpoints |
