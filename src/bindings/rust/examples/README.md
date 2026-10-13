# Examples

Each example is an independent crate, so its dependencies are clear. Run from this `examples/`
directory; every example documents its own arguments:

```sh
cargo run -p <example> -- --help
```

| Example | Demonstrates | Key APIs |
|---|---|---|
| [ferry-lines](ferry-lines/src/main.rs) | scan a whole tileset in parallel, stitch routes across tiles, name places without a geocoder | `GraphReader::tiles`, `edgeinfo`, `opp_index`, `admin_info`, `TimeZoneInfo`, logging to std_err |
| [reachability](reachability/src/main.rs) | snap a coordinate the way the router does, then run your own search under Valhalla's access rules | `Actor::locate`, `Access`, `node_transitions` |
| [match-polyline](match-polyline/src/main.rs) | map-match with typed protobuf and walk the response | `Actor::trace_attributes`, `proto::Options`, `proto::Api` |
| [traffic_debug](traffic_debug/src/main.rs) | read and write a live `traffic.tar` through a typed API | `LiveTraffic`, `TrafficTile`, `ConfigBuilder` |
| [valhalla-service](valhalla-service/src/main.rs) | Rust version of Valhalla's [`valhalla_service`](https://github.com/valhalla/valhalla/blob/master/src/valhalla_service.cc) with Actor API over HTTP, JSON and protobuf | `Actor`, all Valhalla endpoints |
