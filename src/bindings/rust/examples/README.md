# Examples

Each example is an independent crate, so its dependencies are clear. Run from this `examples/`
directory; every example documents its own arguments:

```sh
cargo run -p <example> -- --help
```

| Example | Demonstrates | Key APIs |
|---|---|---|
| [ferry-lines](ferry-lines/src/main.rs) | scan a whole tileset in parallel, stitch routes across tiles, name places without a geocoder | `GraphReader::tiles`, `edgeinfo`, `opp_index`, `admin_info`, `TimeZoneInfo` |
| [reachability](reachability/src/main.rs) | snap a coordinate the way the router does, then run your own search under Valhalla's access rules | `Actor::locate`, `Access`, `node_transitions` |
| [isochrone-h3](isochrone-h3/src/main.rs) | budget-limited expansion with hierarchy limits, drawn as H3 hexagons | `Access`, `node_transitions` |
| [match-polyline](match-polyline/src/main.rs) | map-match with typed protobuf and walk the response | `Actor::trace_attributes`, `proto::Options`, `proto::Api` |
| [traffic_debug](traffic_debug/src/main.rs) | read and write a live `traffic.tar` through a typed API | `LiveTraffic`, `TrafficTile`, `ConfigBuilder` |
| [valhalla-service](valhalla-service/src/main.rs) | Rust version of Valhalla's [`valhalla_service`](https://github.com/valhalla/valhalla/blob/master/src/valhalla_service.cc) with Actor API over HTTP, JSON and protobuf | `Actor`, all Valhalla endpoints |

## Two Dijkstras, one graph

`GraphReader` gives you the road graph — the algorithm on top is yours. `reachability` and `isochrone-h3`
are two searches over the same tiles that differ on every axis that matters:

| | `reachability` | `isochrone-h3` |
|---|---|---|
| queue cost | path length | path duration |
| queue item | bare `GraphId` | label carrying path length + budget |
| hierarchy limits | none — bounded at 1.5–10 km | the point of the example |
| stops on | a stop condition | an exhausted budget |
| tile cache | shared across three searches | fused with the visited set |

The visited set is a bitset per tile rather than a `HashSet<GraphId>` which is much smaller and faster.

Neither honours turn restrictions — both label **nodes**, and a node label cannot record which edge you arrived
on. Valhalla's own path algorithms label edges for that reason. Fine for reachability and coverage questions;
not what you'd build a router on.
