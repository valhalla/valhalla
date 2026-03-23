# Speed values in Valhalla APIs

To calculate speed and related factors when routing, such as travel times, Valhalla APIs consider speed limits in the OpenStreetMap source data, defaults for a particular category of road, or a measure of whether the road is in an urban or rural environment.

## Assignment of speeds to roadways

Routing data contains two attributes to denote speed: `speed` and `speed_limit`.

The most important for routing determination is `speed`, given in units of kilometers per hour. The `speed` value, along with the length of the roadway edge, determine the travel time along a road section.

The `speed_limit` contains the posted speed limit, if available, and can be used by mobile navigation applications to display the speed limit and possibly alert the driver when it is exceeded.

The base `speed` is assigned during tile building in the following order:

1. **`maxspeed` tag**: If a [`maxspeed`](https://wiki.openstreetmap.org/wiki/Key:maxspeed) tag is available from OSM, that speed is used as the routing speed and the `speed_limit` is set to that value. Note that `maxspeed=none` is valid and means the speed limit is unlimited (as on the German Autobahn); in this case we do not set a speed based on `maxspeed` but rely on the default speeds based on the `highway` tag (see next item).
2. **`highway` tag**: If there is no `maxspeed` or `maxspeed=none` tag, then `speed` is based on the OSM [`highway`](https://wiki.openstreetmap.org/wiki/Key:highway) tag. Initial defaults are set in `lua/graph.lua` (the `default_speed` table).
3. **`default_speeds.json` overrides**: During graph enhancement, the `SpeedAssigner` (`src/mjolnir/speed_assigner.h`) can override the Lua defaults with country/region-specific and density-specific (rural/suburban/urban) speeds. This is an external JSON file configured via `mjolnir.default_speeds_config`. See [OpenStreetMapSpeeds/schema](https://github.com/OpenStreetMapSpeeds/schema) for the format. Speeds can also be reassigned on existing tiles without a full rebuild using `valhalla_assign_speeds`.
4. **Road density adjustment**: The road density (the length of drivable roads in kilometers per square kilometer) at each node in the routing graph is estimated during Valhalla data import. The road density is used to determine if a road is in a rural or urban area. Roads in urban areas have their speed reduced if there is no `maxspeed` tag. In the future, this method may be replaced with a more accurate measure of rural versus urban regions, but density produces adequate results for now.

The `speed_type` attribute defines whether the assigned routing speed is from a speed limit or based on the highway tag.

## Traffic speeds

At runtime, `GraphTile::GetSpeed()` resolves the effective speed for an edge using the following priority:

1. **Live traffic** — real-time speeds from a separate traffic overlay (`traffic.tar`, configured via `mjolnir.traffic_extract`). Live traffic data is produced externally and stored as `TrafficSpeed` records in binary tile files that mirror the routing tile hierarchy. See `valhalla/baldr/traffictile.h` for the format.
2. **Predicted (historical) traffic** — per-edge speed profiles covering a full week in 5-minute buckets, DCT-compressed and stored inside routing tiles. Built by `valhalla_add_predicted_traffic` from CSV input. See `valhalla/baldr/predictedspeeds.h` and `docs/mjolnir/historical_traffic.md`.
3. **Constrained flow** — daytime (7am–7pm) average speed, stored on `DirectedEdge`. A lightweight alternative to full predicted profiles for edges where daytime speeds are relatively flat.
4. **Free-flow** — nighttime average speed, stored on `DirectedEdge`. Same idea — cheap to store since it lives directly in the edge rather than in a separate speed profile.
5. **Base speed** — the `DirectedEdge::speed()` value assigned during tile building (see above).

Which speed types are considered is controlled by `flow_mask` (derived from `costing_options.<costing>.speed_types` in the request) and whether time information is available. `DateTimeType` values: `0` = no time, `1` = current, `2` = depart_at, `3` = arrive_by, `4` = invariant (see `proto/options.proto`).
