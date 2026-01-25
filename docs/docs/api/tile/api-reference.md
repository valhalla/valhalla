# Valhalla tile service API reference [BETA]

Valhalla's `/tile` service provides a graph representation as [Mapbox Vector Tiles](https://docs.mapbox.com/data/tilesets/guides/vector-tiles-introduction/) (MVT). Currently the tiles contain 2 layers for edges and nodes with a lot of attributes (akin to [`verbose` /locate](../locate/api-reference.md) requests). It's under active development, hence BETA, however, we don't expect any breaking changes to happen, mostly more "Valhalla-like" filtering of the response to decrease tile size, adding `style.json`(s) or performance improvements.

[View an interactive demo](https://valhalla.github.io/demos/tile)

The default logic for the OpenStreetMap tags, keys, and values used when routing are documented on an [OSM wiki page](https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Valhalla).

## Request options

We support the usual GET & POST with the common "Slippy Map"/XYZ request pattern. However, we expect x/y/z to be wrapped in a `"tile"` object. Extra tile-specific options can be added with a `tile_options` object. Typically one uses some SDK/clients to request tiles like [Maplibre](https://maplibre.org/) or QGIS.

| Option | Description |
| :------------------ | :----------- |
| `tile.z` | The zoom level, max 30. Which zoom levels render which road classes depends on the `loki.service_defaults.mvt_min_zoom_road_class` server configuration. |
| `tile.x` | The "slippy map" X coordinate. |
| `tile.y` | The "slippy map" Y coordinate. |
| `tile_options.return_shortcuts` |  Whether the response contains shortcut edges. Default `false`. |
| `filters` |  By default, the tiles only contain a small subset of attributes. Use `filters` to include more attributes:<ul><li>`attributes`: an array of edge/node attributes to include/exclude, see below for a list</li><li>`action`: either `include` or `exclude`. If `include`, we'll add the provided attributes to the default attributes. If `exclude`, we'll remove the provided attributes from the full list.</li></ul> |
| `verbose` | If `true`, it'll enable _all_ attributes, regardless of `filters`. Default `false`. Note, that the service setting `service_limits.status.allow_verbose` applies here too. |

## Attribute filters

While we're re-using the same code as for `trace_attributes`, we don't support the full list for the tile endpoint and added some which are not implemented for `trace_attributes`. If not specified otherwise, the meaning of the attribute values is either trivial or available at https://valhalla.github.io/valhalla/api/map-matching/api-reference/#edge-items. `access` attributes are returning the numeric representation of a bit mask which needs to be decoded, see https://github.com/valhalla/valhalla/blob/c5151e19f65c3b498aa606a9cc9d6d274fba11bc/valhalla/baldr/graphconstants.h#L37-L47, e.g. a value of 1033 (`0b10000001001`) indicates `auto`, `truck` and `motorcycle` access etc.

Note that `forward`/`backward` refer to the direction drawn in OSM and can be replaced in `<direction>`:

```
// bidirectional attributes
edge.use  // see https://github.com/valhalla/demos/blob/aab1cd6d118703529c0f32e240271592b02e7f82/tile/index.html#L275-L311
edge.tunnel
edge.bridge
edge.roundabout
edge.is_shortcut
edge.leaves_tile
edge.length
edge.weighted_grade
edge.max_upward_grade
edge.max_downward_grade
edge.curvature
edge.destination_only
edge.destination_only_hgv
edge.indoor
edge.hov_type  // see https://github.com/valhalla/demos/blob/aab1cd6d118703529c0f32e240271592b02e7f82/tile/index.html#L275-L311
edge.cycle_lane  // see https://github.com/valhalla/demos/blob/aab1cd6d118703529c0f32e240271592b02e7f82/tile/index.html#L275-L311
edge.bicycle_network
edge.truck_route
edge.speed_type  // see https://github.com/valhalla/demos/blob/aab1cd6d118703529c0f32e240271592b02e7f82/tile/index.html#L275-L311
edge.country_crossing
edge.sac_scale  // see https://github.com/valhalla/demos/blob/aab1cd6d118703529c0f32e240271592b02e7f82/tile/index.html#L275-L311
edge.unpaved
edge.surface  // see https://github.com/valhalla/demos/blob/aab1cd6d118703529c0f32e240271592b02e7f82/tile/index.html#L275-L311
edge.ramp
edge.internal_intersection
edge.shoulder
edge.dismount
edge.use_sidepath
edge.density
edge.sidewalk_left
edge.sidewalk_right
edge.bss_connection
edge.lit
edge.not_thru
edge.part_of_complex_restriction
edge.osm_id
edge.speed_limit
edge.layer

// directional attributes
edge.speed_<direction>
edge.deadend_<direction>
edge.lanecount_<direction>
edge.truck_speed_<direction>
edge.traffic_signal_<direction>
edge.stop_sign_<direction>
edge.yield_sign_<direction>
edge.access_<direction>
edge.live_speed_<direction>

// node attributes
node.drive_on_right
node.elevation
node.tagged_access
node.private_access
node.cash_only_toll
node.mode_change_allowed
node.named_intersection
node.timezone
node.access
```

## Integration into MVT compatible clients/SDKs etc

Most often software supporting MVT lets you specify a URL pattern expecting placeholders like `{z}/{x}/{y}?<query_params>`. For consistency's sake, we use our common `/?json=<post_json>` notation. Sadly that inteferes with placeholder parsing and the `<post_json>` usually has to be URL encoded for use in Maplibre/QGIS et al, e.g. `http://localhost:8002/tile?json=%7B%22tile%22%3A%7B%22z%22%3A{z}%2C%22x%22%3A{x}%2C%22y%22%3A{y}%7D%7D`. Note how x/y/z `{}` placeholders aren't encoded.

See an example `style.json` [here](https://github.com/valhalla/valhalla/blob/master/docs/docs/api/tile/default_style.json).

## Error/status codes and messages

| Status Code | Status | Description |
| :--------- | :---------- | :---------- |
| 174 | Invalid tile coordinates | Either omitted one of x/y/z OR x or y are out of bounds for z. |
