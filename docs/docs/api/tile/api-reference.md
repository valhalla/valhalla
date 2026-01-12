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
| `filters` |  By default, the tiles only contain a small subset of attributes. Use `filters` to include more attributes:<ul><li>`attributes`: an array of attributes to include/exclude, see an exhaustive list in the [map-matching docs](../map-matching/api-reference.md#attribute-filters-trace_attributes-only)</li><li>`action`: either `include` or `exclude`. If `include`, we'll add the provided attributes to the default attributes. If `exclude`, we'll remove the provided attributes from the full list.</li></ul> |
| `verbose` | If `true`, it'll enable _all_ attributes, regardless of `filters`. Default `false`. Note, that the service setting `service_limits.status.allow_verbose` applies here too. |

## Integration into MVT compatible clients/SDKs etc

Most often software supporting MVT lets you specify a URL pattern expecting placeholders like `{z}/{x}/{y}?<query_params>`. For consistency's sake, we use our common `/?json=<post_json>` notation. Sadly that inteferes with placeholder parsing and the `<post_json>` usually has to be URL encoded for use in Maplibre/QGIS et al, e.g. `http://localhost:8002/tile?json=%7B%22tile%22%3A%7B%22z%22%3A{z}%2C%22x%22%3A{x}%2C%22y%22%3A{y}%7D%7D`. Note how x/y/z `{}` placeholders aren't encoded.

See an example `style.json` [here](https://github.com/valhalla/valhalla/blob/master/docs/docs/api/tile/default_style.json).

### Error/status codes and messages

| Status Code | Status | Description |
| :--------- | :---------- | :---------- |
| 174 | Invalid tile coordinates | Either omitted one of x/y/z OR x or y are out of bounds for z. |
