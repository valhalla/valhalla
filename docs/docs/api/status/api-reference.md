# Status service API reference

By default the `/status` endpoint will return a HTTP status code of 200 with `version` and `tileset_last_modified` (as UNIX timestamp) info, which can also be used as a health endpoint for the HTTP API.

However, if `"verbose": true` is passed as a request parameter it will return additional information about the loaded tileset. **Note** that gathering this additional information can be computationally expensive, hence the `verbose` flag can be disallowed in the configuration JSON (`service_limits.status.allow_verbose`, default `false`).

## Outputs of the Status service

If `"verbose": true` is passed as a parameter, the service will output the following response:

| Response key       | Type    | Description  |
| :----------------- | :-----  | :----------- |
| `version`          | string  | The current Valhalla version, e.g. `3.1.4`. |
| `tileset_last_modified`      | integer | The time the tile_extract or tile_dir were last modified as UNIX timestamp, e.g. 1634903519. |
| `has_tiles`        | bool    | Whether a valid tileset is currently loaded. |
| `has_admins`       | bool    | Whether the current tileset was built using the admin database. |
| `has_timezones`    | bool    | Whether the current tileset was built using the timezone database. |
| `has_live_traffic` | bool    | Whether live traffic tiles are currently available. |
| `bbox`             | object  | GeoJSON of the tileset extent. |
| `warnings` (optional) | array | This array may contain warning objects informing about deprecated request parameters, clamped values etc. | 
