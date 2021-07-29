# Status service API reference

By default the `/status` endpoint will simply return a HTTP status code of 200 with an empty JSON object, acting as a health endpoint for the HTTP API. 

However, if `"verbose": true` is passed as a request parameter it will return additional information about the loaded tileset. **Note** that gathering this additional information can be computationally expensive, hence the `verbose` flag can be disallowed in the configuration JSON (`service_limits.status.allow_verbose`, default `false`).

## Outputs of the Status service

If `"verbose": true` is passed as a parameter, the service will output the following response:

| Response key       | Type   | Description  |
| :----------------- | :----- | :----------- |
| `version`          | string | The current Valhalla version, e.g. `3.1.3`. |
| `has_tiles`        | bool   | Whether a valid tileset is currently loaded. |
| `has_admins`       | bool   | Whether the current tileset was built using the admin database. |
| `has_timezones`    | bool   | Whether the current tileset was built using the timezone database. |
| `has_live_traffic` | bool   | Whether live traffic tiles are currently available. |
| `bbox`             | object | GeoJSON of the tileset extent. |
