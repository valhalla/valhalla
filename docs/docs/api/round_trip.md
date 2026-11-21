# Valhalla Round Trip Service API Reference

Valhalla lets users choose a start point, a desired distance and heading, and returns a round trip route that has approximately the desired length and routes the user out from the start and back to it. Its purpose is of recreational nature and allows users to plan round trips for hiking, running, cycling, and driving.

## Inputs of the Round Trip service 

A round trip request generally resembles a directions request (see the [turn-by-turn API reference](route/api-reference.md): it needs to specify a `costing`, optionally `costing_options` and other high level options such as `format`, as well as a single location, along with a `round_trip_options` object that lets the user shape the desired round trip using the following parameters:  


### Round Trip parameters 


| Parameter | Description |
| :-------- | :---------- |
| `distance` (required) | The desired length of the round trip in meters. It is capped by the server's `max_round_trip_distance` for the chosen costing, which is 200 km by default (130 km for `bicycle` and `bikeshare`, 100 km for `pedestrian`). |
| `heading` | The direction in degrees from north, clockwise, that the round trip should lead into. Values outside 0–360 are wrapped. The default is 0 (north). |
| `heading_tolerance` | The width in degrees of the sector around `heading` that the turning points of the round trip are searched in. A higher tolerance extends the search for intermediate locations to a wider direction but follows the `heading` less closely. A more narrow tolerance will produce a more narrow round trip, but reduces the chance of finding a good match. Must be between 10 and 180, the default is 45. |
| `distance_tolerance` | The accepted deviation from `distance` as a fraction of it. The service stops searching once it finds a round trip within the tolerance, so a larger value answers faster. If none is found within the tolerance, the closest one is returned. Must be between 0.02 and 0.5, the default is 0.1. |

The round trip runs clockwise: it goes out on the left side of `heading` and comes back on the right side. Apart from the first and last stretch near the start, the service avoids using the same road twice.

### Location parameters

A round trip request takes exactly one location, the start and end of the round trip. It supports the same parameters as a [route location](route/api-reference.md#locations).

### Costing parameters

The Round Trip service supports the `auto`, `bus`, `taxi`, `truck`, `motorcycle`, `motor_scooter`, `bicycle` and `pedestrian` costing models. Refer to the [route costing models](route/api-reference.md#costing-models) and [costing options](route/api-reference.md#costing-options) documentation for how to specify them.

For round trips, `uturn_penalty` defaults to 3600 seconds instead of 600.

### Other request options

[Directions options](route/api-reference.md#directions-options) such as `units`, `language` and `directions_type`, and the `id` and `format` [request options](route/api-reference.md#other-request-options) work as in a route request. The supported formats are `json`, `osrm`, `gpx` and `pbf`.

Here is an example of a 30 km bicycle round trip heading east:

```json
{
  "locations": [{"lat": 47.5596, "lon": 7.5886}],
  "costing": "bicycle",
  "round_trip_options": {
    "distance": 30000,
    "heading": 90,
    "heading_tolerance": 90
  }
}
```

## Outputs of the Round Trip service

The response has the same format as a [route response](route/api-reference.md#outputs-of-a-route) with a single leg. The trip's `locations` hold the start twice, as origin and as destination.

| Item | Description |
| :--- | :---------- |
| `warnings` (optional) | This array may contain warning objects, e.g. when `distance`, `heading_tolerance` or `distance_tolerance` were clamped to the allowed range. |

## Error checking

| Code | Condition |
| :--- | :-------- |
| 120 | No location was given. |
| 150 | More than one location was given. |
| 125 | The costing model doesn't support round trips. |
| 171 | The start couldn't be snapped to the road network. |
| 178 | `distance` is missing or not positive. |
| 442 | No round trip could be found, e.g. because the roads around the start don't allow one in the requested direction. A wider `heading_tolerance` can help. |

See the [HTTP return codes](route/api-reference.md#http-status-codes-and-conditions) for more on messages you might receive from the service.
