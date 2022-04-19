# Valhalla locate service API reference

Valhalla's locate service, is an open-source service that provides detailed information about streets and intersections close to an input point with some added matching criteria. This allows for tight integration in routing and navigation applications on web or mobile.

[View an interactive demo](http://valhalla.github.io/demos/locate)

The default logic for the OpenStreetMap tags, keys, and values used when routing are documented on an [OSM wiki page](http://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Valhalla).

## Inputs of a locate request

The locate request run locally takes the form of `localhost:8002/locate?json={}`, where the JSON inputs inside the `{}` include location information, name and options for the costing model, and output options. Here is the JSON payload for an example request:

```
{"verbose":true,"locations":[{"lat":42.358528,"lon":-83.271400},{"lat":42.996613,"lon":-78.749855}],"costing":"bicycle","costing_options":{"bicycle":{"bicycle_type":"road"}},"directions_options":{"units":"miles"},"id":"12abc3afe23984fe"}
```

This request provides detailed information about specific streets and intersections near the two input locations. Steets which do not have a surface type condusive to road bicycles will be excluded from the results. The units used for the lengths of the street sections will be displayed in miles.

There is an option to name your request. You can do this by adding and `id` key to your request. The `id` is returned with the response so a user could match to the corresponding request.

Because the locate service is designed to work in tandem with the route service API, the inputs for the two APIs are identical. For detailed options regarding specifiying locations, costing models, costing options, directions options please see the relevant sections in the [routing API docs](../turn-by-turn/api-reference.md#inputs-of-a-route)

### Other request options

| Options | Description |
| :------------------ | :----------- |
| `verbose` |  Can be set to `true` or `false`, but defaults to `false`. If set to `true` dense attribution of the given street or intersection will be returned. |
| `id` | Name your route request. If `id` is specified, the naming will be sent through to the response. |

## Outputs of a locate request

If a request has been named using the optional `id` key, then this `id` key and value will be echoed in the JSON response object.

The locate results are returned as a JSON array, with one JSON object per input location in the order specified. In `verbose` mode details about the streets and intersections includding mode of travel access, names, way ids, shape, side of street as well as the closest point to the input along these features will be returned. If `verbose` was not enabled only the closest point, way id and side of street will be returned.

Here are some sample results with `verbose` set to `false`:

```javascript
[
  {
    "input_lon": -76.495743,
    "input_lat": 40.310555,
    "nodes": [
      {
        "lat": 40.313206,
        "lon": -76.494987
      }
    ],
    "edges": [
      {
        "way_id": 12292268,
        "correlated_lat": 40.313206,
        "side_of_street": "neither",
        "percent_along": 0,
        "correlated_lon": -76.494987
      },
      {
        "way_id": 12292268,
        "correlated_lat": 40.313206,
        "side_of_street": "neither",
        "percent_along": 1,
        "correlated_lon": -76.494987
      }
    ]
  }
]
```


Here are some sample results with `verbose` set to `true`:

```javascript
[
  {
    "input_lon": -76.495743,
    "input_lat": 40.310555,
    "nodes": [
      {
        "traffic_signal": false,
        "type": "street_intersection",
        "lat": 40.313206,
        "node_id": {
          "id": 3080,
          "value": 103353655794,
          "tile_id": 750654,
          "level": 2
        },
        "access": {
          "wheelchair": true,
          "taxi": false,
          "HOV": true,
          "truck": true,
          "emergency": true,
          "pedestrian": true,
          "car": true,
          "bus": true,
          "bicycle": true
        },
        "lon": -76.494987,
        "edge_count": 1,
        "administrative": {
          "time_zone_posix": "EST-05EDT+01,M3.2.0/02:00,M11.1.0/02:00",
          "standard_time_zone_name": "EST",
          "iso_3166-1": "US",
          "daylight_savings_time_zone_name": "EDT",
          "country": "United States of America",
          "iso_3166-2": "PA",
          "state": "Pennsylvania"
        },
        "intersection_type": "dead-end",
        "density": 2,
        "local_edge_count": 1,
        "mode_change": false
      }
    ],
    "edges": [
      {
        "edge_id": {
          "id": 7660,
          "value": 257032954354,
          "tile_id": 750654,
          "level": 2
        },
        "edge_info": {
          "shape": "ivo{kAvg{{pCiMlJ{FfDgDm@c_AqxBeFO}}@`]",
          "way_id": 12292268,
          "names": [
            "Bomgardner Lane"
          ]
        },
        "edge": {
          "classification": {
            "link": false,
            "internal": false,
            "surface": "paved_smooth",
            "classification": "residential"
          },
          "end_node": {
            "id": 3081,
            "value": 103387210226,
            "tile_id": 750654,
            "level": 2
          },
          "speed": 30,
          "traffic_signal": false,
          "start_restriction": {
            "moped": false,
            "wheelchair": false,
            "taxi": false,
            "HOV": false,
            "truck": false,
            "emergency": false,
            "pedestrian": false,
            "car": false,
            "bus": false,
            "bicycle": false
          },
          "speed_limit": 0,
          "geo_attributes": {
            "weighted_grade": 1.67,
            "length": 388
          },
          "cycle_lane": "none",
          "access_restriction": false,
          "part_of_complex_restriction": false,
          "country_crossing": false,
          "has_exit_sign": false,
          "lane_count": 1,
          "speed_type": "classified",
          "drive_on_right": true,
          "destination_only": false,
          "seasonal": false,
          "tunnel": false,
          "bridge": false,
          "access": {
            "moped": true,
            "wheelchair": true,
            "taxi": false,
            "HOV": true,
            "truck": true,
            "emergency": false,
            "pedestrian": true,
            "car": true,
            "bus": true,
            "bicycle": true
          },
          "toll": false,
          "round_about": false,
          "bike_network": {
            "mountain": false,
            "local": false,
            "regional": false,
            "national": false
          },
          "end_restriction": {
            "moped": false,
            "wheelchair": false,
            "taxi": false,
            "HOV": false,
            "truck": false,
            "emergency": false,
            "pedestrian": false,
            "car": false,
            "bus": false,
            "bicycle": false
          },
          "unreachable": false,
          "forward": true,
          "not_thru": false,
          "truck_route": false,
          "use": "road"
        },
        "minimum_reachability": 51,
        "score": 899846.4,
        "traffic_segments": [],
        "percent_along": 0,
        "correlated_lon": -76.494987,
        "side_of_street": "neither",
        "correlated_lat": 40.313206
      },
      {
        "edge_id": {
          "id": 7661,
          "value": 257066508786,
          "tile_id": 750654,
          "level": 2
        },
        "edge_info": {
          "shape": "ivo{kAvg{{pCiMlJ{FfDgDm@c_AqxBeFO}}@`]",
          "way_id": 12292268,
          "names": [
            "Bomgardner Lane"
          ]
        },
        "edge": {
          "classification": {
            "link": false,
            "internal": false,
            "surface": "paved_smooth",
            "classification": "residential"
          },
          "end_node": {
            "id": 3080,
            "value": 103353655794,
            "tile_id": 750654,
            "level": 2
          },
          "speed": 30,
          "traffic_signal": false,
          "start_restriction": {
            "moped": false,
            "wheelchair": false,
            "taxi": false,
            "HOV": false,
            "truck": false,
            "emergency": false,
            "pedestrian": false,
            "car": false,
            "bus": false,
            "bicycle": false
          },
          "speed_limit": 0,
          "geo_attributes": {
            "weighted_grade": -1.67,
            "length": 388
          },
          "cycle_lane": "none",
          "access_restriction": false,
          "part_of_complex_restriction": false,
          "country_crossing": false,
          "has_exit_sign": false,
          "lane_count": 1,
          "speed_type": "classified",
          "drive_on_right": true,
          "destination_only": false,
          "seasonal": false,
          "tunnel": false,
          "bridge": false,
          "access": {
            "moped": true,
            "wheelchair": true,
            "taxi": false,
            "HOV": true,
            "truck": true,
            "emergency": false,
            "pedestrian": true,
            "car": true,
            "bus": true,
            "bicycle": true
          },
          "toll": false,
          "round_about": false,
          "bike_network": {
            "mountain": false,
            "local": false,
            "regional": false,
            "national": false
          },
          "end_restriction": {
            "moped": false,
            "wheelchair": false,
            "taxi": false,
            "HOV": false,
            "truck": false,
            "emergency": false,
            "pedestrian": false,
            "car": false,
            "bus": false,
            "bicycle": false
          },
          "unreachable": false,
          "forward": false,
          "not_thru": true,
          "truck_route": false,
          "use": "road"
        },
        "minimum_reachability": 51,
        "score": 899846.4,
        "traffic_segments": [],
        "percent_along": 1,
        "correlated_lon": -76.494987,
        "side_of_street": "neither",
        "correlated_lat": 40.313206
      }
    ]
  }
]
```

### Attribute Descriptions for Responses

TODO:

### HTTP status codes and error messages

Because the locate service API is so tightly integrated with the route service API the two share the same list of response codes and error messages. Please review the full lists in the [routing service API documentation](../turn-by-turn/api-reference.md#http-status-codes-and-conditions)
