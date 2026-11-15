# Valhalla locate service API reference

Valhalla's locate service is an open-source service that provides detailed information about streets and intersections close to an input point with some added matching criteria. This allows for tight integration in routing and navigation applications on web or mobile.

[View an interactive demo](https://valhalla.github.io/demos/locate)

The default logic for the OpenStreetMap tags, keys, and values used when routing are documented on an [OSM wiki page](https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Valhalla).

## Inputs of a locate request

The locate request run locally takes the form of `localhost:8002/locate?json={}`, where the JSON inputs inside the `{}` include location information, name and options for the costing model, and output options. Here is the JSON payload for an example request:

```json
{"verbose":true,"locations":[{"lat":42.358528,"lon":-83.271400},{"lat":42.996613,"lon":-78.749855}],"costing":"bicycle","costing_options":{"bicycle":{"bicycle_type":"road"}},"directions_options":{"units":"miles"},"id":"12abc3afe23984fe"}
```

This request provides detailed information about specific streets and intersections near the two input locations. Steets which do not have a surface type condusive to road bicycles will be excluded from the results. The units used for the lengths of the street sections will be displayed in miles.

There is an option to name your request. You can do this by adding and `id` key to your request. The `id` is returned with the response so a user could match to the corresponding request.

Because the locate service is designed to work in tandem with the route service API, the inputs for the two APIs are identical. For detailed options regarding specifying locations, costing models, costing options, directions options please see the relevant sections in the [routing API docs](route/api-reference.md#inputs-of-a-route)

### Other request options

| Options | Description |
| :------------------ | :----------- |
| `verbose` |  Can be set to `true` or `false`, but defaults to `false`. If set to `true` dense attribution of the given street or intersection will be returned. |
| `id` | Name your route request. If `id` is specified, the naming will be sent through to the response. |

## Outputs of a locate request

If a request has been named using the optional `id` key, then this `id` key and value will be echoed in the JSON response object.

The locate results are returned as a JSON array, with one JSON object per input location in the order specified. In `verbose` mode details about the streets and intersections including mode of travel access, names, way ids, shape, side of street as well as the closest point to the input along these features will be returned. If `verbose` was not enabled only the closest point, way id and side of street will be returned. A warnings array may also be included. This array may contain warning objects informing about deprecated request parameters, clamped values etc. |

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
    ],
    "warnings": [
      "hov costing is deprecated and will be turned into auto costing with hov2=true costing option"
    ]
  }
]
```


Here are some sample results with `verbose` set to `true` (the 2016-value `predicted_speeds` array is shortened):

```javascript
[
  {
    "input_lat": 50.916223,
    "input_lon": 6.915118,
    "edges": [
      {
        "access_restrictions": [],
        "live_speed": {},
        "correlated_lat": 50.916151,
        "correlated_lon": 6.915217,
        "side_of_street": "left",
        "bounding_circle": {
          "lat": 50.916182,
          "lon": 6.915271,
          "radius": 50
        },
        "linear_reference": "KwTqzCQ09XPEAQBhADczVHU=",
        "percent_along": 0.45856,
        "distance": 10.5,
        "shoulder": false,
        "heading": 48.7,
        "outbound_reach": 100,
        "inbound_reach": 100,
        "edge_info": {
          "way_id": 24326584,
          "bike_network": {
            "national": false,
            "regional": false,
            "local": false,
            "mountain": false
          },
          "names": [
            "Hollerather Straße"
          ],
          "shape": "kctb`Bwl`eLc`@{z@",
          "mean_elevation": 52,
          "speed_limit": 30,
          "osm_node_ids": [
            8307867174,
            8307867178
          ]
        },
        "edge": {
          "end_node": {
            "level": 2,
            "tile_id": 811467,
            "id": 44240,
            "value": 1484454563418
          },
          "speeds": {
            "default": 15,
            "type": "classified",
            "free_flow": 0,
            "constrained_flow": 0,
            "predicted": false
          },
          "access_restriction": false,
          "start_restriction": {
            "bicycle": false,
            "bus": false,
            "car": false,
            "emergency": false,
            "HOV": false,
            "pedestrian": false,
            "taxi": false,
            "truck": false,
            "wheelchair": false,
            "moped": false,
            "motorcycle": false
          },
          "end_restriction": {
            "bicycle": false,
            "bus": false,
            "car": false,
            "emergency": false,
            "HOV": false,
            "pedestrian": false,
            "taxi": false,
            "truck": false,
            "wheelchair": false,
            "moped": false,
            "motorcycle": false
          },
          "part_of_complex_restriction": false,
          "has_sign": false,
          "toll": false,
          "destination_only": false,
          "destination_only_hgv": false,
          "tunnel": false,
          "bridge": false,
          "round_about": false,
          "traffic_signal": false,
          "forward": true,
          "not_thru": false,
          "stop_sign": false,
          "yield_sign": false,
          "cycle_lane": "none",
          "bike_network": false,
          "truck_route": false,
          "lane_count": 1,
          "country_crossing": false,
          "sidewalk_left": true,
          "sidewalk_right": true,
          "sac_scale": "none",
          "deadend": false,
          "geo_attributes": {
            "length": 89,
            "weighted_grade": 0,
            "max_up_slope": 1,
            "max_down_slope": 0,
            "curvature": 0
          },
          "access": {
            "bicycle": true,
            "bus": true,
            "car": true,
            "emergency": false,
            "HOV": true,
            "pedestrian": true,
            "taxi": true,
            "truck": true,
            "wheelchair": true,
            "moped": true,
            "motorcycle": true
          },
          "classification": {
            "classification": "residential",
            "use": "road",
            "surface": "paved_smooth",
            "link": false,
            "internal": false
          }
        },
        "edge_id": {
          "level": 2,
          "tile_id": 811467,
          "id": 107643,
          "value": 3611906215514
        },
        "predicted_speeds": []
      }
    ],
    "nodes": []
  }
]
```

If no edges can be found for a location, `edges` and `nodes` are `null`. In `verbose` mode a `reason` string is added as well.

### Attribute Descriptions for Responses

Lengths and distances are in meters, speeds in kilometers per hour, angles in degrees, unless stated otherwise.

#### Location

| Key | Description |
| :------------------ | :----------- |
| `input_lat`, `input_lon` | The input coordinate. |
| `edges` | The edges the location correlated to. |
| `filtered_edges` | Only present if non-empty. Edges near the location that were dropped by the location's `heading` or `search_filter.level` filter. Same format as `edges`. |
| `nodes` | The end nodes of correlated edges, if the location snapped onto a node. Empty if it snapped to the middle of an edge. |
| `reason` | Verbose only. Why nothing was found for this location. |

#### Edge (verbose)

| Key | Description |
| :------------------ | :----------- |
| `incidents` | Only present if the edge carries live traffic incidents. One object per incident with `id`, `type`, `description`, `long_description`, `creation_time`, `start_time`, `end_time`, `impact`, `sub_type`, `sub_type_description`, `alertc_codes`, `lanes_blocked`, `num_lanes_blocked`, `clear_lanes`, `length`, `closed`, `congestion`, `iso_3166_1_alpha2`, `iso_3166_1_alpha3`. Fields without a value are omitted. |
| `access_restrictions` | Access restrictions on the edge. See [Access restriction](#access-restriction). |
| `live_speed` | Live traffic speed, empty if the edge has no valid live speed. See [Live speed](#live-speed). |
| `correlated_lat`, `correlated_lon` | The point on the edge the input was snapped to. |
| `side_of_street` | `left`, `right` or `neither`: on which side of the edge the input lies, relative to the edge's direction. |
| `bounding_circle` | `lat`, `lon` and `radius` of the circle enclosing the edge's shape, as stored in the tile. `radius` is 0 if the tileset has no bounding circles. |
| `linear_reference` | Base64 [OpenLR](https://www.openlr-association.com/) line location reference for the whole edge, with the positive offset set to `percent_along`. |
| `percent_along` | Position of the correlated point along the edge, from 0 to 1. |
| `distance` | Distance from the input to the correlated point. |
| `shoulder` | A boolean value indicating whether the edge has shoulder (`true` or `false`). If `true` then the data has been tagged to indicate a shoulder is present. `false`, however, could mean either that the data is tagged as having no shoulder present OR the data is untagged and we assume no shoulder (which may be an incorrect assumption) |
| `heading` | Heading of the edge at the correlated point, clockwise from north. |
| `outbound_reach`, `inbound_reach` | Number of edges reachable from / that can reach this edge, counted up to the location's `minimum_reachability`. |
| `edge_info` | Data shared by the edge and its opposing edge. See [Edge info](#edge-info). |
| `edge` | Attributes of the directed edge. See [Directed edge](#directed-edge). |
| `edge_id` | The edge's graph id. See [Graph id](#graph-id). |
| `timed_predicted_speed` | Only present if the location has a `date_time` and the edge has predicted speeds. The predicted speed at that time. |
| `predicted_speeds` | The edge's predicted speeds for the whole week in 5-minute buckets (2016 values), starting Sunday 00:00 local time. Empty if the edge has no predicted speeds. |

#### Edge (non-verbose)

| Key | Description |
| :------------------ | :----------- |
| `way_id` | OSM way id of the edge. |
| `correlated_lat`, `correlated_lon`, `side_of_street`, `percent_along` | As in verbose mode. |

#### Live speed

All keys are absent if the edge has no valid live speed. An edge is split into up to three subsegments.

| Key | Description |
| :------------------ | :----------- |
| `overall_speed` | Speed over the whole edge. |
| `speed_0`, `speed_1`, `speed_2` | Speed on each subsegment, `null` if unknown. |
| `congestion_0`, `congestion_1`, `congestion_2` | Congestion on each subsegment from 0 (free flow) to 1 (stopped), `null` if unknown. |
| `breakpoint_0`, `breakpoint_1` | Where the first and second subsegments end, as a fraction of the edge length. |

#### Edge info

| Key | Description |
| :------------------ | :----------- |
| `way_id` | OSM way id. |
| `bike_network` | Booleans `national`, `regional`, `local`, `mountain`: which bicycle route networks the way is part of. |
| `names` | Street names, refs etc. |
| `shape` | Edge geometry as an [encoded polyline](decoding.md) with 6 digits of precision, in the direction the edge info was stored (see `edge.forward`). |
| `mean_elevation` | Mean elevation in meters, `null` if there's no elevation data. |
| `speed_limit` | The edge's speed limit, `0` if unknown, `"unlimited"` if there's none. |
| `conditional_speed_limits` | Only present if tagged. The key describes the time window during which the speed limit applies, the value is the limit. |
| `levels` | Only present if tagged. An array containing the edge's levels as derived from the `level=*` tag. Values are either numeric, or another array containing two elements, which denote the start and end of a range (inclusive). |
| `osm_node_ids` | Only present if the tileset was built to store them. OSM node ids along the edge's shape. |

#### Directed edge

| Key | Description |
| :------------------ | :----------- |
| `end_node` | Graph id of the node the edge ends at. See [Graph id](#graph-id). |
| `speeds.default` | The base speed used when no traffic speed applies. |
| `speeds.type` | `tagged` if the default speed comes from a `maxspeed` tag, `classified` if it was derived from the road class. |
| `speeds.free_flow`, `speeds.constrained_flow` | Historical night-time and day-time speed, `0` if not set. |
| `speeds.predicted` | Whether the edge has predicted (historical) speed profiles. |
| `access_restriction` | Whether the edge has entries in `access_restrictions`. |
| `start_restriction`, `end_restriction` | Per mode: whether a complex (via-way) turn restriction starts / ends on this edge. |
| `part_of_complex_restriction` | Whether the edge is inside (a via-way of) a complex restriction. |
| `has_sign` | Whether the edge has exit signs or guidance signs. |
| `toll`, `tunnel`, `bridge`, `round_about` | As the names say. |
| `destination_only` | Access is only allowed to reach a destination on it (e.g. `access=destination`, private roads). |
| `destination_only_hgv` | As `destination_only`, but for trucks only. |
| `traffic_signal`, `stop_sign`, `yield_sign` | Whether there's a traffic signal / stop sign / yield sign at the end of the edge. |
| `forward` | Whether the edge runs in the same direction as the shape in `edge_info`. |
| `not_thru` | The edge leads into a region with no exit (no through traffic). |
| `cycle_lane` | `none`, `shared`, `dedicated` or `separated`. |
| `bike_network` | Whether the edge is part of any bicycle route network. |
| `truck_route` | Whether the edge is part of a designated truck route. |
| `lane_count` | Number of lanes in this direction. |
| `country_crossing` | Whether the edge crosses a country border. |
| `sidewalk_left`, `sidewalk_right` | Whether there's a sidewalk on the left / right side. |
| `sac_scale` | Hiking difficulty: `none`, `hiking`, `mountain hiking`, `demanding mountain hiking`, `alpine hiking`, `demanding alpine hiking`, `difficult alpine hiking`. |
| `deadend` | Whether the edge ends in a dead end. |
| `geo_attributes.length` | Edge length. |
| `geo_attributes.weighted_grade` | Length-weighted grade in percent, from -10 to 15. |
| `geo_attributes.max_up_slope`, `geo_attributes.max_down_slope` | Steepest up- and downhill slope in degrees. |
| `geo_attributes.curvature` | Curvature factor from 0 (straight) to 15. |
| `access` | Per mode: whether the mode may travel along this edge in its direction. |
| `classification.classification` | Road class: `motorway`, `trunk`, `primary`, `secondary`, `tertiary`, `unclassified`, `residential`, `service_other`. |
| `classification.use` | What the edge is used for, e.g. `road`, `ramp`, `turn_channel`, `driveway`, `footway`, `cycleway`, `ferry`, `rail`. |
| `classification.surface` | `paved_smooth`, `paved`, `paved_rough`, `compacted`, `dirt`, `gravel`, `path`, `impassable`. |
| `classification.link` | Whether the edge is a `*_link` road. |
| `classification.internal` | Whether the edge is internal to an intersection. |
| `hov_type` | Only present on HOV-only edges: `HOV-2` or `HOV-3`. |

#### Access restriction

| Key | Description |
| :------------------ | :----------- |
| `type` | `hazmat`, `max_height`, `max_width`, `max_length`, `max_weight`, `max_axle_load`, `timed_allowed`, `timed_denied`, `destination_allowed` or `max_axles`. |
| `edge_index` | Index of the edge inside its tile. |
| `bus`, `car`, `emergency`, `HOV`, `pedestrian`, `taxi`, `truck`, `wheelchair`, `moped`, `motorcycle` | Whether the restriction applies to this mode. |
| `except_destination` | Whether the restriction is lifted when using the edge is necessary to reach the destination. |
| `value` | For `max_*` dimension types: the limit in meters or metric tons. For `max_axles`: the number of axles. For `hazmat`: a boolean. For `timed_allowed`, `timed_denied` and `destination_allowed`: an object whose `time_domain` is the encoded time window. |

#### Node (verbose)

| Key | Description |
| :------------------ | :----------- |
| `lat`, `lon` | Node position. |
| `elevation` | Node elevation. |
| `edge_count` | Number of outbound edges from this node in its tile. |
| `access` | Object with a boolean access value per mode. |
| `tagged_access` | Whether the node's access was set by OSM tags instead of defaults. |
| `intersection_type` | `regular`, `false`, `dead-end` or `fork`. |
| `administrative` | `iso_3166-1`, `country`, `iso_3166-2`, `state` and `time_zone_name` (IANA name) of the node's location. |
| `density` | Graph ensity around the node, from 0 to 15. |
| `local_edge_count` | Number of edges across all hierarchy levels. |
| `drive_on_right` | Whether traffic drives on the right. |
| `mode_change` | Whether travel mode can change here. |
| `private_access` | Whether the node is tagged as private. |
| `traffic_signal` | Whether the node has a traffic signal. |
| `type` | `street_intersection`, `gate`, `bollard`, `toll_booth`, `toll_gantry`, `border_control`, `motor_way_junction`, `parking`, `bike_share`, `transit_egress`, `transit_station`, `multi_use_transit_platform`, `sump_buster`, `building_entrance` or `elevator`. |
| `transition_count` | Number of transitions to the same node on other hierarchy levels. |
| `named_intersection` | Whether the intersection has a name. |
| `stop_index` | Only present on transit nodes. Index of the transit stop in the tile. |
| `node_id` | The node's graph id. See [Graph id](#graph-id). |

#### Node (non-verbose)

| Key | Description |
| :------------------ | :----------- |
| `lat`, `lon` | Node position. |

#### Graph id

| Key | Description |
| :------------------ | :----------- |
| `level` | Hierarchy level: 0 (highways), 1 (arterials), 2 (local), 3 (transit). |
| `tile_id` | Tile index within the level. |
| `id` | Index of the edge or node within the tile. |
| `value` | The full 64-bit id, as used elsewhere in the API. |

### HTTP status codes and error messages

Because the locate service API is so tightly integrated with the route service API the two share the same list of response codes and error messages. Please review the full lists in the [routing service API documentation](route/api-reference.md#http-status-codes-and-conditions)
