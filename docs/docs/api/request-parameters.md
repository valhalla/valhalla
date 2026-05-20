# Valhalla API Request Parameters

Reference for all JSON request parameters parsed in `src/worker.cc` and `src/sif/`.
Endpoint abbreviations: **R**=route, **M**=sources_to_targets, **I**=isochrone, **TR**=trace_route, **TA**=trace_attributes, **H**=height, **OR**=optimized_route, **L**=locate, **E**=expansion, **S**=status, **T**=tile.

---

## Top-Level Parameters

| Parameter | Type | Default | Values / Range | Endpoints | Notes |
|---|---|---|---|---|---|
| `id` | string | — | — | all | Echoed back in response for client tracking |
| `jsonp` | string | — | — | all | JSONP callback wrapper; incompatible with `format: pbf` |
| `format` | string | `json` | `json`, `gpx`, `osrm`, `pbf` | all (endpoint-dependent) | `gpx` only for R/TR/OR; `pbf` not for H/L |
| `units` | string | `km` | `km`, `miles`, `mi` | all | Output unit for distances |
| `language` | string | `en-US` | BCP47 locale (see `locales/`) | R, TR, OR | Narrative language |
| `verbose` | bool | `true` (M), `false` (others) | — | all | Slim matrix output when `false` |
| `directions_type` | string | `instructions` | `none`, `maneuvers`, `instructions` | R, TR, OR | `narrative: false` is the deprecated alias for `none` |
| `shape_format` | string | `polyline6` | `polyline6`, `polyline5`, `geojson`, `no_shape` | all | `no_shape` is invalid for H; default is `no_shape` for M |
| `generalize` | float | — | ≥ 0 (meters) | R, TR, OR | Polyline generalization tolerance |
| `linear_references` | bool | `false` | — | R, TR, OR | Include base64-encoded OpenLR references per edge |
| `admin_crossings` | bool | `false` | — | R, TR, OR | Include country/state crossing info in response |
| `reverse` | bool | `false` | — | I | Compute reverse isochrone (inbound rather than outbound) |
| `costing` | string | required | `auto`, `bicycle`, `pedestrian`, `truck`, `bus`, `taxi`, `motor_scooter`, `motorcycle`, `multimodal`, `bikeshare`, `auto_pedestrian` | all (except L, S, T) | `auto_shorter`, `hov`, `auto_data_fix` are deprecated aliases |
| `costing_options` | object | — | keyed by costing name | all | See [Costing Options](#costing-options) |
| `recostings` | array | — | array of costing objects with required `name` field | R, TR, OR | Re-score a found path under additional costings |
| `prioritize_bidirectional` | bool | `false` | — | R | Prefer bidirectional A* even for `depart_at` requests |
| `reverse_time_tracking` | string | — | enum from proto | R | Strategy for reverse time-dependent routing |

### Date/Time

| Parameter | Type | Default | Values / Range | Endpoints | Notes |
|---|---|---|---|---|---|
| `date_time.type` | integer | — | `0`=current, `1`=depart_at, `2`=arrive_by, `3`=invariant | R, TR, OR, M | Required with `date_time.value` for types 1–3 |
| `date_time.value` | string | — | ISO 8601 datetime | R, TR, OR, M | Required when `type` is 1, 2, or 3 |

### Avoid / Exclude

| Parameter | Type | Default | Values / Range | Endpoints | Notes |
|---|---|---|---|---|---|
| `exclude_locations` | array\<Location\> | — | — | R, TR, OR, M, I | Alias: `avoid_locations` (deprecated) |
| `exclude_polygons` | array\<ring\> or GeoJSON FeatureCollection | — | — | R, TR, OR, M, I | Each ring is `[[lon,lat],…]`; alias: `avoid_polygons` |
| `linear_cost_factors` | array | — | — | R, TR | Each entry: `{shape: polyline5_str, factor: float}` or GeoJSON feature with `properties.factor` / `properties.ignore_access_restrictions` |

---

## Route (`/route`, `/optimized_route`)

| Parameter | Type | Default | Values / Range | Notes |
|---|---|---|---|---|
| `locations` | array\<Location\> | required | ≥ 2 | First and last always `break` type |
| `alternates` | integer | `0` | ≥ 0 | Max alternative routes; forced to 0 for > 2 locations |
| `guidance_views` | bool | `false` | — | Include guidance view images in response |
| `banner_instructions` | bool | `false` | — | OSRM serializer banner instructions |
| `voice_instructions` | bool | `false` | — | OSRM serializer voice instructions |
| `turn_lanes` | bool | `false` | — | Include turn lane info |
| `roundabout_exits` | bool | `true` | — | Include roundabout exit maneuvers |
| `elevation_interval` | float | `0` | [0, 1000] meters | `0` = no elevation; samples along the path |

---

## Matrix (`/sources_to_targets`)

| Parameter | Type | Default | Values / Range | Notes |
|---|---|---|---|---|
| `sources` | array\<Location\> | required | ≥ 1 | — |
| `targets` | array\<Location\> | required | ≥ 1 | — |
| `matrix_locations` | integer | unlimited | ≥ 1 | Early-exit target count; only valid for 1:many or many:1 |

---

## Isochrone (`/isochrone`)

| Parameter | Type | Default | Values / Range | Notes |
|---|---|---|---|---|
| `locations` | array\<Location\> | required | exactly 1 | — |
| `contours` | array\<Contour\> | required | ≥ 1 | Each: `{time: float (min), distance: float (km), color: string (hex)}` |
| `polygons` | bool | `false` | — | Return filled polygons instead of isolines |
| `denoise` | float | `1.0` | [0, 1] | Smoothing; 1 = no smoothing |
| `show_locations` | bool | `false` | — | Include input location in GeoJSON output |

---

## Map Matching (`/trace_route`, `/trace_attributes`)

| Parameter | Type | Default | Values / Range | Notes |
|---|---|---|---|---|
| `shape` / `trace` | array\<Location\> | — | ≥ 2 | GPS trace points (mutually exclusive with `encoded_polyline`) |
| `encoded_polyline` | string | — | — | polyline6 (or polyline5 with `shape_format: polyline5` for `/height`) |
| `shape_match` | string | `walk_or_snap` | `map_snap`, `edge_walk`, `walk_or_snap` | Matching strategy |
| `begin_time` | integer | `0` | ≥ 0 (seconds) | Epoch start time when using `durations` |
| `durations` | array\<float\> | — | length = shape points − 1 | Per-segment travel times (seconds) |
| `use_timestamps` | bool | `false` | — | Use per-point `time` field for elapsed time |
| `trace_options.gps_accuracy` | float | — | meters | GPS accuracy for map matching |
| `trace_options.search_radius` | float | — | meters | Candidate search radius per point |
| `trace_options.turn_penalty_factor` | float | — | ≥ 0 | Penalty for turns in matching |
| `trace_options.breakage_distance` | float | — | meters | Max gap before a new trace segment starts |
| `trace_options.interpolation_distance` | float | — | meters | Interpolation distance for matched path |

### Trace Attribute Filters (only `/trace_attributes`)

| Parameter | Type | Default | Values / Range | Notes |
|---|---|---|---|---|
| `filters.action` | string | — | `include`, `exclude` | — |
| `filters.attributes` | array\<string\> | — | attribute keys (e.g. `edge.length`, `node.elapsed_time`) | — |

---

## Height (`/height`)

| Parameter | Type | Default | Values / Range | Notes |
|---|---|---|---|---|
| `shape` | array\<Location\> | — | ≥ 1 | Mutually exclusive with `encoded_polyline` |
| `encoded_polyline` | string | — | — | polyline6 (default) or polyline5 (with `shape_format: polyline5`) |
| `range` | bool | `false` | — | Return cumulative distance alongside elevation |
| `height_precision` | integer | `0` | [0, 2] | Decimal places in elevation response |
| `resample_distance` | float | — | meters | Resample shape at this interval before querying elevation |

---

## Expansion (`/expansion`)

| Parameter | Type | Default | Values / Range | Notes |
|---|---|---|---|---|
| `locations` | array\<Location\> | required | ≥ 1 | — |
| `action` | string | required | `route`, `isochrone`, `sources_to_targets` | The algorithm to expand |
| `expansion_properties` | array\<string\> | — | `durations`, `costs`, `distances`, `statuses`, `edge_ids`, `pred_edge_ids`, `flow_sources`, `expansion_type`, `travel_mode` | Properties to include per expanded edge |
| `expansion_max_distance` | integer | — | meters | Stop expanding beyond this distance |
| `skip_opposites` | bool | `false` | — | Skip reverse-direction edges in output |
| `dedupe` | bool | `false` | — | Emit each edge only once |

---

## Tile (`/tile`)

| Parameter | Type | Default | Values / Range | Notes |
|---|---|---|---|---|
| `tile.z` | integer | required | [0, 30] | Zoom level |
| `tile.x` | integer | required | [0, 2^z) | Tile column |
| `tile.y` | integer | required | [0, 2^z) | Tile row |
| `tile_options.exclude_layers` | array\<string\> | — | `edges`, `nodes`, `shortcuts`, `access_restrictions` | MVT layers to omit |

---

## Location Object

Used in `locations`, `sources`, `targets`, `shape`, `trace`, `exclude_locations`.

| Field | Type | Default | Values / Range | Notes |
|---|---|---|---|---|
| `lat` | float | required | [-90, 90] | — |
| `lon` | float | required | [-180, 180] | — |
| `type` | string | `break` | `break`, `through`, `via`, `break_through` | `trace_attributes` forces `via`; `trace_route` defaults to `via` |
| `name` | string | — | — | Label, echoed in response |
| `street` | string | — | — | Street address hint |
| `date_time` | string | — | ISO 8601 | Per-location time override |
| `heading` | integer | — | [0, 360] degrees | Preferred approach heading |
| `heading_tolerance` | integer | — | degrees | Tolerance around `heading` |
| `preferred_layer` | integer | — | — | Preferred road network layer |
| `node_snap_tolerance` | float | — | meters | Snap-to-node tolerance |
| `minimum_reachability` | integer | — | ≥ 0 | Min reachable edges for candidate |
| `radius` | integer | — | meters | Candidate search radius |
| `accuracy` | integer | — | meters | GPS accuracy (trace use) |
| `time` | float | `-1` | seconds | Timestamp for trace points |
| `rank_candidates` | bool | `true` | — | Rank snap candidates by quality |
| `preferred_side` | string | — | `same`, `opposite`, `either` | Preferred side of road |
| `display_lat` / `display_lon` | float | — | — | Display position (differs from routing position) |
| `search_cutoff` | integer | — | meters | Hard cutoff for candidate search |
| `street_side_tolerance` | integer | — | meters | Tolerance for preferred side matching |
| `street_side_max_distance` | integer | — | meters | Max distance for side-of-road preference |
| `street_side_cutoff` | string | — | road class enum | Ignore side preference on roads above this class |
| `waiting` | float | `0` | seconds | Waiting time at intermediate stops (break/break_through only) |
| `search_filter.min_road_class` | string | `service_other` | road class enum | Lowest road class to snap to |
| `search_filter.max_road_class` | string | `motorway` | road class enum | Highest road class to snap to |
| `search_filter.exclude_tunnel` | bool | `false` | — | — |
| `search_filter.exclude_bridge` | bool | `false` | — | — |
| `search_filter.exclude_toll` | bool | `false` | — | — |
| `search_filter.exclude_ramp` | bool | `false` | — | — |
| `search_filter.exclude_ferry` | bool | `false` | — | — |
| `search_filter.exclude_closures` | bool | `true` | — | — |
| `search_filter.level` | float | max | — | Indoor routing level |

**Road class enum values** (low → high): `service_other`, `residential`, `unclassified`, `tertiary`, `secondary`, `primary`, `trunk`, `motorway`

---

## Costing Options

Set under `costing_options.<costing_name>` (e.g. `costing_options.auto`).

### Base Options (all costings)

Parsed in `src/sif/dynamiccost.cc`. All penalty/cost values are in **seconds**. `kMaxPenalty` = 43200 (12 hours).

| Parameter | Type | Default | Range | Notes |
|---|---|---|---|---|
| `name` | string | — | — | Required for `recostings` entries |
| `shortest` | bool | `false` | — | Minimize distance instead of time |
| `disable_hierarchy_pruning` | bool | `false` | — | Disable graph hierarchy culling (slow, for debugging) |
| `ignore_restrictions` | bool | `false` | — | Ignore all access restrictions |
| `ignore_oneways` | bool | `false` | — | Ignore one-way constraints |
| `ignore_access` | bool | `false` | — | Ignore access tags |
| `ignore_closures` | bool | `false` | — | Ignore live traffic closures |
| `destination_only_penalty` | float | `600` | [0, 43200] | Penalty to enter destination-only roads |
| `maneuver_penalty` | float | `5` | [0, 43200] | Penalty for turns onto a different road name |
| `alley_penalty` | float | `5` | [0, 43200] | Penalty to use an alley |
| `gate_cost` | float | `30` | [0, 43200] | Time cost to pass through a gate |
| `gate_penalty` | float | `300` | [0, 43200] | Penalty for roads with gates |
| `private_access_penalty` | float | `450` | [0, 43200] | Penalty for private roads |
| `country_crossing_cost` | float | `600` | [0, 43200] | Cost to cross a country border |
| `country_crossing_penalty` | float | `0` | [0, 43200] | Additional penalty for border crossings |
| `ferry_cost` | float | `300` | [0, 43200] | Transit cost for a ferry |
| `use_ferry` | float | `0.5` | [0, 1] | Preference for ferries (0=avoid, 1=prefer) |
| `rail_ferry_cost` | float | `300` | [0, 43200] | Transit cost for a rail ferry |
| `use_rail_ferry` | float | `0.4` | [0, 1] | Preference for rail ferries |
| `service_penalty` | float | `15` | [0, 43200] | Penalty for service roads (`75` for `auto`) |
| `service_factor` | float | `1.0` | [0.1, 100000] | Cost multiplier for service roads |
| `use_tracks` | float | `0.5` | [0, 1] | Preference for track roads (`0` for `auto`) |
| `use_living_streets` | float | `0.1` | [0, 1] | Preference for living streets |
| `use_lit` | float | `0` | [0, 1] | Preference for lit roads |
| `closure_factor` | float | `9.0` | [1, 10] | Cost multiplier for live-traffic-closed roads |
| `speed_penalty_factor` | float | `0.05` | [0, 1] | Penalty rate for exceeding speed limit |
| `fixed_speed` | integer | `0` | [0, 252] kph | Override edge speed; `0` = disabled |
| `flow_mask` | integer | — | bitmask | Flow data sources to use (advanced) |
| `height` | float | `1.6` | meters | Vehicle height restriction |
| `width` | float | `1.9` | meters | Vehicle width restriction |
| `length` | float | `2.7` | meters | Vehicle length restriction |
| `weight` | float | `0.8` | metric tons | Vehicle weight restriction |
| `toll_booth_cost` | float | `15` | [0, 43200] | Cost to pass a toll booth (motor vehicles only) |
| `toll_booth_penalty` | float | `0` | [0, 43200] | Penalty for toll roads (motor vehicles only) |

### `auto`, `bus`, `taxi`

Parsed in `src/sif/autocost.cc`.

| Parameter | Type | Default | Range | Notes |
|---|---|---|---|---|
| `alley_factor` | float | `1.0` | [0.1, 100000] | Cost multiplier for alleys |
| `use_highways` | float | `0.5` | [0, 1] | Preference for motorways/trunks |
| `use_tolls` | float | `0.5` | [0, 1] | Preference for toll roads |
| `use_distance` | float | `0` | [0, 1] | Blend toward distance optimization |
| `restriction_probability` | integer | `100` | [0, 100] | % chance to honour probabilistic restrictions |
| `top_speed` | integer | `140` | [10, 252] kph | Target max speed; edges above this get a penalty |
| `include_hov2` | bool | `false` | — | Allow HOV-2 lanes |
| `include_hov3` | bool | `false` | — | Allow HOV-3 lanes |
| `include_hot` | bool | `false` | — | Allow HOT (High-Occupancy Toll) lanes |

### `bicycle`

Parsed in `src/sif/bicyclecost.cc`.

| Parameter | Type | Default | Range | Notes |
|---|---|---|---|---|
| `transport_type` | string | `hybrid` | `road`, `cross`, `hybrid`, `mountain` | Bicycle type |
| `cycling_speed` | float | type-dependent | [5, max] kph | `road`=25, `cross`=20, `hybrid`=18, `mountain`=16 default |
| `use_roads` | float | `0.25` | [0, 1] | Preference for roads vs dedicated infrastructure |
| `use_hills` | float | `0.25` | [0, 1] | Tolerance for hills |
| `avoid_bad_surfaces` | float | `0.25` | [0, 1] | Avoidance of unpaved/rough surfaces |
| `bss_rent_cost` | float | `120` | [0, 43200] | Cost to rent a bike-share bike (seconds) |
| `bss_rent_penalty` | float | `0` | [0, 43200] | Penalty for bike-share stations |

### `pedestrian`

Parsed in `src/sif/pedestriancost.cc`. Some ranges differ by `transport_type`.

| Parameter | Type | Default | Range | Notes |
|---|---|---|---|---|
| `transport_type` | string | `foot` | `foot`, `wheelchair` | Affects defaults for speed, grade, step penalty |
| `walking_speed` | float | `5.1` (foot), `4.0` (wheelchair) | [0.5, 25] kph | — |
| `walkway_factor` | float | `1.0` | [0.1, 100000] | Cost multiplier for walkways/footpaths |
| `sidewalk_factor` | float | `1.0` | [0.1, 100000] | Cost multiplier for sidewalks |
| `alley_factor` | float | `1.0` | [0.1, 100000] | Cost multiplier for alleys |
| `driveway_factor` | float | `5.0` | [0.1, 100000] | Cost multiplier for driveways |
| `step_penalty` | float | `30` (foot), `600` (wheelchair) | [0, 43200] | Penalty for steps/stairs |
| `max_grade` | integer | `90` (foot), `12` (wheelchair) | [0, 90] % | Maximum road grade |
| `max_hiking_difficulty` | integer | `1` | [0, 6] | Max SAC scale (0=paved, 6=extreme alpine) |
| `mode_factor` | float | `1.5` | [0.1, 100000] | Cost multiplier when used in multimodal |
| `max_distance` | integer | `100000` (foot), `10000` (wheelchair) | meters | Maximum route distance |
| `transit_start_end_max_distance` | integer | `2415` | meters | Max walk to/from first/last transit stop |
| `transit_transfer_max_distance` | integer | `805` | meters | Max walk between transit stops |
| `use_hills` | float | `0.5` | [0, 1] | Tolerance for hills |
| `elevator_penalty` | float | `5` | [0, 43200] | Penalty for elevators |
| `bss_rent_cost` | float | `120` | [0, 43200] | Cost to rent a bike-share bike |
| `bss_rent_penalty` | float | `0` | [0, 43200] | Penalty for bike-share stations |

### `truck`

Parsed in `src/sif/truckcost.cc`. Inherits all base options with truck-specific dimension defaults.

| Parameter | Type | Default | Range | Notes |
|---|---|---|---|---|
| `hazmat` | bool | `false` | — | Carrying hazardous materials |
| `low_class_penalty` | float | `0` | [0, 43200] | Penalty for residential/service roads |
| `weight` | float | `21.77` | metric tons | Overrides base default |
| `axle_load` | float | `9.07` | [0, 40] metric tons | Per-axle load |
| `height` | float | `4.11` | meters | Overrides base default |
| `width` | float | `2.6` | meters | Overrides base default |
| `length` | float | `21.64` | meters | Overrides base default |
| `axle_count` | integer | `5` | [2, 20] | Number of axles |
| `use_highways` | float | `0.5` | [0, 1] | Preference for motorways/trunks |
| `use_tolls` | float | `0.5` | [0, 1] | Preference for toll roads |
| `top_speed` | float | `120` | [10, 252] kph | Target max speed |
| `hgv_no_access_penalty` | float | `43200` | [0, 43200] | Penalty for roads with `hgv=no`; 43200 = impassable |
| `use_truck_route` | float | `0` | [0, 1] | Preference for designated truck routes |

### `motorcycle`

Parsed in `src/sif/motorcyclecost.cc`.

| Parameter | Type | Default | Range | Notes |
|---|---|---|---|---|
| `use_highways` | float | `0.5` | [0, 1] | Preference for motorways/trunks |
| `use_tolls` | float | `0.5` | [0, 1] | Preference for toll roads |
| `use_trails` | float | `0` | [0, 1] | Preference for unpaved trails |
| `top_speed` | integer | `140` | [10, 140] kph | Target max speed |

### `motor_scooter`

Parsed in `src/sif/motorscootercost.cc`.

| Parameter | Type | Default | Range | Notes |
|---|---|---|---|---|
| `top_speed` | integer | `45` | [20, 252] kph | Target max speed |
| `use_hills` | float | `0.5` | [0, 1] | Tolerance for hills |
| `use_primary` | float | `0.5` | [0, 1] | Preference for primary roads |

### `multimodal` / `transit`

Parsed in `src/sif/transitcost.cc`. The pedestrian leg uses pedestrian costing options.

| Parameter | Type | Default | Range | Notes |
|---|---|---|---|---|
| `mode_factor` | float | `1.5` | [0.1, 100000] | Cost multiplier when switching from transit to walk |
| `wheelchair` | bool | `false` | — | Require wheelchair-accessible transit |
| `bicycle` | bool | `false` | — | Allow bicycle on transit |
| `use_bus` | float | `0.3` | [0, 1] | Preference for bus routes |
| `use_rail` | float | `0.6` | [0, 1] | Preference for rail routes |
| `use_transfers` | float | `0.3` | [0, 1] | Willingness to transfer |
| `transfer_cost` | float | `15` | [0, 43200] | Time cost per transfer |
| `transfer_penalty` | float | `300` | [0, 43200] | Additional penalty per transfer |
| `filters.stops.action` | string | — | `include`, `exclude` | — |
| `filters.stops.ids` | array\<string\> | — | — | Stop IDs to include/exclude |
| `filters.operators.action` | string | — | `include`, `exclude` | — |
| `filters.operators.ids` | array\<string\> | — | — | Operator IDs to include/exclude |
| `filters.routes.action` | string | — | `include`, `exclude` | — |
| `filters.routes.ids` | array\<string\> | — | — | Route IDs to include/exclude |
