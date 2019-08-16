# Map Matching service API reference

With the Mapbox Map Matching service, you can match coordinates, such as GPS locations, to roads and paths that have been mapped in OpenStreetMap. By doing this, you can turn a path into a route with narrative instructions and also get the attribute values from that matched line.

You can view an [interactive demo](http://valhalla.github.io/demos/map_matching/) or use [Mobility Explorer](https://github.com/transitland/mobility-explorer).

There are two separate Map Matching calls that perform different operations on an input set of latitude,longitude coordinates. The `trace_route` action returns the shape snapped to the road network and narrative directions, while `trace_attributes` returns detailed attribution along the portion of the route.

It is important to note that all service requests should be *POST* because `shape` or `encoded_polyline` can be fairly large.

## Trace route action

The `trace_route` action takes the costing mode and a list of latitude,longitude coordinates, for example, from a GPS trace, to turn them into a route with the shape snapped to the road network and a set of guidance directions. You might use this to take a GPS trace from a bike route into a set of narrative instructions so you can re-create your trip or share it with others.

By default a single trip leg is returned in a trace_route response. You can split the route response into multiple legs by setting `"type":"break"` on any of the input shape objects in the `shape` parameter of your query. The first and last locations should always have type break. Note that setting breaks is not supported for encoded_polyline input, and is only supported for `map_snap` mode of the trace_route endpoint.

## Trace attributes action

The `trace_attributes` action takes the costing mode and a GPS trace or latitude,longitude positions and returns detailed attribution along the portion of the route. This includes details for each section of road along the path, as well as any intersections along the path. Some of the use cases for `trace_attributes` include getting:

* just-in-time information for navigation. Getting attributes only for portions of the upcoming route can improve performance because returning full details along an entire route can create a very large payload. For example, regular route responses include shape and a set of maneuvers along each route leg. The maneuvers are a generalization of the path to simplify the description. Detailed attributes and localization of attributes along a maneuver would require significant additions to the route response. For long and even moderate length routes this can be wasteful, as the chances of re-routing along a long route are high.
* speed limits. Speed limits along a path are a good example of just-in-time information that can be used for navigation. A single maneuver in a route (US-1, for example) may have many different speed limits along the full length of the maneuver. The `trace_attributes` action allows speed limits along each road segment to be determined and associated to portions of a maneuver.
* way IDs. You can turn a GPS trace into a set of way IDs that match the trace.
* the current road. A map-matching call with a recent set of GPS locations can be useful to find information about the current road, even if not doing navigation or having a route loaded on device.

Note that the attributes that are returned are Valhalla routing attributes, not the base OSM tags or base data. Valhalla imports OSM tags and normalizes many of them to a standard set of values used for routing. The default logic for the OpenStreetMap tags, keys, and values used when routing are documented on an [OSM wiki page](http://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Valhalla). To get the base OSM tags along a path, you need to take the OSM way IDs that are returned as attributes along the path and query OSM directly through a process such as the [Overpass API](http://wiki.openstreetmap.org/wiki/Overpass_API).

## Inputs of the Map Matching service

### Shape-matching parameters

`shape_match` is an optional string input parameter. It allows some control of the matching algorithm based on the type of input.

| shape_match type | Description |
| :--------- | :----------- |
| `edge_walk` | Indicates an edge walking algorithm can be used. This algorithm requires nearly exact shape matching, so it should only be used when the shape is from a prior Valhalla route. |
| `map_snap` | Indicates that a map-matching algorithm should be used because the input shape might not closely match Valhalla edges. This algorithm is more expensive. |
| `walk_or_snap` | Also the default option. This will try edge walking and if this does not succeed, it will fall back and use map matching. |

### Costing models and other options

Valhalla Map Matching uses the `auto`, `auto_shorter`, `bicycle`, `bus`, and `pedestrian` costing models available in the Valhalla route service. Refer to the [route costing models](/turn-by-turn/api-reference.md#costing-models) and [costing options](/turn-by-turn/api-reference.md#costing-options) documentation for more on how to specify this input.

Costing for `multimodal` is not supported for map matching because it would be difficult to get favorable GPS traces.

You can also set `directions_options` to specify output units, language, and whether or not to return directions in a narrative form. Refer to the [route options](/turn-by-turn/api-reference.md#directions-options) documentation for examples.

`trace_route` has additional options that allow more flexibility in specifying timestamps (when using encoded polyline input for the trace) and for using timestamps when computing elapsed time along the matched path. These options are:

| Option | Description |
| :--------- | :---------- |
| `begin_time` | Begin timestamp for the trace. This is used along with the `durations` so that timestamps can be specified for a trace that is specified using an encoded polyline. |
| `durations` | List of durations (seconds) between each successive pair of input trace points. This allows trace points to be supplied as an encoded polyline and timestamps to be created by using this list of "delta" times along with the `begin_time` of the trace. |
| `use_timestamps` | A boolean value indicating whether the input timestamps or durations should be used when computing elapsed time at each edge along the matched path. If true, timestamps are used. If false (default), internal costing is applied to compute elapsed times. |
| `trace_options` | Additional options. |
| `trace_options.search_radius` | Search radius in meters associated with supplied trace points. |
| `trace_options.gps_accuracy` | GPS accuracy in meters associated with supplied trace points. |
| `trace_options.breakage_distance` | Breaking distance in meters between trace points. |
| `trace_options.interpolation_distance` | Interpolation distance in meters beyond which trace points are merged together. |

### Attribute filters (`trace_attributes` only)

The `trace_attributes` action allows you to apply filters to `include` or `exclude` specific attribute filter keys in your response. These filters are optional and can be added to the action string inside of the `filters` object.

If no filters are used, all attributes are enabled and returned in the `trace_attributes` response.

These are the available filter keys. Review their [descriptions](#outputs-of-trace_attributes) for more information.

```
// Edge filter Keys
edge.names
edge.length
edge.speed
edge.road_class
edge.begin_heading
edge.end_heading
edge.begin_shape_index
edge.end_shape_index
edge.traversability
edge.use
edge.toll
edge.unpaved
edge.tunnel
edge.bridge
edge.roundabout
edge.internal_intersection
edge.drive_on_right
edge.surface
edge.sign.exit_number
edge.sign.exit_branch
edge.sign.exit_toward
edge.sign.exit_name
edge.travel_mode
edge.vehicle_type
edge.pedestrian_type
edge.bicycle_type
edge.transit_type
edge.id
edge.way_id
edge.weighted_grade
edge.max_upward_grade
edge.max_downward_grade
edge.mean_elevation
edge.lane_count
edge.cycle_lane
edge.bicycle_network
edge.sidewalk
edge.density
edge.speed_limit
edge.truck_speed
edge.truck_route

// Node filter keys
node.intersecting_edge.begin_heading
node.intersecting_edge.from_edge_name_consistency
node.intersecting_edge.to_edge_name_consistency
node.intersecting_edge.driveability
node.intersecting_edge.cyclability
node.intersecting_edge.walkability
node.intersecting_edge.use
node.intersecting_edge.road_class
node.elapsed_time
node.admin_index
node.type
node.fork
node.time_zone

// Other filter keys
osm_changeset
shape
admin.country_code
admin.country_text
admin.state_code
admin.state_text
matched.point
matched.type
matched.edge_index
matched.begin_route_discontinuity
matched.end_route_discontinuity
matched.distance_along_edge
matched.distance_from_trace_point
```

## Outputs of the Map Matching service

### Outputs of `trace_route`

The outputs of the `trace_route` action are the same as the [outputs of a route](/turn-by-turn/api-reference.md#outputs-of-a-route) action.

### Outputs of `trace_attributes`

The `trace_attributes` results contains a list of edges and, optionally, the following items: `osm_changeset`, list of `admins`, `shape`, `matched_points`, and `units`.

| Result item | Description |
| :--------- | :---------- |
| `edges` | List of edges associated with input shape. See the list of [edge items](#edge-items) for details. |
| `osm_changeset` | Identifier of the OpenStreetMap base data version. |
| `admins` | List of the administrative codes and names. See the list of [admin items](#admin-items) for details. |
| `shape` | The [encoded polyline](../../decoding.md) of the matched path. |
| `matched_points` | List of match results when using the `map_snap` shape match algorithm. There is a one-to-one correspondence with the input set of latitude, longitude coordinates and this list of match results. See the list of [matched point items](#matched-point-items) for details. |
| `units` | The specified units with the request, in either kilometers or miles. |

#### Edge items

Each `edge` may include:

| Edge item | Description |
| :--------- | :---------- |
| `names` | List of names. |
| `length` | Edge length in the units specified. The default is kilometers. |
| `speed` | Edge speed in the units specified. The default is kilometers per hour. |
| `road_class` | Road class values:<ul><li>`motorway`</li><li>`trunk`</li><li>`primary`</li><li>`secondary`</li><li>`tertiary`</li><li>`unclassified`</li><li>`residential`</li><li>`service_other`</li></ul> |
| `begin_heading` | The direction at the beginning of an edge. The units are degrees from north in a clockwise direction. |
| `end_heading` | The direction at the end of an edge. The units are degrees from north in a clockwise direction.. |
| `begin_shape_index` | Index into the list of shape points for the start of the edge. |
| `end_shape_index` | Index into the list of shape points for the end of the edge. |
| `traversability` | Traversability values, if available:<ul><li>`forward`</li><li>`backward`</li><li>`both`</li></ul> |
| `use` | Use values: <ul><li>`tram`</li><li>`road`</li><li>`ramp`</li><li>`turn_channel`</li><li>`track`</li><li>`driveway`</li><li>`alley`</li><li>`parking_aisle`</li><li>`emergency_access`</li><li>`drive_through`</li><li>`culdesac`</li><li>`cycleway`</li><li>`mountain_bike`</li><li>`sidewalk`</li><li>`footway`</li><li>`steps`</li><li>`other`</li><li>`rail-ferry`</li><li>`ferry`</li><li>`rail`</li><li>`bus`</li><li>`rail_connection`</li><li>`bus_connnection`</li><li>`transit_connection`</li></ul> |
| `toll` | True if the edge has any toll. |
| `unpaved` | True if the edge is unpaved or rough pavement. |
| `tunnel` | True if the edge is a tunnel. |
| `bridge` | True if the edge is a bridge. |
| `roundabout` | True if the edge is a roundabout. |
| `internal_intersection` | True if the edge is an internal intersection. |
| `drive_on_right` | True if the flag is enabled for driving on the right side of the street. |
| `surface` | Surface values: <ul><li>`paved_smooth`</li><li>`paved`</li><li>`paved_rough`</li><li>`compacted`</li><li>`dirt`</li><li>`gravel`</li><li>`path`</li><li>`impassable`</li></ul> |
| `sign` | Contains the interchange guide information associated with this edge. See the list of [sign items](#sign-items) for details. |
| `travel_mode` | Travel mode values:<ul><li>`drive`</li><li>`pedestrian`</li><li>`bicycle`</li><li>`transit`</li></ul> |
| `vehicle_type` | Vehicle type values:<ul><li>`car`</li><li>`motorcycle`</li><li>`bus`</li><li>`tractor_trailer`</li></ul> |
| `pedestrian_type` | Pedestrian type values:<ul><li>`foot`</li><li>`wheelchair`</li><li>`segway`</li></ul> |
| `bicycle_type` | Bicycle type values:<ul><li>`road`</li><li>`cross`</li><li>`hybrid`</li><li>`mountain`</li></ul> |
| `transit_type` | Transit type values: <ul><li>`tram`</li><li>`metro`</li><li>`rail`</li><li>`bus`</li><li>`ferry`</li><li>`cable_car`</li><li>`gondola`</li><li>`funicular`</li></ul>|
| `id` | Identifier of an edge within the tiled, hierarchical graph. |
| `way_id` | Way identifier of the OpenStreetMap base data. |
| `weighted_grade` | The weighted grade factor. Valhalla manufactures a `weighted_grade` from elevation data. It is a measure used for hill avoidance in routing - sort of a relative energy use along an edge. But since an edge in Valhalla can possibly go up and down over several hills it might not equate to what most folks think of as grade. |
| `max_upward_grade` | The maximum upward slope. A value of 32768 indicates no elevation data is available for this edge. |
| `max_downward_grade` | The maximum downward slope. A value of 32768 indicates no elevation data is available for this edge. |
| `mean_elevation` | The mean or average elevation along the edge. Units are meters by default. If the units are specified as miles, then the mean elevation is returned in feet. A value of 32768 indicates no elevation data is available for this edge. |
| `lane_count` | The number of lanes for this edge. |
| `cycle_lane` | The type (if any) of bicycle lane along this edge. |
| `bicycle_network` | The bike network for this edge. |
| `sidewalk` | Sidewalk values:<ul><li>`left`</li><li>`right`</li><li>`both`</li></ul> |
| `density` | The relative density along the edge. |
| `speed_limit` | Edge speed limit in the units specified. The default is kilometers per hour. |
| `truck_speed` | Edge truck speed in the units specified. The default is kilometers per hour. |
| `truck_route` | True if edge is part of a truck network/route. |
| `end_node` | The node at the end of this edge. See the list of [end node items](#end-node-items) for details. |

#### Sign items

Each `sign` may include:

| Sign item | Description |
| :--------- | :---------- |
| `exit_number` | List of exit number elements. If an exit number element exists, it is typically just one value. Element example: `91B` |
| `exit_branch` | List of exit branch elements. An exit branch element is the subsequent road name or route number after the sign. Element example: `I 95 North` |
| `exit_toward` | List of exit toward elements. The exit toward element is the location where the road ahead goes; the location is typically a control city, but may also be a future road name or route number. Element example: `New York` |
| `exit_name` | List of exit name elements. The exit name element is the interchange identifier, although typically not used in the United States. Element example: `Gettysburg Pike` |

#### End node items

Each `end_node` may include:

| Node item | Description |
| :--------- | :---------- |
| `intersecting_edges` | List of intersecting edges at this node. See the list of [intersecting edge items](#intersecting-edge-items) for details. |
| `elapsed_time` | Elapsed time of the path to arrive at this node. |
| `admin_index` | Index value in the admin list. |
| `type` | Node type values: <ul><li>`street_intersection`</li><li>`gate`</li><li>`bollard`</li><li>`toll_booth`</li><li>`multi_use_transit_stop`</li><li>`bike_share`</li><li>`parking`</li><li>`motor_way_junction`</li><li>`border_control`</li></ul> |
| `fork` | True if this node is a fork. |
| `time_zone` | Time zone string for this node. |

#### Intersecting edge items

Each `intersecting_edge` may include:

| Intersecting edge item | Description |
| :--------- | :---------- |
| `begin_heading` | The direction at the beginning of this intersecting edge. The units are degrees from north in a clockwise direction. |
| `from_edge_name_consistency` | True if this intersecting edge at the end node has consistent names with the path `from edge`. |
| `to_edge_name_consistency` | True if this intersecting edge at the end node has consistent names with the path `to edge`. |
| `driveability` | Driveability values, if available:<ul><li>`forward`</li><li>`backward`</li><li>`both`</li></ul> |
| `cyclability` | Cyclability values, if available:<ul><li>`forward`</li><li>`backward`</li><li>`both`</li></ul> |
| `walkability` | Walkability values, if available:<ul><li>`forward`</li><li>`backward`</li><li>`both`</li></ul> |
| `use` | Use values: <ul><li>`tram`</li><li>`road`</li><li>`ramp`</li><li>`turn_channel`</li><li>`track`</li><li>`driveway`</li><li>`alley`</li><li>`parking_aisle`</li><li>`emergency_access`</li><li>`drive_through`</li><li>`culdesac`</li><li>`cycleway`</li><li>`mountain_bike`</li><li>`sidewalk`</li><li>`footway`</li><li>`steps`</li><li>`other`</li><li>`rail-ferry`</li><li>`ferry`</li><li>`rail`</li><li>`bus`</li><li>`rail_connection`</li><li>`bus_connnection`</li><li>`transit_connection`</li></ul> |
| `road_class` | Road class values:<ul><li>`motorway`</li><li>`trunk`</li><li>`primary`</li><li>`secondary`</li><li>`tertiary`</li><li>`unclassified`</li><li>`residential`</li><li>`service_other`</li></ul> |

#### Admin items

Each `admin` may include:

| Admin item | Description |
| :--------- | :---------- |
| `country_code` | Country [ISO 3166-1 alpha-2](https://en.wikipedia.org/wiki/ISO_3166-1_alpha-2) code. |
| `country_text` | Country name. |
| `state_code` | State code. |
| `state_text` | State name. |

#### Matched point items

Each `matched_point` may include:

| Matched point item | Description |
| :--------- | :---------- |
| `lat` | The latitude of the matched point. |
| `lon` | The longitude of the matched point. |
| `type` | Describes the type of this match result - possible values include:<ul><li>`unmatched`</li><li>`interpolated`</li><li>`matched`</li></ul> |
| `edge_index` | The index of the edge in the list of edges that this matched point is associated with. This value will not exist if this point was unmatched. |
| `begin_route_discontinuity` | The boolean value is true if this match result is the begin location of a route disconnect. This value will not exist if this is false. |
| `end_route_discontinuity` |  The boolean value is true if this match result is the end location of a route disconnect.  This value will not exist if this is false. |
| `distance_along_edge` | The distance along the associated edge for this matched point. For example, if the matched point is halfway along the edge then the value would be 0.5. This value will not exist if this point was unmatched. |
| `distance_from_trace_point` | The distance in meters from the trace point to the matched point. This value will not exist if this point was unmatched. |

## Get better results

Follow these guidelines to improve the Map Matching results.

* You should have good GPS accuracy, meaning that there are few obstructions affecting GPS signals. This can be difficult in urban areas.
* Make sure the trace point density is within the approximate range of one per second and one per 10 seconds. The greater the range between them, the more chance of inaccurate results.
* Have each trace represent one continuous path.
* Verify that there is a corresponding match with the OpenStreetMap network.

You can use certain parameters to tune the response. Unless otherwise noted, each of these options is specified within a root-level `trace_options` object.

* Use `turn_penalty_factor` to penalize turns from one road segment to next. For a pedestrian `trace_route`, you may see a back-and-forth motion along the streets of your path. Try increasing the turn penalty factor to 500 to smooth out jittering of points. Note that if GPS accuracy is already good, increasing this will have a negative affect on your results.
* Set the `gps_accuracy` to indicate the accuracy in meters.
* Apply a `search_radius` to specify the search radius (in meters) within which to search road candidates for each measurement. The maximum search radius is 100 meters. Note that performance may decrease with a higher search radius value.
* Add a `time` component to your GPS data to inform the map matching algorithm about when the point was measured. Providing a `time` attribute is not available for `encoded_polyline` data and can only be specified with GPS data provided by the `shape` attribute. `time` is specified in seconds and can be a UNIX epoch time or any increasing sequence:
```
{"shape":[{"lat":39.983841,"lon":-76.735741,"time":0},{"lat":39.983704,"lon":-76.735298,"time":2},{"lat":39.983578,"lon":-76.734848,"time":6},...]}
```


## Example Map Matching requests

### Example `trace_route` requests

The following are example JSON payloads for POST requests for the `trace_routes` action.

*`trace_route` with shape parameter*

```
{"shape":[{"lat":39.983841,"lon":-76.735741,"type":"break"},{"lat":39.983704,"lon":-76.735298,"type":"via"},{"lat":39.983578,"lon":-76.734848,"type":"via"},{"lat":39.983551,"lon":-76.734253,"type":"break"},{"lat":39.983555,"lon":-76.734116,"type":"via"},{"lat":39.983589,"lon":-76.733315,"type":"via"},{"lat":39.983719,"lon":-76.732445,"type":"via"},{"lat":39.983818,"lon":-76.731712,"type":"via"},{"lat":39.983776,"lon":-76.731506,"type":"via"},{"lat":39.983696,"lon":-76.731369,"type":"break"}],"costing":"auto","shape_match":"map_snap"}}
```

*`trace_route` with encoded polyline parameter*

```
{"encoded_polyline":"_grbgAh~{nhF?lBAzBFvBHxBEtBKdB?fB@dBZdBb@hBh@jBb@x@\\|@x@pB\\x@v@hBl@nBPbCXtBn@|@z@ZbAEbAa@~@q@z@QhA]pAUpAVhAPlAWtASpAAdA[dASdAQhAIlARjANnAZhAf@n@`A?lB^nCRbA\\xB`@vBf@tBTbCFbARzBZvBThBRnBNrBP`CHbCF`CNdCb@vBX`ARlAJfADhA@dAFdAP`AR`Ah@hBd@bBl@rBV|B?vB]tBCvBBhAF`CFnBXtAVxAVpAVtAb@|AZ`Bd@~BJfA@fAHdADhADhABjAGzAInAAjAB|BNbCR|BTjBZtB`@lBh@lB\\|Bl@rBXtBN`Al@g@t@?nAA~AKvACvAAlAMdAU`Ac@hAShAI`AJ`AIdAi@bAu@|@k@p@]p@a@bAc@z@g@~@Ot@Bz@f@X`BFtBXdCLbAf@zBh@fBb@xAb@nATjAKjAW`BI|AEpAHjAPdAAfAGdAFjAv@p@XlAVnA?~A?jAInAPtAVxAXnAf@tBDpBJpBXhBJfBDpAZ|Ax@pAz@h@~@lA|@bAnAd@hAj@tAR~AKxAc@xAShA]hAIdAAjA]~A[v@BhB?dBSv@Ct@CvAI~@Oz@Pv@dAz@lAj@~A^`B^|AXvAVpAXdBh@~Ap@fCh@hB\\zBN`Aj@xBFdA@jALbAPbAJdAHdAJbAHbAHfAJhALbA\\lBTvBAdC@bC@jCKjASbC?`CM`CDpB\\xAj@tB\\fA\\bAVfAJdAJbAXz@L|BO`AOdCDdA@~B\\z@l@v@l@v@l@r@j@t@b@x@b@r@z@jBVfCJdAJdANbCPfCF|BRhBS~BS`AYbAe@~BQdA","shape_match":"map_snap","costing":"pedestrian","directions_options":{"units":"miles"}}
```

*`trace_route` with additional trace options*

```
{"encoded_polyline":"{gmagAp~_nhF[_AZqAjAaB`AkB\\cAZeAPcAHiAKiCQaAUeAe@cA]qAQsACqAF}AIsAg@{@s@i@sAw@i@uANyAPsAv@cAZ_ALqA_@cA_Ai@q@w@[uAm@}AaAqAs@s@m@u@c@oA]mAMiAIkAYqBRyATsANsBKyBAiA?iAFsALuAPsAXaBGgCg@oBKiBf@iBGiBg@oBHqA?{Ai@cA@oAbAgAj@g@j@k@n@q@p@e@n@k@d@}@\\cAFgAIoBg@m@y@[_ASu@K{AHgBP{@@w@Jy@L}@JsAMoAe@kAq@kAk@gAg@y@a@m@g@q@mAo@e@gAc@sAA_AI_BFu@LiALaALaAHoAFiAF{A@yABkAKmAKcAMiBU}AYu@MaBUqBu@{As@iAaAaA_Ao@aAi@s@q@{@{@a@sAi@oAGeAD}@[YiAmA[u@BgBXaBHgAQ}@]s@S]x@_BbC}@lAw@dAs@dAk@dAm@z@s@n@y@p@{@`Ao@z@_@z@u@jA_Af@aANiAAcBPsAl@s@X}@T{@PoAPeAl@{@t@_AbAgAjBk@r@e@~AY~@U~@YhAi@lAo@t@kBb@u@NyAX{@k@MoAi@{@eAx@aArAmBt@_ALgAPiAXkAf@w@j@u@p@_A^aA\\aA`@eAt@eATcAGcAG}@AeAX_Aj@q@r@aAp@wAt@sARk@xA?pBAvAu@Ps@_Bk@i@y@\\s@f@o@j@u@f@{@RwAb@mAZaARkATqANyAImAK}@C_AFqADiAC_AUs@[}Ao@eBm@q@_@_Bg@iAm@kAaAc@s@c@w@y@{AWcAc@{Am@c@yBF{@AkAYmA_@uBD{@B{@Gu@ScBIu@NaB^_B`@sAC{Ac@}A]yAg@aBHuAJ_BAaAEgAA_AAy@@eBX_AFw@JcB^s@VuA`AuAl@kBl@_AFeAJcAB{ABwAj@o@l@q@b@u@Z{AXmAaAm@o@oAwAsAcAsAu@u@cAe@iAy@eAk@iAc@gAOgAWgBe@wAo@}Ai@sBY{@u@s@y@Y{@[aA]s@oB\\eCPoARwAXgBV{Ab@sAf@oAb@kAFsA?kACeADeA^aABeAO_Be@gB]}@eAaB_AoAw@aAm@o@q@cBp@eBz@a@fAi@|@o@LiBAyAIoAu@qBkAuAiAa@gAa@cB_@eAQm@c@o@{ANaBf@{@h@_BV}@XcBz@?{@s@o@y@u@}@mAaBw@qCm@mG}@uJLoBVkBVaBh@iB`@gBXgBZ_BZaBNgACwASyAUqAGuBGsBTcBRiB^eCTmBTaBNoAVaB@kAMiBZcAd@eAf@q@z@mAVgB[gAq@aAk@s@UkAG{B_@sAW}ASoBMwBSuB]gBq@eBs@eBaAuAaAeAo@{@i@}@e@{@g@{@m@m@y@_@y@[aBk@a@cBFsB[iAaAGoA`@IcC^yA`@}AVkBNwBBmBR_Cr@{CrAyD|@mCf@kBXeB\\eBl@aCf@cBc@sBg@qAUkBe@sBw@eBi@uA{@gAyAc@qAk@[wB@gARqANgAPiAJsAAeAa@mBKmB@uBBwB@oBZoBJkCJkANaCY}Ac@aBa@iBc@iBu@qBu@sBu@sB_AiBcA}AiAqAaAqAeAaA}@]iANgAj@}@dAcAhAeAfAyAfAo@`@k@f@o@f@s@d@o@f@k@h@o@`@s@V_Br@wAv@{An@uAt@qAj@wAl@iAv@gA~@qAbA{A`AcBz@o@^{A`AmAhAo@b@sAxA_@z@]x@a@t@qAxAk@j@qAfAm@b@m@b@kAfAuA~@eAhAcAbAiAr@qA|@gAdAcA`A{@bA_AhAgA|@mAr@qA|@mAlA","shape_match":"map_snap","costing":"pedestrian","trace_options":{"turn_penalty_factor":500},"directions_options":{"units":"miles"}}
```

### Example `trace_attributes` requests

The following are example JSON payloads for POST requests for the `trace_attributes` action.

*`trace_attributes` with shape parameter*

```
{"shape":[{"lat":39.983841,"lon":-76.735741},{"lat":39.983704,"lon":-76.735298},{"lat":39.983578,"lon":-76.734848},{"lat":39.983551,"lon":-76.734253},{"lat":39.983555,"lon":-76.734116},{"lat":39.983589,"lon":-76.733315},{"lat":39.983719,"lon":-76.732445},{"lat":39.983818,"lon":-76.731712},{"lat":39.983776,"lon":-76.731506},{"lat":39.983696,"lon":-76.731369}],"costing":"auto","shape_match":"walk_or_snap","filters":{"attributes":["edge.names","edge.id", "edge.weighted_grade","edge.speed"],"action":"include"}}
```

*`trace_attributes` with encoded polyline parameter*

```
{"encoded_polyline":"_grbgAh~{nhF?lBAzBFvBHxBEtBKdB?fB@dBZdBb@hBh@jBb@x@\\|@x@pB\\x@v@hBl@nBPbCXtBn@|@z@ZbAEbAa@~@q@z@QhA]pAUpAVhAPlAWtASpAAdA[dASdAQhAIlARjANnAZhAf@n@`A?lB^nCRbA\\xB`@vBf@tBTbCFbARzBZvBThBRnBNrBP`CHbCF`CNdCb@vBX`ARlAJfADhA@dAFdAP`AR`Ah@hBd@bBl@rBV|B?vB]tBCvBBhAF`CFnBXtAVxAVpAVtAb@|AZ`Bd@~BJfA@fAHdADhADhABjAGzAInAAjAB|BNbCR|BTjBZtB`@lBh@lB\\|Bl@rBXtBN`Al@g@t@?nAA~AKvACvAAlAMdAU`Ac@hAShAI`AJ`AIdAi@bAu@|@k@p@]p@a@bAc@z@g@~@Ot@Bz@f@X`BFtBXdCLbAf@zBh@fBb@xAb@nATjAKjAW`BI|AEpAHjAPdAAfAGdAFjAv@p@XlAVnA?~A?jAInAPtAVxAXnAf@tBDpBJpBXhBJfBDpAZ|Ax@pAz@h@~@lA|@bAnAd@hAj@tAR~AKxAc@xAShA]hAIdAAjA]~A[v@BhB?dBSv@Ct@CvAI~@Oz@Pv@dAz@lAj@~A^`B^|AXvAVpAXdBh@~Ap@fCh@hB\\zBN`Aj@xBFdA@jALbAPbAJdAHdAJbAHbAHfAJhALbA\\lBTvBAdC@bC@jCKjASbC?`CM`CDpB\\xAj@tB\\fA\\bAVfAJdAJbAXz@L|BO`AOdCDdA@~B\\z@l@v@l@v@l@r@j@t@b@x@b@r@z@jBVfCJdAJdANbCPfCF|BRhBS~BS`AYbAe@~BQdA","shape_match":"map_snap","costing":"pedestrian","directions_options":{"units":"miles"}}
```

*`trace_attributes` with `include` attribute filter*

```
{"shape":[{"lat":39.983841,"lon":-76.735741},{"lat":39.983704,"lon":-76.735298},{"lat":39.983578,"lon":-76.734848},{"lat":39.983551,"lon":-76.734253},{"lat":39.983555,"lon":-76.734116},{"lat":39.983589,"lon":-76.733315},{"lat":39.983719,"lon":-76.732445},{"lat":39.983818,"lon":-76.731712},{"lat":39.983776,"lon":-76.731506},{"lat":39.983696,"lon":-76.731369}],"costing":"auto","shape_match":"walk_or_snap","filters":{"attributes":["edge.names","edge.id", "edge.weighted_grade","edge.speed"],"action":"include"}}
```

*`trace_attributes` with `exclude` attribute filter*

```
{"shape":[{"lat":39.983841,"lon":-76.735741},{"lat":39.983704,"lon":-76.735298},{"lat":39.983578,"lon":-76.734848},{"lat":39.983551,"lon":-76.734253},{"lat":39.983555,"lon":-76.734116},{"lat":39.983589,"lon":-76.733315},{"lat":39.983719,"lon":-76.732445},{"lat":39.983818,"lon":-76.731712},{"lat":39.983776,"lon":-76.731506},{"lat":39.983696,"lon":-76.731369}],"costing":"auto","shape_match":"walk_or_snap","filters":{"attributes":["edge.names","edge.begin_shape_index","edge.end_shape_index","shape"],"action":"exclude"}}
```
*If you would like to visualize the map matched points that correlate to specified input locations - use the following filter*
```
"filters":{"attributes":["edge.id","matched.point","matched.type","matched.edge_index","matched.begin_route_discontinuity","matched.end_route_discontinuity","matched.distance_along_edge"],"action":"include"}
```
