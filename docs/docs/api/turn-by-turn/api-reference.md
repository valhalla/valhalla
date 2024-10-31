# Valhalla routing service API reference

Valhalla's routing service (a.k.a. turn-by-turn), is an open-source routing service that lets you integrate routing and navigation into a web or mobile application.

[View an interactive demo](http://valhalla.github.io/demos/routing)

The default logic for the OpenStreetMap tags, keys, and values used when routing are documented on an [OSM wiki page](http://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Valhalla).

## Inputs of a route

The route request run locally takes the form of `localhost:8002/route?json={}`, where the JSON inputs inside the `{}` include location information, name and options for the costing model, and output options. Here is the JSON payload for an example request:

```json
{"locations":[{"lat":42.358528,"lon":-83.271400,"street":"Appleton"},{"lat":42.996613,"lon":-78.749855,"street":"Ranch Trail"}],"costing":"auto","costing_options":{"auto":{"country_crossing_penalty":2000.0}},"units":"miles","id":"my_work_route"}
```

This request provides automobile routing between the Detroit, Michigan area and Buffalo, New York, with an optional street name parameter to improve navigation at the start and end points. It attempts to avoid routing north through Canada by adding a penalty for crossing international borders. The resulting route is displayed in miles.

There is an option to name your route request. You can do this by appending the following to your request `&id=`. The `id` is returned with the response so a user could match to the corresponding request.

### Locations

You specify locations as an ordered list of two or more locations within a JSON array. Locations are visited in the order specified.

A location must include a latitude and longitude in decimal degrees. The coordinates can come from many input sources, such as a GPS location, a point or a click on a map, a geocoding service, and so on. Note that the Valhalla cannot search for names or addresses or perform geocoding or reverse geocoding. External search services, such as [Mapbox Geocoding](https://www.mapbox.com/api-documentation/#geocoding), can be used to find places and geocode addresses, which must be converted to coordinates for input.

To build a route, you need to specify two `break` locations. In addition, you can include `through`, `via` or `break_through` locations to influence the route path.

| Location parameters | Description |
| :--------- | :----------- |
| `lat` | Latitude of the location in degrees. This is assumed to be both the routing location and the display location if no `display_lat` and `display_lon` are provided. |
| `lon` | Longitude of the location in degrees. This is assumed to be both the routing location and the display location if no `display_lat` and `display_lon` are provided. |
| `type` | Type of location, either `break`, `through`, `via` or `break_through`. Each type controls two characteristics: whether or not to allow a u-turn at the location and whether or not to generate guidance/legs at the location. A `break` is a location at which we allows u-turns and generate legs and arrival/departure maneuvers. A `through` location is a location at which we neither allow u-turns nor generate legs or arrival/departure maneuvers. A `via` location is a location at which we allow u-turns but do not generate legs or arrival/departure maneuvers. A `break_through` location is a location at which we do not allow u-turns but do generate legs and arrival/departure maneuvers. If no type is provided, the type is assumed to be a `break`. The types of the first and last locations are ignored and are treated as `break`s. |
| `heading` | (optional) Preferred direction of travel for the start from the location. This can be useful for mobile routing where a vehicle is traveling in a specific direction along a road, and the route should start in that direction. The `heading` is indicated in degrees from north in a clockwise direction, where north is 0°, east is 90°, south is 180°, and west is 270°. |
| `heading_tolerance` | (optional) How close in degrees a given street's angle must be in order for it to be considered as in the same direction of the `heading` parameter. The default value is 60 degrees. |
| `street` | (optional) Street name. The street name may be used to assist finding the correct routing location at the specified latitude, longitude. This is not currently implemented. |
| `way_id` | (optional) OpenStreetMap identification number for a polyline [way](http://wiki.openstreetmap.org/wiki/Way). The way ID may be used to assist finding the correct routing location at the specified latitude, longitude. This is not currently implemented. |
| `minimum_reachability` | Minimum number of nodes (intersections) reachable for a given edge (road between intersections) to consider that edge as belonging to a connected region. When correlating this location to the route network, try to find candidates who are reachable from this many or more nodes (intersections). If a given candidate edge reaches less than this number of nodes its considered to be a disconnected island and we'll search for more candidates until we find at least one that isn't considered a disconnected island. If this value is larger than the configured service limit it will be clamped to that limit. The default is a minimum of 50 reachable nodes. |
| `radius` | The number of meters about this input location within which edges (roads between intersections) will be considered as candidates for said location. When correlating this location to the route network, try to only return results within this distance (meters) from this location. If there are no candidates within this distance it will return the closest candidate within reason. If this value is larger than the configured service limit it will be clamped to that limit. The default is 0 meters. |
| `rank_candidates` | Whether or not to rank the edge candidates for this location. The ranking is used as a penalty within the routing algorithm so that some edges will be penalized more heavily than others. If `true` candidates will be ranked according to their distance from the input and various other attributes. If `false` the candidates will all be treated as equal which should lead to routes that are just the most optimal path with emphasis about which edges were selected. |
| `preferred_side` | If the location is not offset from the road centerline or is closest to an intersection this option has no effect. Otherwise the determined side of street is used to determine whether or not the location should be visited from the `same`, `opposite` or `either` side of the road with respect to the side of the road the given locale drives on. In Germany (driving on the right side of the road), passing a value of `same` will only allow you to leave from or arrive at a location such that the location will be on your right. In Australia (driving on the left side of the road), passing a value of `same` will force the location to be on your left. A value of `opposite` will enforce arriving/departing from a location on the opposite side of the road from that which you would be driving on while a value of `either` will make no attempt limit the side of street that is available for the route. |
| `display_lat` | Latitude of the map location in degrees. If provided the `lat` and `lon` parameters will be treated as the routing location and the `display_lat` and `display_lon` will be used to determine the side of street. Both `display_lat` and `display_lon` must be provided and valid to achieve the desired effect. |
| `display_lon` | Longitude of the map location in degrees. If provided the `lat` and `lon` parameters will be treated as the routing location and the `display_lat` and `display_lon` will be used to determine the side of street. Both `display_lat` and `display_lon` must be provided and valid to achieve the desired effect. |
| `search_cutoff` | The cutoff at which we will assume the input is too far away from civilisation to be worth correlating to the nearest graph elements. The default is 35 km. |
| `node_snap_tolerance` | During edge correlation this is the tolerance used to determine whether or not to snap to the intersection rather than along the street, if the snap location is within this distance from the intersection the intersection is used instead. The default is 5 meters. |
| `street_side_tolerance` | If your input coordinate is less than this tolerance away from the edge centerline then we set your side of street to none otherwise your side of street will be left or right depending on direction of travel. The default is 5 meters. |
| `street_side_max_distance` | The max distance in meters that the input coordinates or display ll can be from the edge centerline for them to be used for determining the side of street. Beyond this distance the side of street is set to none. The default is 1000 meters. |
| `street_side_cutoff` | Disables the `preferred_side` when set to `same` or `opposite` if the edge has a road class less than that provided by `street_side_cutoff`. The road class must be one of the following strings: motorway, trunk, primary, secondary, tertiary, unclassified, residential, service_other.  The default value is `service_other` so that `preferred_side` will not be disabled for any edges. |
| `search_filter` | A set of optional filters to exclude candidate edges based on their attribution. The following exclusion filters are supported: <ul><li>`exclude_tunnel` (boolean, defaults to `false`): whether to exclude roads marked as tunnels</li><li>`exclude_bridge` (boolean, defaults to `false`): whether to exclude roads marked as bridges</li><li>`exclude_toll` (boolean, defaults to `false`): whether to exclude toll</li><li>`exclude_ferry` (boolean, defaults to `false`): whether to exclude ferry</li><li>`exclude_ramp` (boolean, defaults to `false`): whether to exclude link roads marked as ramps, note that some turn channels are also marked as ramps</li><li>`exclude_closures` (boolean, defaults to `true`): whether to exclude roads considered closed due to live traffic closure. <br>**Note:** This option cannot be set if `costing_options.<costing>.ignore_closures` is also specified. An error is returned if both options are specified. <br>**Note 2:** Ignoring closures at destination and source locations does NOT work for date_time type `0/1` & `2` respectively</li><li>`min_road_class` (string, defaults to `"service_other"`): lowest road class allowed</li><li>`max_road_class` (string, defaults to `"motorway"`): highest road class allowed</li><li>**BETA** `level` (float): if specified, will only consider edges that are on or traverse the passed floor level. It will set `search_cutoff` to a default value of 300 meters if no cutoff value is passed. Additionally, if a `search_cutoff` is passed, it will be clamped to 1000 meters.</li></ul>Road classes from highest to lowest are: motorway, trunk, primary, secondary, tertiary, unclassified, residential, service_other. |
| `preferred_layer` | The layer on which edges should be considered. If provided, edges whose layer does not match the provided value will be discarded from the candidate search. |

Optionally, you can include the following location information without impacting the routing. This information is carried through the request and returned as a convenience.

* `name` = Location or business name. The name may be used in the route narration directions, such as "You have arrived at _&lt;business name&gt;_.")
* `city` = City name.
* `state` = State name.
* `postal_code` = Postal code.
* `country` = Country name.
* `phone` = Telephone number.
* `url` = URL for the place or location.
* `waiting`: The waiting time in seconds at this location. E.g. when the route describes a pizza delivery tour, each location has a service time, which can be respected by setting `waiting` on the location, then the departure will be delayed by this amount in seconds. Only works for `break` or `break_through` types.
* `side_of_street` = (response only) The side of street of a `break` `location` that is determined based on the actual route when the `location` is offset from the street. The possible values are `left` and `right`.
* `date_time` = (response only for `/route`) Expected date/time for the user to be at the location using the ISO 8601 format (YYYY-MM-DDThh:mm) in the local time zone of departure or arrival.  For example "2015-12-29T08:00". If `waiting` was set on this location in the request, and it's an intermediate location, the `date_time` will describe the departure time at this location.

Future development work includes adding location options and information related to time at each location. This will allow routes to specify a start time or an arrive by time at each location. There is also ongoing work to improve support for `through` locations.

### Costing models

Valhalla's routing service uses dynamic, run-time costing to generate the route path. The route request must include the name of the costing model and can include optional parameters available for the chosen costing model.

| Costing model | Description |
| :----------------- | :----------- |
| `auto` | Standard costing for driving routes by car, motorcycle, truck, and so on that obeys automobile driving rules, such as access and turn restrictions. `Auto` provides a short time path (though not guaranteed to be shortest time) and uses intersection costing to minimize turns and maneuvers or road name changes. Routes also tend to favor highways and higher classification roads, such as motorways and trunks. |
| `bicycle` | Standard costing for travel by bicycle, with a slight preference for using [cycleways](http://wiki.openstreetmap.org/wiki/Key:cycleway) or roads with bicycle lanes. Bicycle routes follow regular roads when needed, but avoid roads without bicycle access. |
| `bus` | Standard costing for bus routes. Bus costing inherits the auto costing behaviors, but checks for bus access on the roads. |
|**BETA** `bikeshare` | A combination of pedestrian and bicycle. Use bike share station(`amenity:bicycle_rental`) to change the travel mode |
| `truck` | Standard costing for trucks. Truck costing inherits the auto costing behaviors, but checks for truck access, width and height restrictions, and weight limits on the roads. |
| `hov` | DEPRECATED: use `auto` cost with HOV costing options.|
| `taxi` | Standard costing for taxi routes. Taxi costing inherits the auto costing behaviors, but checks for taxi lane access on the roads and favors those roads.|
| `motor_scooter` | Standard costing for travel by motor scooter or moped.  By default, motor_scooter costing will avoid higher class roads unless the country overrides allows motor scooters on these roads.  Motor scooter routes follow regular roads when needed, but avoid roads without motor_scooter, moped, or mofa access. |
|**BETA** `motorcycle` | Standard costing for travel by motorcycle.  This costing model provides options to tune the route to take roadways (road touring) vs. tracks and trails (adventure motorcycling).|
| `multimodal` | Currently supports pedestrian and transit. In the future, multimodal will support a combination of all of the above. |
| `pedestrian` | Standard walking route that excludes roads without pedestrian access. In general, pedestrian routes are shortest distance with the following exceptions: walkways and footpaths are slightly favored, while steps or stairs and alleys are slightly avoided. |

#### Costing options

Costing methods can have several options that can be adjusted to develop the route path, as well as for estimating time along the path. Specify costing model options in your request using the format of `costing_options.type`, such as ` costing_options.auto`.

* Cost options are fixed costs in seconds that are added to both the path cost and the estimated time. Examples of costs are `gate_costs` and `toll_booth_costs`, where a fixed amount of time is added. Costs are not generally used to influence the route path; instead, use penalties to do this. Costs must be in the range of 0.0 seconds to 43200.0 seconds (12 hours), otherwise a default value will be assigned.
* Penalty options are fixed costs in seconds that are only added to the path cost. Penalties can influence the route path determination but do not add to the estimated time along the path. For example, add a `toll_booth_penalty` to create route paths that tend to avoid toll booths. Penalties must be in the range of 0.0 seconds to 43200.0 seconds (12 hours), otherwise a default value will be assigned.
* Factor options are used to multiply the cost along an edge or road section in a way that influences the path to favor or avoid a particular attribute. Factor options do not impact estimated time along the path, though. Factors must be in the range 0.1 to 100000.0, where factors of 1.0 have no influence on cost. Anything outside of this range will be assigned a default value. Use a factor less than 1.0 to attempt to favor paths containing preferred attributes, and a value greater than 1.0 to avoid paths with undesirable attributes. Avoidance factors are more effective than favor factors at influencing a path. A factor's impact also depends on the length of road containing the specified attribute, as longer roads have more impact on the costing than very short roads. For this reason, penalty options tend to be better at influencing paths.

A special costing option is `shortest`, which, when `true`, will solely use distance as cost and disregard all other costs, penalties and factors. It's available for all costing models except `multimodal` & `bikeshare`.

Another special case is `disable_hierarchy_pruning` costing option. As the name indicates, `disable_hierarchy_pruning = true` will disable hierarchies in routing algorithms, which allows us to find the actual optimal route even in edge cases. For example, together with `shortest = true` they can find the actual shortest route. When `disable_hierarchy_pruning` is `true` and arc distances between source and target are not above the max limit, the actual optimal route will be calculated at the expense of performance. Note that if arc distances between locations exceed the max limit, `disable_hierarchy_pruning` is `true` will not be applied. This costing option is available for all motorized costing models, i.e `auto`, `motorcycle`, `motor_scooter`, `bus`, `truck` & `taxi`. For `bicycle` and `pedestrian` hierarchies are always disabled by default.

Additionally to the main costing option, the `recostings` option can be used to calculate the travelling time of the found route based on different costing options. By e.g. adding
```json
"recostings":[
    {"costing":"auto","fixed_speed":20,"name":"auto_20"},
    {"costing":"auto","fixed_speed":50,"name":"auto_50"}
]
```
to the route request, the values `time_auto_20` and `time_auto_50` will be added to summaries to show how much time the route would cost with these given costing options. Passing a recosting which make the route impossible to follow (e.g. the main rout is by car over a motorway and recosting with pedestrian costing) leads to a `none` result of this recosting.

##### Automobile and bus costing options

These options are available for `auto`, `bus`, and `truck` costing methods.

| Automobile options | Description |
| :-------------------------- | :----------- |
| `maneuver_penalty` | A penalty applied when transitioning between roads that do not have consistent naming–in other words, no road names in common. This penalty can be used to create simpler routes that tend to have fewer maneuvers or narrative guidance instructions. The default maneuver penalty is five seconds. |
| `gate_cost` | A cost applied when a [gate](http://wiki.openstreetmap.org/wiki/Tag:barrier%3Dgate) with undefined or private access is encountered. This cost is added to the estimated time / elapsed time. The default gate cost is 30 seconds. |
| `gate_penalty` | A penalty applied when a [gate](https://wiki.openstreetmap.org/wiki/Tag:barrier%3Dgate) with no access information is on the road. The default gate penalty is 300 seconds. |
| `private_access_penalty` | A penalty applied when a [gate](https://wiki.openstreetmap.org/wiki/Tag:barrier%3Dgate) or [bollard](https://wiki.openstreetmap.org/wiki/Tag:barrier%3Dbollard) with `access=private` is encountered. The default private access penalty is 450 seconds. |
| `destination_only_penalty` | A penalty applied when entering an road which is only allowed to enter if necessary to reach the [destination](https://wiki.openstreetmap.org/wiki/Tag:vehicle%3Ddestination). |
| `toll_booth_cost` | A cost applied when a [toll booth](http://wiki.openstreetmap.org/wiki/Tag:barrier%3Dtoll_booth) is encountered. This cost is added to the estimated and elapsed times. The default cost is 15 seconds. |
| `toll_booth_penalty` | A penalty applied to the cost when a [toll booth](http://wiki.openstreetmap.org/wiki/Tag:barrier%3Dtoll_booth) is encountered. This penalty can be used to create paths that avoid toll roads. The default toll booth penalty is 0. |
| `ferry_cost` | A cost applied when entering a ferry. This cost is added to the estimated and elapsed times. The default cost is 300 seconds (5 minutes). |
| `use_ferry` | This value indicates the willingness to take ferries. This is a range of values between 0 and 1. Values near 0 attempt to avoid ferries and values near 1 will favor ferries. The default value is 0.5. Note that sometimes ferries are required to complete a route so values of 0 are not guaranteed to avoid ferries entirely. |
| `use_highways` | This value indicates the willingness to take highways. This is a range of values between 0 and 1. Values near 0 attempt to avoid highways and values near 1 will favor highways. The default value is 1.0. Note that sometimes highways are required to complete a route so values of 0 are not guaranteed to avoid highways entirely. |
| `use_tolls` | This value indicates the willingness to take roads with tolls. This is a range of values between 0 and 1. Values near 0 attempt to avoid tolls and values near 1 will not attempt to avoid them. The default value is 0.5. Note that sometimes roads with tolls are required to complete a route so values of 0 are not guaranteed to avoid them entirely. |
| `use_living_streets` | This value indicates the willingness to take living streets. This is a range of values between 0 and 1. Values near 0 attempt to avoid living streets and values near 1 will favor living streets. The default value is 0 for trucks, 0.1 for cars, buses, motor scooters and motorcycles. Note that sometimes living streets are required to complete a route so values of 0 are not guaranteed to avoid living streets entirely. |
| `use_tracks` | This value indicates the willingness to take track roads. This is a range of values between 0 and 1. Values near 0 attempt to avoid tracks and values near 1 will favor tracks a little bit. The default value is 0 for autos, 0.5 for motor scooters and motorcycles. Note that sometimes tracks are required to complete a route so values of 0 are not guaranteed to avoid tracks entirely. |
| `service_penalty` | A penalty applied for transition to generic service road. The default penalty is 0 trucks and 15 for cars, buses, motor scooters and motorcycles. |
| `service_factor` | A factor that modifies (multiplies) the cost when generic service roads are encountered. The default `service_factor` is 1. |
| `country_crossing_cost` | A cost applied when encountering an international border. This cost is added to the estimated and elapsed times. The default cost is 600 seconds. |
| `country_crossing_penalty` | A penalty applied for a country crossing. This penalty can be used to create paths that avoid spanning country boundaries. The default penalty is 0. |
| `shortest` | Changes the metric to quasi-shortest, i.e. purely distance-based costing. Note, this will disable all other costings & penalties. Also note, `shortest` will not disable hierarchy pruning, leading to potentially sub-optimal routes for some costing models. The default is `false`. |
| `use_distance` | A factor that allows controlling the contribution of distance and time to the route costs. The value is in range between 0 and 1, where 0 only takes time into account (default) and 1 only distance. A factor of 0.5 will weight them roughly equally. **Note:** this costing is currently only available for auto costing. |
| `disable_hierarchy_pruning` | Disable hierarchies to calculate the actual optimal route. The default is `false`. **Note:** This could be quite a performance drainer so there is a upper limit of distance. If the upper limit is exceeded, this option will always be `false`. |
| `top_speed` | Top speed the vehicle can go. Also used to avoid roads with higher speeds than this value. `top_speed` must be between 10 and 252 KPH. The default value is 120 KPH for `truck` and 140 KPH for `auto` and `bus`. |
| `fixed_speed` | Fixed speed the vehicle can go. Used to override the calculated speed. Can be useful if speed of vehicle is known. `fixed_speed` must be between 1 and 252 KPH. The default value is 0 KPH which disables fixed speed and falls back to the standard calculated speed based on the road attribution. |
| `ignore_closures` | If set to `true`, ignores all closures, marked due to live traffic closures, during routing. **Note:** This option cannot be set if `location.search_filter.exclude_closures` is also specified in the request and will return an error if it is |
| `closure_factor` | A factor that penalizes the cost when traversing a closed edge (eg: if `search_filter.exclude_closures` is `false` for origin and/or destination location and the route starts/ends on closed edges). Its value can range from `1.0` - don't penalize closed edges, to `10.0` - apply high cost penalty to closed edges. Default value is `9.0`. **Note:** This factor is applicable only for motorized modes of transport, i.e `auto`, `motorcycle`, `motor_scooter`, `bus`, `truck` & `taxi`. |
| `ignore_restrictions` | If set to `true`, ignores any restrictions (e.g. turn/dimensional/conditional restrictions). Especially useful for matching GPS traces to the road network regardless of restrictions. Default is `false`. |
| `ignore_oneways` | If set to `true`, ignores one-way restrictions. Especially useful for matching GPS traces to the road network ignoring uni-directional traffic rules. Not included in `ignore_restrictions` option. Default is `false`. |
| `ignore_non_vehicular_restrictions` | Similar to `ignore_restrictions`, but will respect restrictions that impact vehicle safety, such as weight and size restrictions. |
| `ignore_access` | Will ignore mode-specific access tags. Especially useful for matching GPS traces to the road network regardless of restrictions. Default is `false`. |
| `ignore_closures` | Will ignore traffic closures. Default is `false`. |
| `speed_types` | Will determine which speed sources are used, if available. A list of strings with the following possible values: <ul><li><code>freeflow</code></li><li><code>constrained</code></li><li><code>predicted</code></li><li><code>current</code></li></ul> Default is all sources (again, only if available). |

###### Other costing options
The following options are available for `auto`, `bus`, `taxi`, and `truck` costing methods.

| Vehicle Options | Description |
| :-------------------------- | :----------- |
| `height` | The height of the vehicle (in meters). Default 1.9 for car, bus, taxi and 4.11 for truck. |
| `width` | The width of the vehicle (in meters). Default 1.6 for car, bus, taxi and 2.6 for truck. |
| `exclude_unpaved` | This value indicates whether or not the path may include unpaved roads. If `exclude_unpaved` is set to 1 it is allowed to start and end with unpaved roads, but is not allowed to have them in the middle of the route path, otherwise they are allowed. Default false. |
| `exclude_cash_only_tolls` | A boolean value which indicates the desire to avoid routes with cash-only tolls. Default false. |
| `include_hov2` | A boolean value which indicates the desire to include HOV roads with a 2-occupant requirement in the route when advantageous. Default false. |
| `include_hov3` | A boolean value which indicates the desire to include HOV roads with a 3-occupant requirement in the route when advantageous. Default false. |
| `include_hot` | A boolean value which indicates the desire to include tolled HOV roads which require the driver to pay a toll if the occupant requirement isn't met. Default false. |

The following options are available for `truck` costing.

| Truck options | Description |
| :-------------------------- | :----------- |
| `length` | The length of the truck (in meters). Default 21.64. |
| `weight` | The weight of the truck (in metric tons). Default 21.77. |
| `axle_load` | The axle load of the truck (in metric tons). Default 9.07. |
| `axle_count` | The axle count of the truck. Default 5. |
| `hazmat` | A value indicating if the truck is carrying hazardous materials. Default false. |
| `hgv_no_access_penalty` | A penalty applied to roads with no HGV/truck access. If set to a value less than 43200 seconds, HGV will be allowed on these roads and the penalty applies. Default 43200, i.e. trucks are not allowed. |
| `low_class_penalty` | A penalty (in seconds) which is applied when going to residential or service roads. Default is 30 seconds. |
| `use_truck_route` | This value is a range of values from 0 to 1, where 0 indicates a light preference for streets marked as truck routes, and 1 indicates that streets not marked as truck routes should be avoided. This information is derived from the `hgv=designated` tag. Note that even with values near 1, there is no guarantee the returned route will include streets marked as truck routes. The default value is 0. | 


##### Bicycle costing options
The default bicycle costing is tuned toward road bicycles with a slight preference for using [cycleways](http://wiki.openstreetmap.org/wiki/Key:cycleway) or roads with bicycle lanes. Bicycle routes use regular roads where needed or where no direct bicycle lane options exist, but avoid roads without bicycle access. The costing model recognizes several factors unique to bicycle travel and offers several options for tuning bicycle routes. Several factors unique to travel by bicycle influence the resulting route.

* The types of roads suitable for bicycling depend on the type of bicycle. Road bicycles (skinny or narrow tires) generally are suited to paved roads or perhaps very short sections of compacted gravel. They are not designed for riding on coarse gravel or most paths and tracks through wooded areas or farmland. Mountain bikes, on the other hand, are able to traverse a wider set of surfaces.
* Average travel speed can be highly variable and can depend on bicycle type, fitness and experience of the cyclist, road surface, and hills. The costing model assumes a default speed on smooth, flat roads for each supported bicycle type. This speed can be overridden by an input option. The base speed is modulated by surface type (in conjunction with the bicycle type). In addition, speed is modified based on the hilliness of a road section.
* Bicyclists vary in their tolerance for riding on roads. Most novice bicyclists, and even other bicyclists, prefer cycleways and dedicated cycling paths and would rather avoid all but the quietest neighborhood roads. Other cyclists may be experienced riding on roads and prefer to take roadways because they often provide the fastest way to get between two places. The bicycle costing model accounts for this with a `use_roads` factor to indicate a cyclist's tolerance for riding on roads.
* Bicyclists vary in their fitness level and experience level, and many want to avoid hilly roads, and especially roads with very steep uphill or even downhill sections. Even if the fastest path is over a mountain, many cyclists prefer a flatter path that avoids the climb and descent up and over the mountain.

The following options described above for autos also apply to bicycle costing methods: `maneuver_penalty`, `gate_cost`, `gate_penalty`,  `destination_only_penalty` , `country_crossing_cost`, `country_costing_penalty`, and `service_penalty`.

These additional options are available for bicycle costing methods.

| Bicycle options | Description |
| :-------------------------- | :----------- |
| `bicycle_type` | The type of bicycle. The default type is `Hybrid`. <ul><li>`Road`: a road-style bicycle with narrow tires that is generally lightweight and designed for speed on paved surfaces. </li><li>`Hybrid` or `City`: a bicycle made mostly for city riding or casual riding on roads and paths with good surfaces.</li><li>`Cross`: a cyclo-cross bicycle, which is similar to a road bicycle but with wider tires suitable to rougher surfaces.</li><li>`Mountain`: a mountain bicycle suitable for most surfaces but generally heavier and slower on paved surfaces.</li><ul> |
| `cycling_speed` | Cycling speed is the average travel speed along smooth, flat roads. This is meant to be the speed a rider can comfortably maintain over the desired distance of the route. It can be modified (in the costing method) by surface type in conjunction with bicycle type and (coming soon) by hilliness of the road section. When no speed is specifically provided, the default speed is determined by the bicycle type and are as follows: Road = 25 KPH (15.5 MPH), Cross = 20 KPH (13 MPH), Hybrid/City = 18 KPH (11.5 MPH), and Mountain = 16 KPH (10 MPH). |
| `use_roads` | A cyclist's propensity to use roads alongside other vehicles. This is a range of values from 0 to 1, where 0 attempts to avoid roads and stay on cycleways and paths, and 1 indicates the rider is more comfortable riding on roads. Based on the `use_roads` factor, roads with certain classifications and higher speeds are penalized in an attempt to avoid them when finding the best path. The default value is 0.5. |
| `use_hills` | A cyclist's desire to tackle hills in their routes. This is a range of values from 0 to 1, where 0 attempts to avoid hills and steep grades even if it means a longer (time and distance) path, while 1 indicates the rider does not fear hills and steeper grades. Based on the `use_hills` factor, penalties are applied to roads based on elevation change and grade. These penalties help the path avoid hilly roads in favor of flatter roads or less steep grades where available. Note that it is not always possible to find alternate paths to avoid hills (for example when route locations are in mountainous areas). The default value is 0.5. |
| `use_ferry` | This value indicates the willingness to take ferries. This is a range of values between 0 and 1. Values near 0 attempt to avoid ferries and values near 1 will favor ferries. Note that sometimes ferries are required to complete a route so values of 0 are not guaranteed to avoid ferries entirely. The default value is 0.5. |
| `use_living_streets` | This value indicates the willingness to take living streets. This is a range of values between 0 and 1. Values near 0 attempt to avoid living streets and values from 0.5 to 1 will currently have no effect on route selection. The default value is 0.5. Note that sometimes living streets are required to complete a route so values of 0 are not guaranteed to avoid living streets entirely. |
| `avoid_bad_surfaces` | This value is meant to represent how much a cyclist wants to avoid roads with poor surfaces relative to the bicycle type being used. This is a range of values between 0 and 1. When the value is 0, there is no penalization of roads with different surface types; only bicycle speed on each surface is taken into account. As the value approaches 1, roads with poor surfaces for the bike are penalized heavier so that they are only taken if they significantly improve travel time. When the value is equal to 1, all bad surfaces are completely disallowed from routing, including start and end points. The default value is 0.25. |
|`bss_return_cost`| This value is useful when `bikeshare` is chosen as travel mode. It is meant to give the time will be used to return a rental bike. This value will be displayed in the final directions and used to calculate the whole duration. The default value is 120 seconds.|
|`bss_return_penalty`| This value is useful when `bikeshare` is chosen as travel mode. It is meant to describe the potential effort to return a rental bike. This value won't be displayed and used only inside of the algorithm.|
| `shortest` | Changes the metric to quasi-shortest, i.e. purely distance-based costing. Note, this will disable all other costings & penalties. Also note, `shortest` will not disable hierarchy pruning, leading to potentially sub-optimal routes for some costing models. The default is `false`. |

##### Motor_scooter costing options
Standard costing for travel by motor scooter or moped.  By default, motor_scooter costing will avoid higher class roads unless the country overrides allows motor scooters on these roads.  Motor scooter routes follow regular roads when needed, but avoid roads without motor_scooter, moped, or mofa access. The costing model recognizes factors unique to motor_scooter travel and offers options for tuning motor_scooter routes. Factors unique to travel by motor_scooter influence the resulting route.

All of the options described above for autos also apply to motor_scooter costing methods.  These additional options are available for motor_scooter costing methods.

| Motor_scooter options | Description |
| :-------------------------- | :----------- |
| `top_speed` | Top speed the motorized scooter can go. Used to avoid roads with higher speeds than this value. For `motor_scooter` this value must be between 20 and 120 KPH. The default value is 45 KPH (~28 MPH) |
| `use_primary` | A rider's propensity to use primary roads. This is a range of values from 0 to 1, where 0 attempts to avoid primary roads, and 1 indicates the rider is more comfortable riding on primary roads. Based on the `use_primary` factor, roads with certain classifications and higher speeds are penalized in an attempt to avoid them when finding the best path. The default value is 0.5. |
| `use_hills` | A rider's desire to tackle hills in their routes. This is a range of values from 0 to 1, where 0 attempts to avoid hills and steep grades even if it means a longer (time and distance) path, while 1 indicates the rider does not fear hills and steeper grades. Based on the `use_hills` factor, penalties are applied to roads based on elevation change and grade. These penalties help the path avoid hilly roads in favor of flatter roads or less steep grades where available. Note that it is not always possible to find alternate paths to avoid hills (for example when route locations are in mountainous areas). The default value is 0.5. |
| `shortest` | Changes the metric to quasi-shortest, i.e. purely distance-based costing. Note, this will disable all other costings & penalties. Also note, `shortest` will not disable hierarchy pruning, leading to potentially sub-optimal routes for some costing models. The default is `false`. |
| `disable_hierarchy_pruning` | Disable hierarchies to calculate the actual optimal route. The default is `false`. **Note:** This could be quite a performance drainer so there is a upper limit of distance. If the upper limit is exceeded, this option will always be `false`. |

##### Motorcycle costing options -> **BETA**
Standard costing for travel by motorcycle.  By default, motorcycle costing will default to higher class roads.  The costing model recognizes factors unique to motorcycle travel and offers options for tuning motorcycle routes.

All of the options described above for autos also apply to motorcycle costing methods.
The following options are available for motorcycle costing:

| Motorcycle options | Description |
| :-------------------------- | :----------- |
| `use_highways` | A riders's propensity to prefer the use of highways. This is a range of values from 0 to 1, where 0 attempts to avoid highways, and values toward 1 indicates the rider prefers highways. The default value is 1.0. |
| `use_trails` | A riders's desire for adventure in their routes.  This is a range of values from 0 to 1, where 0 will avoid trails, tracks, unclassified or bad surfaces and values towards 1 will tend to avoid major roads and route on secondary roads.  The default value is 0.0. |
| `shortest` | Changes the metric to quasi-shortest, i.e. purely distance-based costing. Note, this will disable all other costings & penalties. Also note, `shortest` will not disable hierarchy pruning, leading to potentially sub-optimal routes for some costing models. The default is `false`. |
| `disable_hierarchy_pruning` | Disable hierarchies to calculate the actual optimal route. The default is `false`. **Note:** This could be quite a performance drainer so there is a upper limit of distance. If the upper limit is exceeded, this option will always be `false`. |

##### Pedestrian costing options

These options are available for pedestrian costing methods.

| Pedestrian options | Description |
| :-------------------------- | :----------- |
| `walking_speed` | Walking speed in kilometers per hour. Must be between 0.5 and 25 km/hr. Defaults to 5.1 km/hr (3.1 miles/hour). |
| `walkway_factor` | A factor that modifies the cost when encountering roads classified as `footway` (no motorized vehicles allowed), which may be designated footpaths or designated sidewalks along residential roads. Pedestrian routes generally attempt to favor using these [walkways and sidewalks](https://wiki.openstreetmap.org/wiki/Sidewalks). The default walkway_factor is 1.0. |
| `sidewalk_factor` | A factor that modifies the cost when encountering roads with dedicated sidewalks. Pedestrian routes generally attempt to favor using [sidewalks](https://wiki.openstreetmap.org/wiki/Key:sidewalk). The default sidewalk_factor is 1.0. |
| `alley_factor` | A factor that modifies (multiplies) the cost when [alleys](http://wiki.openstreetmap.org/wiki/Tag:service%3Dalley) are encountered. Pedestrian routes generally want to avoid alleys or narrow service roads between buildings. The default alley_factor is 2.0. |
| `driveway_factor` | A factor that modifies (multiplies) the cost when encountering a [driveway](http://wiki.openstreetmap.org/wiki/Tag:service%3Ddriveway), which is often a private, service road. Pedestrian routes generally want to avoid driveways (private). The default driveway factor is 5.0. |
| `step_penalty` | A penalty in seconds added to each transition onto a path with [steps or stairs](http://wiki.openstreetmap.org/wiki/Tag:highway%3Dsteps). Higher values apply larger cost penalties to avoid paths that contain flights of steps. |
| `use_ferry` | This value indicates the willingness to take ferries. This is range of values between 0 and 1. Values near 0 attempt to avoid ferries and values near 1 will favor ferries. The default value is 0.5. Note that sometimes ferries are required to complete a route so values of 0 are not guaranteed to avoid ferries entirely. |
| `use_living_streets` | This value indicates the willingness to take living streets. This is a range of values between 0 and 1. Values near 0 attempt to avoid living streets and values near 1 will favor living streets. The default value is 0.6. Note that sometimes living streets are required to complete a route so values of 0 are not guaranteed to avoid living streets entirely. |
| `use_tracks` | This value indicates the willingness to take track roads. This is a range of values between 0 and 1. Values near 0 attempt to avoid tracks and values near 1 will favor tracks a little bit. The default value is 0.5. Note that sometimes tracks are required to complete a route so values of 0 are not guaranteed to avoid tracks entirely. |
| `use_hills` | This is a range of values from 0 to 1, where 0 attempts to avoid hills and steep grades even if it means a longer (time and distance) path, while 1 indicates the pedestrian does not fear hills and steeper grades. Based on the `use_hills` factor, penalties are applied to roads based on elevation change and grade. These penalties help the path avoid hilly roads in favor of flatter roads or less steep grades where available. Note that it is not always possible to find alternate paths to avoid hills (for example when route locations are in mountainous areas). The default value is 0.5. |
| `use_lit` | This value is a range of values from 0 to 1, where 0 indicates indifference towards lit streets, and 1 indicates that unlit streets should be avoided. Note that even with values near 1, there is no guarantee the returned route will include lit segments. The default value is 0. |
| `service_penalty` | A penalty applied for transition to generic service road. The default penalty is 0. |
| `service_factor` | A factor that modifies (multiplies) the cost when generic service roads are encountered. The default `service_factor` is 1. |
| `destination_only_penalty` | A penalty applied when entering an road which is only allowed to enter if necessary to reach the [destination](https://wiki.openstreetmap.org/wiki/Tag:vehicle%3Ddestination) |
| `max_hiking_difficulty` | This value indicates the maximum difficulty of hiking trails that is allowed. Values between 0 and 6 are allowed. The values correspond to *sac_scale* values within OpenStreetMap, see reference [here](https://wiki.openstreetmap.org/wiki/Key:sac_scale). The default value is 1 which means that well cleared trails that are mostly flat or slightly sloped are allowed. Higher difficulty trails can be allowed by specifying a higher value for max_hiking_difficulty.
|`bss_rent_cost`| This value is useful when `bikeshare` is chosen as travel mode. It is meant to give the time will be used to rent a bike from a bike share station. This value will be displayed in the final directions and used to calculate the whole duration. The default value is 120 seconds.|
|`bss_rent_penalty`| This value is useful when `bikeshare` is chosen as travel mode. It is meant to describe the potential effort to rent a bike from a bike share station. This value won't be displayed and used only inside of the algorithm.|
| `shortest` | Changes the metric to quasi-shortest, i.e. purely distance-based costing. Note, this will disable all other costings & penalties. Also note, `shortest` will not disable hierarchy pruning, leading to potentially sub-optimal routes for some costing models. The default is `false`. |
| `max_distance` | Sets the maximum total walking distance of a route. Default is 100 km (~62 miles). |
| `transit_start_end_max_distance` | A pedestrian option that can be added to the request to extend the defaults (2145 meters or approximately 1.5 miles). This is the maximum walking distance at the beginning or end of a route.|
| `transit_transfer_max_distance` | A pedestrian option that can be added to the request to extend the defaults (800 meters or 0.5 miles). This is the maximum walking distance between transfers.|
| `type` | If set to `blind`, enables additional route instructions, especially useful for blind users: Announcing crossed streets, the stairs, bridges, tunnels, gates and bollards, which need to be passed on route; information about traffic signals on crosswalks; route numbers not announced for named routes. Default `foot` |
| `mode_factor` | A factor which the cost of a pedestrian edge will be multiplied with on multimodal request, e.g. `bss` or `multimodal/transit`. Default is a factor of 1.5, i.e. avoiding walking.

##### Transit costing options

These options are available for transit costing when the multimodal costing model is used.

| Transit options | Description |
| :-------------------------- | :----------- |
| `use_bus` | User's desire to use buses. Range of values from 0 (try to avoid buses) to 1 (strong preference for riding buses). |
| `use_rail` | User's desire to use rail/subway/metro. Range of values from 0 (try to avoid rail) to 1 (strong preference for riding rail).|
| `use_transfers` |User's desire to favor transfers. Range of values from 0 (try to avoid transfers) to 1 (totally comfortable with transfers).|
| `filters` | A way to filter for one or more ~~`stops`~~ (TODO: need to re-enable), `routes`, or `operators`. Filters must contain a list of so-called Onestop IDs, which is (supposed to be) a unique identifier for GTFS data, and an `action`. The OneStop ID is simply the feeds's directory name and the object's GTFS ID separated by an underscore, i.e. a route with `route_id: AUR` in `routes.txt` from the feed `NYC` would have the OneStop ID `NYC_AUR`, similar with operators/agencies. <ul><li>`ids`: any number of Onestop IDs</li><li>`action`: either `exclude` to exclude all of the `ids` listed in the filter or `include` to include only the `ids` listed in the filter</li></ul>

##### Hard exclusions -> **EXPERIMENTAL**

The following options are available for all costing methods. Those options are not available by default, the server config must have `service_limits.allow_hard_exclusions` set to true in order to allow them. If not allowed and any of the hard excludes is set to true, the server will return a warning and ignore the hard excludes.

| Vehicle Options | Description |
| :-------------------------- | :----------- |
| `exclude_bridges` | This value indicates whether or not the path may include bridges. If `exclude_bridges` is set to true it is allowed to start and end with bridges, but is not allowed to have them in the middle of the route path, otherwise they are allowed. If set to true, it is highly plausible that no path will be found. Default false. |
| `exclude_tunnels` | This value indicates whether or not the path may include tunnels. If `exclude_tunnels` is set to true it is allowed to start and end with tunnels, but is not allowed to have them in the middle of the route path, otherwise they are allowed. If set to true, it is highly plausible that no path will be found. Default false. |
| `exclude_tolls` | This value indicates whether or not the path may include tolls. If `exclude_tolls` is set to true it is allowed to start and end with tolls, but is not allowed to have them in the middle of the route path, otherwise they are allowed. If set to true, it is highly plausible that no path will be found. Default false. |
| `exclude_highways` | This value indicates whether or not the path may include highways. If `exclude_highways` is set to true it is allowed to start and end with highways, but is not allowed to have them in the middle of the route path, otherwise they are allowed. If set to true, it is highly plausible that no path will be found. Default false. |
| `exclude_ferries` | This value indicates whether or not the path may include ferries. If `exclude_ferries` is set to true it is allowed to start and end with ferries, but is not allowed to have them in the middle of the route path, otherwise they are allowed. If set to true, it is highly plausible that no path will be found. Default false. |

##### Sample JSON payloads for multimodal requests with transit

A multimodal request at the current date and time:

```json
{"locations":[{"lat":40.730930,"lon":-73.991379,"street":"Wanamaker Place"},{"lat":40.749706,"lon":-73.991562,"street":"Penn Plaza"}],"costing":"multimodal","units":"miles"}
```

A multimodal request departing on 2016-03-29 at 08:00:

```json
{"locations":[{"lat":40.749706,"lon":-73.991562,"type":"break","street":"Penn Plaza"},{"lat":40.73093,"lon":-73.991379,"type":"break","street":"Wanamaker Place"}],"costing":"multimodal","date_time":{"type":1,"value":"2016-03-29T08:00"}}
```

A multimodal request for a route favoring buses and a person walking at a set speed of 4.1 km/h:

```json
{"locations":[{"lat":40.749706,"lon":-73.991562,"type":"break","street":"Penn Plaza"},{"lat":40.73093,"lon":-73.991379,"type":"break","street":"Wanamaker Place"}],"costing":"multimodal","costing_options":{"transit":{"use_bus":"1.0","use_rail":"0.0","use_transfers":"0.3"},"pedestrian":{"walking_speed":"4.1"}}}
```

A multimodal request with a filter for certain Onestop IDs:

```json
{"locations":[{"lat":40.730930,"lon":-73.991379,"street":"Wanamaker Place"},{"lat":40.749706,"lon":-73.991562,"street":"Penn Plaza"}],"costing":"multimodal","costing_options":{"transit":{"filters":{"routes":{"ids":["NYC_AUR"],"action":"exclude"},"operators":{"ids":["paris_CFG","berlin_VBB"],"action":"include"}}}},"units":"miles"}
```

#### Directions options

Directions options should be specified at the top level of the JSON object.

| Options | Description |
| :------------------ | :----------- |
| `units` | Distance units for output. Allowable unit types are miles (or mi) and kilometers (or km). If no unit type is specified, the units default to kilometers. |
| `language` | The language of the narration instructions based on the [IETF BCP 47](https://tools.ietf.org/html/bcp47) language tag string. If no language is specified or the specified language is unsupported, United States-based English (en-US) is used. [Currently supported language list](#supported-language-tags) |
| `directions_type` |  An enum with 3 values. <ul><li>`none` indicating no maneuvers or instructions should be returned.</li><li>`maneuvers` indicating that only maneuvers be returned.</li><li>`instructions` indicating that maneuvers with instructions should be returned (this is the default if not specified).</li></ul> |
| `format` | Four options are available: <ul><li>`json` is default valhalla routing directions JSON format</li><li>`gpx` returns the route as a GPX (GPS exchange format) XML track</li><li>`osrm` creates a OSRM compatible route directions JSON</li><li>`pbf` formats the result using protocol buffers</li></ul> |
| `shape_format` | If `"format" : "osrm"` is set: Specifies the optional format for the path shape of each connection. One of `polyline6` (default), `polyline5`, `geojson` or `no_shape`. |
| `banner_instructions` | If the format is `osrm`, this boolean indicates if each step should have the additional `bannerInstructions` attribute, which can be displayed in some navigation system SDKs. |
| `voice_instructions` | If the format is `osrm`, this boolean indicates if each step should have the additional `voiceInstructions` attribute, which can be heard in some navigation system SDKs. |
| `alternates` |  A number denoting how many alternate routes should be provided. There may be no alternates or less alternates than the user specifies. Alternates are not yet supported on multipoint routes (that is, routes with more than 2 locations). They are also not supported on time dependent routes. |

For example a bus request with the result in Spanish using the OSRM (Open Source Routing Machine) format with the additional bannerInstructions and voiceInstructions in the steps would use the following json:

```json
{"locations":[{"lat":40.730930,"lon":-73.991379},{"lat":40.749706,"lon":-73.991562}],"format":"osrm","costing":"bus","banner_instructions":true,"voice_instructions":true,"language":"es-ES"}
```

##### Supported language tags

| Language tag | Language alias | Description |
| :------------------ | :----------- | :----------- |
| `bg-BG` | `bg` | Bulgarian (Bulgaria) |
| `ca-ES` | `ca` | Catalan (Spain) |
| `cs-CZ` | `cs` | Czech (Czech Republic) |
| `da-DK` | `da` | Danish (Denmark) |
| `de-DE` | `de` | German (Germany) |
| `el-GR` | `el` | Greek (Greece) |
| `en-GB` | | English (United Kingdom) |
| `en-US-x-pirate` | `en-x-pirate` | English (United States) Pirate |
| `en-US` | `en` | English (United States) |
| `es-ES` | `es` | Spanish (Spain) |
| `et-EE` | `et` | Estonian (Estonia) |
| `fi-FI` | `fi` | Finnish (Finland) |
| `fr-FR` | `fr` | French (France) |
| `hi-IN` | `hi` | Hindi (India) |
| `hu-HU` | `hu` | Hungarian (Hungary) |
| `it-IT` | `it` | Italian (Italy) |
| `ja-JP` | `ja` | Japanese (Japan) |
| `nb-NO` | `nb` | Bokmal (Norway) |
| `nl-NL` | `nl` | Dutch (Netherlands) |
| `pl-PL` | `pl` | Polish (Poland) |
| `pt-BR` | | Portuguese (Brazil) |
| `pt-PT` | `pt` | Portuguese (Portugal) |
| `ro-RO` | `ro` | Romanian (Romania) |
| `ru-RU` | `ru` | Russian (Russia) |
| `sk-SK` | `sk` | Slovak (Slovakia) |
| `sl-SI` | `sl` | Slovenian (Slovenia) |
| `sv-SE` | `sv` | Swedish (Sweden) |
| `tr-TR` | `tr` | Turkish (Turkey) |
| `uk-UA` | `uk` | Ukrainian (Ukraine) |

#### Other request options

| Options | Description |
| :------------------ | :----------- |
| `exclude_locations` |  A set of locations to exclude or avoid within a route can be specified using a JSON array of avoid_locations. The avoid_locations have the same format as the locations list. At a minimum each avoid location must include latitude and longitude. The avoid_locations are mapped to the closest road or roads and these roads are excluded from the route path computation.|
| `exclude_polygons` |  One or multiple exterior rings of polygons in the form of nested JSON arrays, e.g. `[[[lon1, lat1], [lon2,lat2]],[[lon1,lat1],[lon2,lat2]]]`. Roads intersecting these rings will be avoided during path finding. If you only need to avoid a few specific roads, it's **much** more efficient to use `exclude_locations`. Valhalla will close open rings (i.e. copy the first coordinate to the last position).|
| `date_time` | This is the local date and time at the location.<ul><li>`type`<ul><li>0 - Current departure time.</li><li>1 - Specified departure time</li><li>2 - Specified arrival time. Not yet implemented for multimodal costing method.</li><li>3 - Invariant specified time. Time does not vary over the course of the path. Not implemented for multimodal or bike share routing</li></ul></li><li>`value` - the date and time is specified in ISO 8601 format (YYYY-MM-DDThh:mm) in the local time zone of departure or arrival.  For example "2016-07-03T08:06"</li></ul><br> |
| `elevation_interval` | Elevation interval (meters) for requesting elevation along the route. Valhalla data must have been generated with elevation data. If no `elevation_interval` is specified, no elevation will be returned for the route. An elevation interval of 30 meters is recommended when elevation along the route is desired, matching the default data source's resolution. |
| `id` | Name your route request. If `id` is specified, the naming will be sent thru to the response. |
| `linear_references` | When present and `true`, the successful `route` response will include a key `linear_references`. Its value is an array of base64-encoded [OpenLR location references][openlr], one for each graph edge of the road network matched by the input trace. |
| `prioritize_bidirectional` | Prioritize `bidirectional a*` when `date_time.type = depart_at/current`. By default `time_dependent_forward a*` is used in these cases, but `bidirectional a*` is much faster. Currently it does not update the time (and speeds) when searching for the route path, but the ETA on that route is recalculated based on the time-dependent speeds |
| `roundabout_exits` | A boolean indicating whether exit instructions at roundabouts should be added to the output or not. Default is true. |
| `admin_crossings` | When present and `true`, the successful route summary will include the two keys `admins` and `admin_crossings`. `admins` is an array of administrative regions the route lies within. `admin_crossings` is an array of objects that contain `from_admin_index` and `to_admin_index`, which are indices into the `admins` array. They also contain `from_shape_index` and `to_shape_index`, which are start and end indices of the edge along which an administrative boundary is crossed. |

[openlr]: https://www.openlr-association.com/fileadmin/user_upload/openlr-whitepaper_v1.5.pdf

## Outputs of a route

If a route has been named in the request using the optional `&id=` input, then the name will be returned as a string `id` on the JSON object.

The route results are returned as a `trip`. This is a JSON object that contains details about the trip, including locations, a summary with basic information about the entire trip, and a list of `legs`.

Basic trip information includes:

| Trip item | Description |
| :---- | :----------- |
| `status` | Status code. |
| `status_message ` | Status message. |
| `units` | The specified units of length are returned, either kilometers or miles. |
| `language` | The language of the narration instructions. If the user specified a language in the directions options and the specified language was supported - this returned value will be equal to the specified value. Otherwise, this value will be the default (en-US) language. |
| `locations` | Location information is returned in the same form as it is entered with additional fields to indicate the side of the street. |
| `warnings` (optional) | This array may contain warning objects informing about deprecated request parameters, clamped values etc. |

The summary JSON object includes:

| Summary item | Description |
| :---- | :----------- |
| `time` | Estimated elapsed time to complete the trip. |
| `length` | Distance traveled for the entire trip. Units are either miles or kilometers based on the input units specified. |
| `has_toll`| Flag indicating if the path uses one or more toll segments. |
| `has_highway`| Flag indicating if the path uses one or more highway segments. |
| `has_ferry`| Flag indicating if the path uses one or more ferry segments. |
| `min_lat` | Minimum latitude of a bounding box containing the route. |
| `min_lon` | Minimum longitude of a bounding box containing the route. |
| `max_lat` | Maximum latitude of a bounding box containing the route. |
| `max_lon` | Maximum longitude of a bounding box containing the route. |


### Trip legs and maneuvers

A `trip` contains one or more `legs`. For *n* number of `break` locations, there are *n-1* legs. `Through` locations do not create separate legs.

Each leg of the trip includes a summary, which is comprised of the same information as a trip summary but applied to the single leg of the trip. It also includes a `shape`, which is an [encoded polyline](https://developers.google.com/maps/documentation/utilities/polylinealgorithm) of the route path (with 6 digits decimal precision), and a list of `maneuvers` as a JSON array. For more about decoding route shapes, see these [code examples](../../decoding.md).

If `elevation_interval` is specified, each leg of the trip will return `elevation` along the route as a JSON array. The `elevation_interval` is also returned. Units for both `elevation` and `elevation_interval` are either meters or feet based on the input units specified. 

Each maneuver includes:

| Maneuver item | Description |
| :--------- | :---------- |
| `type` | Type of maneuver. See below for a list. |
| `instruction` | Written maneuver instruction. Describes the maneuver, such as "Turn right onto Main Street". |
| `verbal_transition_alert_instruction` | Text suitable for use as a verbal alert in a navigation application. The transition alert instruction will prepare the user for the forthcoming transition. For example: "Turn right onto North Prince Street". |
| `verbal_pre_transition_instruction` | Text suitable for use as a verbal message immediately prior to the maneuver transition. For example "Turn right onto North Prince Street, U.S. 2 22". |
| `verbal_post_transition_instruction` | Text suitable for use as a verbal message immediately after the maneuver transition. For example "Continue on U.S. 2 22 for 3.9 miles". |
| `street_names` | List of street names that are consistent along the entire nonobvious maneuver. |
| `begin_street_names` | When present, these are the street names at the beginning (transition point) of the nonobvious maneuver (if they are different than the names that are consistent along the entire nonobvious maneuver). |
| `time` | Estimated time along the maneuver in seconds. |
| `length` | Maneuver length in the units specified. |
| `begin_shape_index` | Index into the list of shape points for the start of the maneuver. |
| `end_shape_index` | Index into the list of shape points for the end of the maneuver. |
| `toll` | True if the maneuver has any toll, or portions of the maneuver are subject to a toll. |
| `highway` | True if a highway is encountered on this maneuver. |
| `rough` | True if the maneuver is unpaved or rough pavement, or has any portions that have rough pavement. |
| `gate` | True if a gate is encountered on this maneuver. |
| `ferry` | True if a ferry is encountered on this maneuver. |
| `sign` | Contains the interchange guide information at a road junction associated with this maneuver. See below for details. |
| `roundabout_exit_count` | The spoke to exit roundabout after entering. |
| `depart_instruction` | Written depart time instruction. Typically used with a transit maneuver, such as "Depart: 8:04 AM from 8 St - NYU". |
| `verbal_depart_instruction` | Text suitable for use as a verbal depart time instruction. Typically used with a transit maneuver, such as "Depart at 8:04 AM from 8 St - NYU". |
| `arrive_instruction` | Written arrive time instruction. Typically used with a transit maneuver, such as "Arrive: 8:10 AM at 34 St - Herald Sq". |
| `verbal_arrive_instruction` | Text suitable for use as a verbal arrive time instruction. Typically used with a transit maneuver, such as "Arrive at 8:10 AM at 34 St - Herald Sq". |
| `transit_info` | Contains the attributes that describe a specific transit route. See below for details. |
| `verbal_multi_cue` | True if the `verbal_pre_transition_instruction` has been appended with the verbal instruction of the next maneuver. |
| `travel_mode` | Travel mode.<ul><li>"drive"</li><li>"pedestrian"</li><li>"bicycle"</li><li>"transit"</li></ul>|
| `travel_type` | Travel type for drive.<ul><li>"car"</li></ul>Travel type for pedestrian.<ul><li>"foot"</li></ul>Travel type for bicycle.<ul><li>"road"</li></ul>Travel type for transit.<ul><li>Tram or light rail = "tram"</li><li>Metro or subway = "metro"</li><li>Rail = "rail"</li><li>Bus = "bus"</li><li>Ferry = "ferry"</li><li>Cable car = "cable_car"</li><li>Gondola = "gondola"</li><li>Funicular = "funicular"</li></ul>|
| `bss_maneuver_type` | Used when `travel_mode` is `bikeshare`. Describes bike share maneuver. The default value is "NoneAction <ul><li>"NoneAction"</li><li>"RentBikeAtBikeShare"</li><li>"ReturnBikeAtBikeShare"</li></ul> |

For the maneuver `type`, the following are available:

```json
kNone = 0;
kStart = 1;
kStartRight = 2;
kStartLeft = 3;
kDestination = 4;
kDestinationRight = 5;
kDestinationLeft = 6;
kBecomes = 7;
kContinue = 8;
kSlightRight = 9;
kRight = 10;
kSharpRight = 11;
kUturnRight = 12;
kUturnLeft = 13;
kSharpLeft = 14;
kLeft = 15;
kSlightLeft = 16;
kRampStraight = 17;
kRampRight = 18;
kRampLeft = 19;
kExitRight = 20;
kExitLeft = 21;
kStayStraight = 22;
kStayRight = 23;
kStayLeft = 24;
kMerge = 25;
kRoundaboutEnter = 26;
kRoundaboutExit = 27;
kFerryEnter = 28;
kFerryExit = 29;
kTransit = 30;
kTransitTransfer = 31;
kTransitRemainOn = 32;
kTransitConnectionStart = 33;
kTransitConnectionTransfer = 34;
kTransitConnectionDestination = 35;
kPostTransitConnectionDestination = 36;
kMergeRight = 37;
kMergeLeft = 38;
kElevatorEnter = 39;
kStepsEnter = 40;
kEscalatorEnter = 41;
kBuildingEnter = 42;
kBuildingExit = 43;
```

The maneuver `sign` may contain four lists of interchange sign elements as follows:

* `exit_number_elements` = list of exit number elements. If an exit number element exists, it is typically just one value.
* `exit_branch_elements` = list of exit branch elements. The exit branch element text is the subsequent road name or route number after the sign.
* `exit_toward_elements` = list of exit toward elements. The exit toward element text is the location where the road ahead goes - the location is typically a control city, but may also be a future road name or route number.
* `exit_name_elements` = list of exit name elements. The exit name element is the interchange identifier - typically not used in the US.

Each maneuver sign element includes:

| Maneuver sign element item | Description |
| :------------------ | :---------- |
| `text` | Interchange sign text. <ul><li>exit number example: 91B.</li><li>exit branch example: I 95 North.</li><li>exit toward example: New York.</li><li>exit name example: Gettysburg Pike.</li><ul> |
| `consecutive_count` | The frequency of this sign element within a set a consecutive signs. This item is optional. |

A maneuver `transit_info` includes:

| Maneuver transit route item | Description |
| :--------- | :---------- |
| `onestop_id` | Global transit route identifier. |
| `short_name` | Short name describing the transit route. For example "N". |
| `long_name` | Long name describing the transit route. For example "Broadway Express". |
| `headsign` | The sign on a public transport vehicle that identifies the route destination to passengers. For example "ASTORIA - DITMARS BLVD". |
| `color` | The numeric color value associated with a transit route. The value for yellow would be "16567306". |
| `text_color` | The numeric text color value associated with a transit route. The value for black would be "0". |
| `description` | The description of the transit route. For example "Trains operate from Ditmars Boulevard, Queens, to Stillwell Avenue, Brooklyn, at all times. N trains in Manhattan operate along Broadway and across the Manhattan Bridge to and from Brooklyn. Trains in Brooklyn operate along 4th Avenue, then through Borough Park to Gravesend. Trains typically operate local in Queens, and either express or local in Manhattan and Brooklyn, depending on the time. Late night trains operate via Whitehall Street, Manhattan. Late night service is local". |
| `operator_onestop_id` | Global operator/agency identifier. |
| `operator_name` | Operator/agency name. For example, "BART", "King County Marine Division", and so on.  Short name is used over long name. |
| `operator_url` | Operator/agency URL. For example, "http://web.mta.info/". |
| `transit_stops` | A list of the stops/stations associated with a specific transit route. See below for details. |

A `transit_stop` includes:

| Transit stop item | Description |
| :--------- | :---------- |
| `type` | Type of stop (simple stop=0; station=1). |
| `name` | Name of the stop or station. For example "14 St - Union Sq". |
| `arrival_date_time` | Arrival date and time using the [ISO 8601](https://en.wikipedia.org/wiki/ISO_8601) format (YYYY-MM-DDThh:mm). For example, "2015-12-29T08:06". |
| `departure_date_time` | Departure date and time using the [ISO 8601](https://en.wikipedia.org/wiki/ISO_8601) format (YYYY-MM-DDThh:mm). For example, "2015-12-29T08:06". |
| `is_parent_stop` | True if this stop is a marked as a parent stop. |
| `assumed_schedule` | True if the times are based on an assumed schedule because the actual schedule is not known. |
| `lat` | Latitude of the transit stop in degrees. |
| `lon` | Longitude of the transit stop in degrees. |

Continuing with the earlier routing example from the Detroit, Michigan area, a maneuver such as this one may be returned with that request: `{"begin_shape_index":0,"length":0.109,"end_shape_index":1,"instruction":"Go south on Appleton.","street_names":["Appleton"],"type":1,"time":0}`

In the future, look for additional maneuver information to enhance navigation applications, including landmark usage.

### HTTP status codes and conditions

The following is a table of HTTP status error code conditions that may occur for a particular request. In general, the service follows the [HTTP specification](https://en.wikipedia.org/wiki/List_of_HTTP_status_codes). That is to say that `5xx` returns are generally ephemeral server problems that should be resolved shortly or are the result of a bug. `4xx` returns are used to mark requests that cannot be carried out, generally due to bad input in the request or problems with the underlying data. A `2xx` return is expected when there is a successful route result or `trip`, as described above.

| Status Code | Status | Description |
| :--------- | :---------- | :---------- |
| 200 | *your_trip_json* | A happy bit of json describing your `trip` result |
| 400 | Failed to parse json request | You need a valid json request |
| 400 | Failed to parse location | You need a valid location object in your json request |
| 400 | Failed to parse correlated location | There was a problem with the location once correlated to the route network |
| 400 | Insufficiently specified required parameter 'locations' | You forgot the locations parameter |
| 400 | No edge/node costing provided | You forgot the costing parameter |
| 400 | Insufficient number of locations provided | You didn't provide enough locations |
| 400 | Exceeded max route locations of X | You are asking for too many locations |
| 400 | Locations are in unconnected regions. Go check/edit the map at osm.org | You are routing between regions of no connectivity |
| 400 | No costing method found for 'X' | You are asking for a non-existent costing mode |
| 400 | Path distance exceeds the max distance limit | You want to travel further than this mode allows |
| 400 | No suitable edges near location | There were no edges applicable to your mode of travel near the input location |
| 400 | No data found for location | There was no route data tile at the input location |
| 400 | No path could be found for input | There was no path found between the input locations |
| 404 | Try any of: '/route' '/locate' | You asked for an invalid path |
| 405 | Try a POST or GET request instead | We only support GET and POST requests |
| 500 | Failed to parse intermediate request format | Had a problem reading an intermediate request format |
| 500 | Failed to parse TripPath | Had a problem reading the computed path from Protobuf |
| 500 | Could not build directions for TripPath | Had a problem using the trip path to create TripDirections |
| 500 | Failed to parse TripDirections | Had a problem using the trip directions to serialize a json response |
| 501 | Not implemented | Not Implemented |

### Internal error codes and conditions

The following is a table of exception internal error code conditions that may occur for a particular request. An [error code utility header file](https://raw.githubusercontent.com/valhalla/baldr/master/valhalla/baldr/errorcode_util.h) can be included by any of the Valhalla service projects.

The codes correspond to code returned from a particular [Valhalla project](https://github.com/valhalla/valhalla).

| Error code | Error |
| :--------- | :---------- |
|**1xx**| **Loki project codes** |
|100 | Failed to parse json request |
|101 | Try a POST or GET request instead |
|102 | The config actions for Loki are incorrectly loaded |
|103 | Missing max_locations configuration |
|104 | Missing max_distance configuration |
|105 | Path action not supported |
|106 | Try any of |
|107 | Not Implemented |
|110 | Insufficiently specified required parameter 'locations' |
|111 | Insufficiently specified required parameter 'time' |
|112 | Insufficiently specified required parameter 'locations' or 'sources & targets' |
|113 | Insufficiently specified required parameter 'contours' |
|114 | Insufficiently specified required parameter 'shape' or 'encoded_polyline' |
|120 | Insufficient number of locations provided |
|121 | Insufficient number of sources provided |
|122 | Insufficient number of targets provided |
|123 | Insufficient shape provided |
|124 | No edge/node costing provided |
|125 | No costing method found |
|126 | No shape provided |
|130 | Failed to parse location |
|131 | Failed to parse source |
|132 | Failed to parse target |
|140 | Action does not support multimodal costing |
|141 | Arrive by for multimodal not implemented yet |
|142 | Arrive by not implemented for isochrones |
|143 | ignore_closure in costing and exclude_closure in search_filter cannot both be specified |
|150 | Exceeded max locations |
|151 | Exceeded max time |
|152 | Exceeded max contours |
|153 | Too many shape points |
|154 | Path distance exceeds the max distance limit |
|155 | Outside the valid walking distance at the beginning or end of a multimodal route |
|156 | Outside the valid walking distance between stops of a multimodal route |
|157 | Exceeded max avoid locations |
|158 | Input trace option is out of bounds |
|160 | Date and time required for origin for date_type of depart at |
|161 | Date and time required for destination for date_type of arrive by |
|162 | Date and time is invalid.  Format is YYYY-MM-DDTHH:MM |
|163 | Invalid date_type |
|170 | Locations are in unconnected regions. Go check/edit the map at osm.org |
|171 | No suitable edges near location |
|199 | Unknown |
|**2xx** | **Odin project codes** |
|200 | Failed to parse intermediate request format |
|201 | Failed to parse TripPath |
|210 | Trip path does not have any nodes |
|211 | Trip path has only one node |
|212 | Trip must have at least 2 locations |
|213 | Error - No shape or invalid node count |
|220 | Turn degree out of range for cardinal direction |
|230 | Invalid TripDirections_Maneuver_Type in method FormTurnInstruction |
|231 | Invalid TripDirections_Maneuver_Type in method FormRelativeTwoDirection |
|232 | Invalid TripDirections_Maneuver_Type in method FormRelativeThreeDirection |
|299 | Unknown |
|**3xx** | **Skadi project codes** |
|300 | Failed to parse json request |
|301 | Try a POST or GET request instead |
|302 | The config actions for Skadi are incorrectly loaded |
|303 | Path action not supported |
|304 | Try any of |
|305 | Not Implemented |
|310 | No shape provided |
|311 | Insufficient shape provided |
|312 | Insufficiently specified required parameter 'shape' or 'encoded_polyline' |
|313 | 'resample_distance' must be >=  |
|314 | Too many shape points |
|399 | Unknown |
|**4xx** | **Thor project codes** |
|400 | Unknown action |
|401 | Failed to parse intermediate request format |
|410 | Insufficiently specified required parameter 'locations' |
|411 | Insufficiently specified required parameter 'shape' |
|412 | No costing method found |
|420 | Failed to parse correlated location |
|421 | Failed to parse location |
|422 | Failed to parse source |
|423 | Failed to parse target |
|424 | Failed to parse shape |
|430 | Exceeded max iterations in CostMatrix::SourceToTarget |
|440 | Cannot reach destination - too far from a transit stop |
|441 | Location is unreachable |
|442 | No path could be found for input |
|443 | Exact route match algorithm failed to find path |
|444 | Map Match algorithm failed to find path |
|445 | Shape match algorithm specification in api request is incorrect. Please see documentation for valid shape_match input. |
|499 | Unknown |
|**5xx** | **Tyr project codes** |
|500 | Failed to parse intermediate request format |
|501 | Failed to parse TripDirections |
|504 | GeoTIFF serialization not supported by service |
|599 | Unknown |
