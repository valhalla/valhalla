# Valhalla routing service API reference

Valhalla's routing service (a.k.a. turn-by-turn), is an open-source routing service that lets you integrate routing and navigation into a web or mobile application.

[View an interactive demo](http://valhalla.github.io/demos/routing)

The default logic for the OpenStreetMap tags, keys, and values used when routing are documented on an [OSM wiki page](http://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Valhalla).

## Inputs of a route

The route request run locally takes the form of `localhost:8002/route?json={}`, where the JSON inputs inside the `{}` include location information, name and options for the costing model, and output options. Here is the JSON payload for an example request:

```
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
| `search_cutoff` | The cutoff at which we will assume the input is too far away from civilisation to be worth correlating to the nearest graph elements |
| `node_snap_tolerance` | During edge correlation this is the tolerance used to determine whether or not to snap to the intersection rather than along the street, if the snap location is within this distance from the intersection the intersection is used instead. The default is 5 meters |
| `street_side_tolerance` | If your input coordinate is less than this tolerance away from the edge centerline then we set your side of street to none otherwise your side of street will be left or right depending on direction of travel |
| `street_side_max_distance` | The max distance in meters that the input coordinates or display ll can be from the edge centerline for them to be used for determining the side of street. Beyond this distance the side of street is set to none |
| `search_filter` | A set of optional filters to exclude candidate edges based on their attribution. The following exclusion filters are supported: <ul><li>`exclude_tunnel` (boolean, defaults to `false`): whether to exclude roads marked as tunnels</li><li>`exclude_bridge` (boolean, defaults to `false`): whether to exclude roads marked as bridges</li><li>`exclude_ramp` (boolean, defaults to `false`): whether to exclude link roads marked as ramps, note that some turn channels are also marked as ramps</li><li>`exclude_closures` (boolean, defaults to `true`): whether to exclude roads considered closed due to live traffic closure. **Note:** This option cannot be set if `costing_options.<costing>.ignore_closures` is also specified. An error is returned if both options are specified. **Note 2:** Ignoring closures at destination and source locations does NOT work for date_time type `0/1` & `2` respectively</li><li>`min_road_class` (string, defaults to `"service_other"`): lowest road class allowed</li><li>`max_road_class` (string, defaults to `"motorway"`): highest road class allowed</li></ul>Road classes from highest to lowest are: motorway, trunk, primary, secondary, tertiary, unclassified, residential, service_other.

Optionally, you can include the following location information without impacting the routing. This information is carried through the request and returned as a convenience.

* `name` = Location or business name. The name may be used in the route narration directions, such as "You have arrived at _&lt;business name&gt;_.")
* `city` = City name.
* `state` = State name.
* `postal_code` = Postal code.
* `country` = Country name.
* `phone` = Telephone number.
* `url` = URL for the place or location.
* `side_of_street` = (response only) The side of street of a `break` `location` that is determined based on the actual route when the `location` is offset from the street. The possible values are `left` and `right`.
* `date_time` = (response only) Expected date/time for the user to be at the location using the ISO 8601 format (YYYY-MM-DDThh:mm) in the local time zone of departure or arrival.  For example "2015-12-29T08:00".

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
| `hov` | Standard costing for high-occupancy vehicle (HOV) routes. HOV costing inherits the auto costing behaviors, but checks for HOV lane access on the roads and favors those roads.|
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

##### Automobile and bus costing options

These options are available for `auto`, `bus`, and `truck` costing methods.

| Automobile options | Description |
| :-------------------------- | :----------- |
| `maneuver_penalty` | A penalty applied when transitioning between roads that do not have consistent naming–in other words, no road names in common. This penalty can be used to create simpler routes that tend to have fewer maneuvers or narrative guidance instructions. The default maneuver penalty is five seconds. |
| `gate_cost` | A cost applied when a [gate](http://wiki.openstreetmap.org/wiki/Tag:barrier%3Dgate) is encountered. This cost is added to the estimated time / elapsed time. The default gate cost is 30 seconds. |
| `toll_booth_cost` | A cost applied when a [toll booth](http://wiki.openstreetmap.org/wiki/Tag:barrier%3Dtoll_booth) is encountered. This cost is added to the estimated and elapsed times. The default cost is 15 seconds. |
| `toll_booth_penalty` | A penalty applied to the cost when a [toll booth](http://wiki.openstreetmap.org/wiki/Tag:barrier%3Dtoll_booth) is encountered. This penalty can be used to create paths that avoid toll roads. The default toll booth penalty is 0. |
| `ferry_cost` | A cost applied when entering a ferry. This cost is added to the estimated and elapsed times. The default cost is 300 seconds (5 minutes). |
| `use_ferry` | This value indicates the willingness to take ferries. This is a range of values between 0 and 1. Values near 0 attempt to avoid ferries and values near 1 will favor ferries. The default value is 0.5. Note that sometimes ferries are required to complete a route so values of 0 are not guaranteed to avoid ferries entirely. |
| `use_highways` | This value indicates the willingness to take highways. This is a range of values between 0 and 1. Values near 0 attempt to avoid highways and values near 1 will favor highways. The default value is 1.0. Note that sometimes highways are required to complete a route so values of 0 are not guaranteed to avoid highways entirely. |
| `use_tolls` | This value indicates the willingness to take roads with tolls. This is a range of values between 0 and 1. Values near 0 attempt to avoid tolls and values near 1 will not attempt to avoid them. The default value is 0.5. Note that sometimes roads with tolls are required to complete a route so values of 0 are not guaranteed to avoid them entirely. |
| `use_living_streets` | This value indicates the willingness to take living streets. This is a range of values between 0 and 1. Values near 0 attempt to avoid living streets and values near 1 will favor living streets. The default value is 0 for trucks, 0.1 for cars, buses, motor scooters and motorcycles. Note that sometimes living streets are required to complete a route so values of 0 are not guaranteed to avoid living streets entirely. |
| `use_tracks` | This value indicates the willingness to take track roads. This is a range of values between 0 and 1. Values near 0 attempt to avoid tracks and values near 1 will favor tracks a little bit. The default value is 0 for autos, 0.5 for motor scooters and motorcycles. Note that sometimes tracks are required to complete a route so values of 0 are not guaranteed to avoid tracks entirely. |
| `service_penalty` | A penalty applied for transition to generic service road. The default penalty is 0 trucks and 15 for cars, buses, motor scooters and motorcycles. |
| `service_factor` | A factor that modifies (multiplies) the cost when generic service roads are encountered. The default `service_factor` is 1.2 for cars and buses and 1 for trucks, motor scooters and motorcycles. |
| `country_crossing_cost` | A cost applied when encountering an international border. This cost is added to the estimated and elapsed times. The default cost is 600 seconds. |
| `country_crossing_penalty` | A penalty applied for a country crossing. This penalty can be used to create paths that avoid spanning country boundaries. The default penalty is 0. |
| `shortest` | Changes the metric to quasi-shortest, i.e. purely distance-based costing. Note, this will disable all other costings & penalties. Also note, `shortest` will not disable hierarchy pruning, leading to potentially sub-optimal routes for some costing models. The default is `false`. |
| `top_speed` | Top speed the vehicle can go. Also used to avoid roads with higher speeds than this value. `top_speed` must be between 10 and 252 KPH. The default value is 140 KPH. |
| `ignore_closures` | If set to `true`, ignores all closures, marked due to live traffic closures, during routing. **Note:** This option cannot be set if `location.search_filter.exclude_closures` is also specified in the request and will return an error if it is |

###### Truck-specific costing options

In addition to the above, the following options are available for `truck` costing.

| Truck options | Description |
| :-------------------------- | :----------- |
| `height` | The height of the truck (in meters). |
| `width` | The width of the truck (in meters). |
| `length` | The length of the truck (in meters). |
| `weight` | The weight of the truck (in metric tons). |
| `axle_load` | The axle load of the truck (in metric tons). |
| `hazmat` | A value indicating if the truck is carrying hazardous materials. |

##### Bicycle costing options
The default bicycle costing is tuned toward road bicycles with a slight preference for using [cycleways](http://wiki.openstreetmap.org/wiki/Key:cycleway) or roads with bicycle lanes. Bicycle routes use regular roads where needed or where no direct bicycle lane options exist, but avoid roads without bicycle access. The costing model recognizes several factors unique to bicycle travel and offers several options for tuning bicycle routes. Several factors unique to travel by bicycle influence the resulting route.

*	The types of roads suitable for bicycling depend on the type of bicycle. Road bicycles (skinny or narrow tires) generally are suited to paved roads or perhaps very short sections of compacted gravel. They are not designed for riding on coarse gravel or most paths and tracks through wooded areas or farmland. Mountain bikes, on the other hand, are able to traverse a wider set of surfaces.
*	Average travel speed can be highly variable and can depend on bicycle type, fitness and experience of the cyclist, road surface, and hills. The costing model assumes a default speed on smooth, flat roads for each supported bicycle type. This speed can be overridden by an input option. The base speed is modulated by surface type (in conjunction with the bicycle type). In addition, speed is modified based on the hilliness of a road section.
*	Bicyclists vary in their tolerance for riding on roads. Most novice bicyclists, and even other bicyclists, prefer cycleways and dedicated cycling paths and would rather avoid all but the quietest neighborhood roads. Other cyclists may be experienced riding on roads and prefer to take roadways because they often provide the fastest way to get between two places. The bicycle costing model accounts for this with a `use_roads` factor to indicate a cyclist's tolerance for riding on roads.
*	Bicyclists vary in their fitness level and experience level, and many want to avoid hilly roads, and especially roads with very steep uphill or even downhill sections. Even if the fastest path is over a mountain, many cyclists prefer a flatter path that avoids the climb and descent up and over the mountain.

The following options described above for autos also apply to bicycle costing methods: `maneuver_penalty`, `gate_cost`, `gate_penalty`, `country_crossing_cost`, `country_costing_penalty`, and `service_penalty`.

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
|`bss_return_cost`| This value is useful when `bikeshare` is chosen as travel mode. It is meant to give the time will be used to return a rental bike. This value will be displayed in the final directions and used to calculate the whole duation. The default value is 120 seconds.|
|`bss_return_penalty`| This value is useful when `bikeshare` is chosen as travel mode. It is meant to describe the potential effort to return a rental bike. This value won't be displayed and used only inside of the algorithm.|
| `shortest` | Changes the metric to quasi-shortest, i.e. purely distance-based costing. Note, this will disable all other costings & penalties. Also note, `shortest` will not disable hierarchy pruning, leading to potentially sub-optimal routes for some costing models. The default is `false`. |

##### Motor_scooter costing options
Standard costing for travel by motor scooter or moped.  By default, motor_scooter costing will avoid higher class roads unless the country overrides allows motor scooters on these roads.  Motor scooter routes follow regular roads when needed, but avoid roads without motor_scooter, moped, or mofa access. The costing model recognizes factors unique to motor_scooter travel and offers options for tuning motor_scooter routes. Factors unique to travel by motor_scooter influence the resulting route.

All of the options described above for autos also apply to motor_scooter costing methods.  These additional options are available for motor_scooter costing methods.

| Motor_scooter options | Description |
| :-------------------------- | :----------- |
| `top_speed` | Top speed the motorized scooter can go. Used to avoid roads with higher speeds than this value. For `motor_scooter` this value must be between 20 and 120 KPH. The default value is 45 KPH (~28 MPH) |
| `use_primary` | A riders's propensity to use primary roads. This is a range of values from 0 to 1, where 0 attempts to avoid primary roads, and 1 indicates the rider is more comfortable riding on primary roads. Based on the `use_primary` factor, roads with certain classifications and higher speeds are penalized in an attempt to avoid them when finding the best path. The default value is 0.5. |
| `use_hills` | A riders's desire to tackle hills in their routes. This is a range of values from 0 to 1, where 0 attempts to avoid hills and steep grades even if it means a longer (time and distance) path, while 1 indicates the rider does not fear hills and steeper grades. Based on the `use_hills` factor, penalties are applied to roads based on elevation change and grade. These penalties help the path avoid hilly roads in favor of flatter roads or less steep grades where available. Note that it is not always possible to find alternate paths to avoid hills (for example when route locations are in mountainous areas). The default value is 0.5. |
| `shortest` | Changes the metric to quasi-shortest, i.e. purely distance-based costing. Note, this will disable all other costings & penalties. Also note, `shortest` will not disable hierarchy pruning, leading to potentially sub-optimal routes for some costing models. The default is `false`. |

##### Motorcycle costing options -> **BETA**
Standard costing for travel by motorcycle.  By default, motorcycle costing will default to higher class roads.  The costing model recognizes factors unique to motorcycle travel and offers options for tuning motorcycle routes.

All of the options described above for autos also apply to motorcycle costing methods.
The following options are available for motorcycle costing:

| Motorcycle options | Description |
| :-------------------------- | :----------- |
| `use_highways` | A riders's propensity to prefer the use of highways. This is a range of values from 0 to 1, where 0 attempts to avoid highways, and values toward 1 indicates the rider prefers highways. The default value is 1.0. |
| `use_trails` | A riders's desire for adventure in their routes.  This is a range of values from 0 to 1, where 0 will avoid trails, tracks, unclassified or bad surfaces and values towards 1 will tend to avoid major roads and route on secondary roads.  The default value is 0.0. |
| `shortest` | Changes the metric to quasi-shortest, i.e. purely distance-based costing. Note, this will disable all other costings & penalties. Also note, `shortest` will not disable hierarchy pruning, leading to potentially sub-optimal routes for some costing models. The default is `false`. |

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
| `service_penalty` | A penalty applied for transition to generic service road. The default penalty is 0. |
| `service_factor` | A factor that modifies (multiplies) the cost when generic service roads are encountered. The default `service_factor` is 1. |
| `max_hiking_difficulty` | This value indicates the maximum difficulty of hiking trails that is allowed. Values between 0 and 6 are allowed. The values correspond to *sac_scale* values within OpenStreetMap, see reference [here](https://wiki.openstreetmap.org/wiki/Key:sac_scale). The default value is 1 which means that well cleared trails that are mostly flat or slightly sloped are allowed. Higher difficulty trails can be allowed by specifying a higher value for max_hiking_difficulty.
|`bss_rent_cost`| This value is useful when `bikeshare` is chosen as travel mode. It is meant to give the time will be used to rent a bike from a bike share station. This value will be displayed in the final directions and used to calculate the whole duation. The default value is 120 seconds.|
|`bss_rent_penalty`| This value is useful when `bikeshare` is chosen as travel mode. It is meant to describe the potential effort to rent a bike from a bike share station. This value won't be displayed and used only inside of the algorithm.|
| `shortest` | Changes the metric to quasi-shortest, i.e. purely distance-based costing. Note, this will disable all other costings & penalties. Also note, `shortest` will not disable hierarchy pruning, leading to potentially sub-optimal routes for some costing models. The default is `false`. |
##### Transit costing options

These options are available for transit costing when the multimodal costing model is used.

| Transit options | Description |
| :-------------------------- | :----------- |
| `use_bus` | User's desire to use buses. Range of values from 0 (try to avoid buses) to 1 (strong preference for riding buses). |
| `use_rail` | User's desire to use rail/subway/metro. Range of values from 0 (try to avoid rail) to 1 (strong preference for riding rail).|
| `use_transfers` |User's desire to favor transfers. Range of values from 0 (try to avoid transfers) to 1 (totally comfortable with transfers).|
| `transit_start_end_max_distance` | A pedestrian option that can be added to the request to extend the defaults (2145 meters or approximately 1.5 miles). This is the maximum walking distance at the beginning or end of a route.|
| `transit_transfer_max_distance` | A pedestrian option that can be added to the request to extend the defaults (800 meters or 0.5 miles). This is the maximum walking distance between transfers.|
| `filters` | A way to filter for one or more `stops`, `routes`, or `operators`. Filters must contain a list of [Onestop IDs](https://transit.land/documentation/onestop-id-scheme/), which is a unique identifier for Transitland data, and an `action`. <ul><li>`ids`: any number of Onestop IDs (such as o-9q9-bart)</li><li>`action`: either `exclude` to exclude all of the `ids` listed in the filter or `include` to include only the `ids` listed in the filter</li></ul>

###### Filter transit data

When using `filters`, you need to include a [Onestop ID](https://transit.land/documentation/onestop-id-scheme/) to identify the stop, routes, or operators to include or exclude in your query. Depending on how you are interacting with transit data from Transitland, there are different ways of obtaining the Onestop ID.

- Turn-by-Turn API: Query a transit route query and parse the returned JSON maneuver  for `transit_info` to find `operator_onestop_id` and the route `onestop_id`. A `transit_stop` contains the `onestop_id` for the stop.
- [Mobility Explorer](https://github.com/transitland/mobility-explorer): Click a single route, stop, or operator on the map, or use the drop-down menu to find the Onestop ID for routes and operators. The Onestop ID, among other details, is listed in the sidebar.
- [Transitland](https://transit.land/): Use the Transitland Datastore API to query directly for stops, routes, and operators using a number of options. For example, you can filter for only [subway routes](http://transit.land/api/v1/routes?vehicle_type=metro) or [bus routes](http://transit.land/api/v1/routes?vehicle_type=bus). See the [Transitland Datastore API documentation](https://transit.land/documentation/datastore/api-endpoints.html) for details.

##### Sample JSON payloads for multimodal requests with transit

A multimodal request at the current date and time:

```
{"locations":[{"lat":40.730930,"lon":-73.991379,"street":"Wanamaker Place"},{"lat":40.749706,"lon":-73.991562,"street":"Penn Plaza"}],"costing":"multimodal","units":"miles"}
```

A multimodal request departing on 2016-03-29 at 08:00:

```
{"locations":[{"lat":40.749706,"lon":-73.991562,"type":"break","street":"Penn Plaza"},{"lat":40.73093,"lon":-73.991379,"type":"break","street":"Wanamaker Place"}],"costing":"multimodal","date_time":{"type":1,"value":"2016-03-29T08:00"}}
```

A multimodal request for a route favoring buses and a person walking at a set speed of 4.1 km/h:

```
{"locations":[{"lat":40.749706,"lon":-73.991562,"type":"break","street":"Penn Plaza"},{"lat":40.73093,"lon":-73.991379,"type":"break","street":"Wanamaker Place"}],"costing":"multimodal","costing_options":{"transit":{"use_bus":"1.0","use_rail":"0.0","use_transfers":"0.3"},"pedestrian":{"walking_speed":"4.1"}}}
```

A multimodal request with a filter for certain Onestop IDs:

```
{"locations":[{"lat":40.730930,"lon":-73.991379,"street":"Wanamaker Place"},{"lat":40.749706,"lon":-73.991562,"street":"Penn Plaza"}],"costing":"multimodal","costing_options":{"transit":{"filters":{"stops":{"ids":["s-dr5rsq8pqg-8st~nyu&#60;r21n","s-dr5rsr9wyg-14st&#126;unionsq&#60;r20n"],"action":"exclude"},"routes":{"ids":["r-dr5r-r"],"action":"exclude"},"operators":{"ids":["o-dr5r-path"],"action":"include"}}}},"units":"miles"}
```

#### Directions options

Directions options should be specified at the top level of the JSON, and usage of the `directions_options` nested structure is deprecated.

| Options | Description |
| :------------------ | :----------- |
| `units` | Distance units for output. Allowable unit types are miles (or mi) and kilometers (or km). If no unit type is specified, the units default to kilometers. |
| `language` | The language of the narration instructions based on the [IETF BCP 47](https://tools.ietf.org/html/bcp47) language tag string. If no language is specified or the specified language is unsupported, United States-based English (en-US) is used. [Currently supported language list](#supported-language-tags) |
| `directions_type` |  An enum with 3 values. <ul><li>`none` indicating no maneuvers or instructions should be returned.</li><li>`maneuvers` indicating that only maneuvers be returned.</li><li>`instructions` indicating that maneuvers with instructions should be returned (this is the default if not specified).</li></ul> |
| `narrative` |  **DEPRECATED** Should use `directions_type` instead. Boolean to allow you to disable narrative production. Locations, shape, length, and time are still returned. The narrative production is enabled by default. Set the value to `false` to disable the narrative. |

##### Supported language tags

| Language tag | Language alias | Description |
| :------------------ | :----------- | :----------- |
| `bg-BG` | `bg` | Bulgarian (Bulgaria) |
| `ca-ES` | `ca` | Catalan (Spain) |
| `cs-CZ` | `cs` | Czech (Czech Republic) |
| `de-DE` | `de` | German (Germany) |
| `el-GR` | `el` | Greek (Greece) |
| `en-US` | `en` | English (United States) |
| `en-US-x-pirate` | `pirate` | English (United States) Pirate |
| `es-ES` | `es` | Spanish (Spain) |
| `et-EE` | `et` | Estonian (Estonia) |
| `fr-FR` | `fr` | French (France) |
| `hi-IN` | `hi` | Hindi (India) |
| `it-IT` | `it` | Italian (Italy) |
| `ja-JP` | `ja` | Japanese (Japan) |
| `pt-PT` | `pt` | Portuguese (Portugal) |
| `ru-RU` | `ru` | Russian (Russia) |
| `sk-SK` | `sk` | Slovak (Slovakia) |
| `sl-SI` | `sl` | Slovenian (Slovenia) |
| `sv-SE` | `sv` | Swedish (Sweden) |

#### Other request options

| Options | Description |
| :------------------ | :----------- |
| `avoid_locations` |  A set of locations to exclude or avoid within a route can be specified using a JSON array of avoid_locations. The avoid_locations have the same format as the locations list. At a minimum each avoid location must include latitude and longitude. The avoid_locations are mapped to the closest road or roads and these roads are excluded from the route path computation.|
| `date_time` | This is the local date and time at the location.<ul><li>`type`<ul><li>0 - Current departure time.</li><li>1 - Specified departure time</li><li>2 - Specified arrival time. Not yet implemented for multimodal costing method.</li></li>3 - Invariant specified time. Time does not vary over the course of the path. Not implemented for multimodal or bike share routing</li></ul></li><li>`value` - the date and time is specified in ISO 8601 format (YYYY-MM-DDThh:mm) in the local time zone of departure or arrival.  For example "2016-07-03T08:06"</li></ul><ul><b>NOTE: This option is not supported for Valhalla's matrix service.</b><ul> |
| `out_format` | Output format. If no `out_format` is specified, JSON is returned. Future work includes PBF (protocol buffer) support. |
| `id` | Name your route request. If `id` is specified, the naming will be sent thru to the response. |
| `linear_references` | When present and `true`, the successful `route` response will include a key `linear_references`. Its value is an array of base64-encoded [OpenLR location references][openlr], one for each graph edge of the road network matched by the input trace. |

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

The summary JSON object includes:

| Summary item | Description |
| :---- | :----------- |
| `time` | Estimated elapsed time to complete the trip. |
| `length` | Distance traveled for the entire trip. Units are either miles or kilometers based on the input units specified. |
| `min_lat` | Minimum latitude of a bounding box containing the route. |
| `min_lon` | Minimum longitude of a bounding box containing the route. |
| `max_lat` | Maximum latitude of a bounding box containing the route. |
| `max_lon` | Maximum longitude of a bounding box containing the route. |

### Trip legs and maneuvers

A `trip` contains one or more `legs`. For *n* number of `break` locations, there are *n-1* legs. `Through` locations do not create separate legs.

Each leg of the trip includes a summary, which is comprised of the same information as a trip summary but applied to the single leg of the trip. It also includes a `shape`, which is an [encoded polyline](https://developers.google.com/maps/documentation/utilities/polylinealgorithm) of the route path (with 6 digits decimal precision), and a list of `maneuvers` as a JSON array. For more about decoding route shapes, see these [code examples](/decoding.md).

Each maneuver includes:

| Maneuver item | Description |
| :--------- | :---------- |
| `type` | Type of maneuver. See below for a list. |
| `instruction` | Written maneuver instruction. Describes the maneuver, such as "Turn right onto Main Street". |
| `verbal_transition_alert_instruction` | Text suitable for use as a verbal alert in a navigation application. The transition alert instruction will prepare the user for the forthcoming transition. For example: "Turn right onto North Prince Street". |
| `verbal_pre_transition_instruction` | Text suitable for use as a verbal message immediately prior to the maneuver transition. For example "Turn right onto North Prince Street, U.S. 2 22". |
| `verbal_post_transition_instruction` | Text suitable for use as a verbal message immediately after the maneuver transition. For example "Continue on U.S. 2 22 for 3.9 miles". |
| `street_names` | List of street names that are consistent along the entire maneuver. |
| `begin_street_names` | When present, these are the street names at the beginning of the maneuver (if they are different than the names that are consistent along the entire maneuver). |
| `time` | Estimated time along the maneuver in seconds. |
| `length` | Maneuver length in the units specified. |
| `begin_shape_index` | Index into the list of shape points for the start of the maneuver. |
| `end_shape_index` | Index into the list of shape points for the end of the maneuver. |
| `toll` | True if the maneuver has any toll, or portions of the maneuver are subject to a toll. |
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

```
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
| `onestop_id` | Global transit route identifier from Transitland. |
| `short_name` | Short name describing the transit route. For example "N". |
| `long_name` | Long name describing the transit route. For example "Broadway Express". |
| `headsign` | The sign on a public transport vehicle that identifies the route destination to passengers. For example "ASTORIA - DITMARS BLVD". |
| `color` | The numeric color value associated with a transit route. The value for yellow would be "16567306". |
| `text_color` | The numeric text color value associated with a transit route. The value for black would be "0". |
| `description` | The description of the the transit route. For example "Trains operate from Ditmars Boulevard, Queens, to Stillwell Avenue, Brooklyn, at all times. N trains in Manhattan operate along Broadway and across the Manhattan Bridge to and from Brooklyn. Trains in Brooklyn operate along 4th Avenue, then through Borough Park to Gravesend. Trains typically operate local in Queens, and either express or local in Manhattan and Brooklyn, depending on the time. Late night trains operate via Whitehall Street, Manhattan. Late night service is local". |
| `operator_onestop_id` | Global operator/agency identifier from Transitland. |
| `operator_name` | Operator/agency name. For example, "BART", "King County Marine Division", and so on.  Short name is used over long name. |
| `operator_url` | Operator/agency URL. For example, "http://web.mta.info/". |
| `transit_stops` | A list of the stops/stations associated with a specific transit route. See below for details. |

A `transit_stop` includes:

| Transit stop item | Description |
| :--------- | :---------- |
| `type` | Type of stop (simple stop=0; station=1). |
| `onestop_id` | Global transit stop identifier from Transitland. |
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
|599 | Unknown |
