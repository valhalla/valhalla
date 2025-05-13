# Speed values in Valhalla APIs

To calculate speed and related factors when routing, such as travel times, Valhalla APIs consider speed limits in the OpenStreetMap source data, defaults for a particular category of road, or a measure of whether the road is in an urban or rural environment.

Real-time or historical traffic information is not currently included in speed calculations. Valhalla is working towards these capabilities, and the tiled data structures of Valhalla and dynamic costing approach can readily support traffic information when available.

## Assignment of speeds to roadways

Routing data contains two attributes to denote speed: `speed` and `speed_limit`.

The most important for routing determination is `speed`, given in units of kilometers per hour. The `speed` value, along with the length of the roadway edge, determine the travel time along a road section.

The `speed_limit` contains the posted speed limit, if available, and can be used by mobile navigation applications to display the speed limit and possibly alert the driver when it is exceeded.

The `speed` is assigned based on tags within the OpenStreetMap data as follows:

* `max_speed`: If a [`maxspeed`](http://wiki.openstreetmap.org/wiki/Key:maxspeed) tag is available from OSM, that speed is used as the routing speed and the `speed_limit` is set to that value. Note that `maxspeed=none` is valid and means the speed limit is unlimited (as on the German Autobahn); in this case we do not set a speed based on `maxspeed` but rely on the default speeds based on the `highway` tag (see next bullet point).
* `highway`:  If there is no `maxspeed` or `maxspeed=none` tag, then `speed` is based on the OSM [`highway`](http://wiki.openstreetmap.org/wiki/Key:highway) tag. There are a default set of speeds for each `highway` tag. Note that future work involves implementing [country-specific default speeds](https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Maxspeed) for highway tags.
* road density: The road density (the length of drivable roads in kilometers per square kilometer) at each node in the routing graph is estimated during Valhalla data import. The road density is used to determine if a road is in a rural or urban area. Roads in urban areas have their speed reduced if there is no `maxspeed` tag. In the future, this method may be replaced with a more accurate measure of rural versus urban regions, but density produces adequate results for now.

The `speed_type` attribute defines whether the assigned routing speed is from a speed limit or based on the highway tag.
