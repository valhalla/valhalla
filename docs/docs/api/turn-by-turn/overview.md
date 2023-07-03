# Routing overview

The Valhalla route service (a.k.a. turn-by-turn) is an open-source routing service that lets you integrate routing and navigation into a web or mobile application. The service works globally, and provides dynamic and customizable routing by driving, walking, bicycling, and using multimodal and transit options, with clear directions for maneuvers along the route.

## Route requests and results

When you [request a route](api-reference.md#inputs-of-a-route), you are sending and receiving [JSON](https://en.wikipedia.org/wiki/JSON), which is a human-readable text format. In the JSON array, you need to specify the [locations](api-reference.md#locations) to visit on the route, the [costing model](api-reference.md#costing-models) that represents the mode of travel, such as car or bicycle, and your API key. The location coordinates, given in decimal degrees, can come from many input sources, such as a GPS location, a point or a click on a map, a geocoding service such as [Mapbox Geocoding](https://docs.mapbox.com/api/search/#geocoding), and so on. Costing methods can have several options that can be adjusted to develop the the route path and estimate the time along the path.

The service [route results](api-reference.md#outputs-of-a-route) provide details about the trip, including locations, a summary with basic information about the entire trip and a list of legs. Each leg has its own summary, a shape, which is an encoded polyline of the route path, and a list of maneuvers. These maneuvers provide written narrative instructions, plus verbal alerts that can be used as audio guidance in navigation apps.

The JSON returned from the route query can be drawn on a map and shown as instructions for maneuvers along the route. You can [display Valhalla routes](add-routing-to-a-map.md) on web and mobile maps.

## Data sources in Turn-by-Turn

Valhalla draws data from OpenStreetMap for the major part of the routing graph and can optionally include elevation data from the TileZen project hosted on the [AWS Open Data](https://registry.opendata.aws/terrain-tiles/) platform, user-provided GTFS feeds, reproducible timezone data, custom historical traffic data, live traffic and incident feeds.

The [source code](https://github.com/valhalla) is open to view and modify, and contributions are welcomed.
