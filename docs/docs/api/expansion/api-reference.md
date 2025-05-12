# Expansion service API reference (BETA)

Routing algorithms find the best path by _expanding_ their search from start nodes/edges across the routing network until the destination is reached (unidirectional) or both search branches meet (bidirectional). This service could be subject to change in terms of API until we remove the BETA label.

The expansion service wraps the `route`, `isochrone` and `sources_to_targets` services and returns a GeoJSON with all network edges (way segments) the underlying routing algorithm visited during the expansion, with relevant properties for each edge (e.g. `duration` & `distance`). A top-level `algorithm` propertry informs about the used algorithm: unidirectional & bidirectional A* (for `route`), unidirectional Dijkstra (for `isochrone`) or bidirectional Dijkstra (for `sources_to_targets`).

**Note**, for even moderately long routes (or isochrones or few sources/targets) the `/expansion` action can produce gigantic GeoJSON responses of 100s of MB.

![A 11 km isochrone expansion result in Vienna, Austria](../images/expansion_dijkstra.png)

## Inputs of the Expansion service

Since this service wraps other services, the request format mostly follows the ones of the [route](../turn-by-turn/api-reference.md#inputs-of-a-route), [isochrone](../isochrone/api-reference.md#inputs-of-the-isochrone-service) and [matrix](../matrix/api-reference.md#inputs-of-the-matrix-service). Additionally, it accepts the following parameters:

| Parameter                         | Description                           |
|:----------------------------------| :------------------------------------ |
| `action` (required)               | The service whose expansion should be tracked. Currently one of `route`, `isochrone` or `sources_to_targets`. | 
| `skip_opposites` (optional)       | If set to `true` the output won't contain an edge's opposing edge. Opposing edges can be thought of as both directions of one road segment. Of the two, we discard the directional edge with higher cost and keep the one with less cost. Default false. | 
| `dedupe` (optional)               | If set to `true`, the output will contain each edge only once, significantly reducing the response size. The expansion will keep track of already visited edges and override their properties, ensuring that only the one with higher edge state is returned. Default `false`.                                     | 
| `expansion_properties` (optional) | A JSON array of strings of the GeoJSON property keys you'd like to have in the response. One or multiple of "duration", "distance", "cost", "edge_id", "pred_edge_id", "edge_status" or "expansion_type". **Note**, that each additional property will increase the output size by minimum ~ 10%. By default an empty `properties` object is returned. |

The `expansion_properties` choices are as follows:

| Property   | Description                           |
| :--------- | :------------------------------------ |
| `distance`   | Returns the accumulated distance in meters for each edge in order of graph traversal. | 
| `duration`   | Returns the accumulated duration in seconds for each edge in order of graph traversal. | 
| `cost`       | Returns the accumulated cost for each edge in order of graph traversal. | 
| `edge_id`   | Returns the internal edge IDs for each edge in order of graph traversal. Mostly interesting for debugging. | 
| `pred_edge_id` |  Returns the internal edge IDs of the predecessor for each edge in order of graph traversal. Mostly interesting for debugging. |
| `edge_status`   | Returns the edge states for each edge in order of graph traversal. Mostly interesting for debugging. Can be one of "r" (reached), "s" (settled), "c" (connected). |
| `expansion_type`   | Returns the expansion direction from which the edge was encountered. 0 for forward, 1 for reverse. |

An example request is:

```json
{
	"costing": "pedestrian",
	"action": "isochrone",
	"id": 1,
	"locations": [
		{
			"lon": 14.440689,
			"lat": 50.087052
		}
	],
	"contours": [
		{
			"time": 1
		}
	],
	"skip_opposites": true,
	"expansion_properties": [
		"duration",
		"edge_id",
		"pred_edge_id",
		"edge_status",
		"cost", 
		"expansion_type"
	]
}
```

## Outputs of the Expansion service

In the service response, the expanded way segments are returned as [GeoJSON](http://geojson.org/) as plain `LineString`s. Due to the verbosity of the GeoJSON format, single geometry features would produce prohibitively huge responses.

The output will only contain the `properties` which were specified in the `expansion_properties` request array. If the parameter was omitted in the request, the output will contain an empty `properties` object.

An example response for `"action": "isochrone"` is:

```json
{
	"type": "FeatureCollection",
	"features": [
		{
			"type": "Feature",
			"geometry": {
				"type": "LineString",
				"coordinates": [
					[
						14.441804,
						50.087371
					],
					[
						14.440275,
						50.087137
					]
				]
			},
			"properties": {
				"duration": 19,
				"cost": 19,
				"edge_status": "s",
				"edge_id": 4049718357265,
				"pred_edge_id": 70368744177663
			}
		},
		{
			"type": "Feature",
			"geometry": {
				"type": "LineString",
				"coordinates": [
					[
						14.440275,
						50.087137
					],
					[
						14.439981,
						50.087084
					]
				]
			},
			"properties": {
				"duration": 34,
				"cost": 34,
				"edge_status": "s",
				"edge_id": 4049617693969,
				"pred_edge_id": 4049718357265
			}
		},
		{
			"type": "Feature",
			"geometry": {
				"type": "LineString",
				"coordinates": [
					[
						14.439981,
						50.087084
					],
					[
						14.438998,
						50.086788
					]
				]
			},
			"properties": {
				"duration": 89,
				"cost": 89,
				"edge_status": "s",
				"edge_id": 4444184259857,
				"pred_edge_id": 4049617693969
			}
		}
	],
	"properties": {
		"algorithm": "dijkstras"
	}
}
```

## Credits

The image includes data from [OpenStreetMap](http://www.openstreetmap.org/) and the ["Positron" basemap by Carto](https://carto.com/help/building-maps/basemap-list/#positron-with-labels).
