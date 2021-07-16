# Expansion service API reference

Routing algorithms find the best path by _expanding_ their search from start nodes/edges across the routing network until the destination is reached (unidirectional) or both search branches met (bidirectional).

The expansion service wraps the `route` and `isochrone` services and returns a GeoJSON with all network edges (way segments) the underlying routing algorithm visited during the expansion with relevant properties for each edge (`duration`, `distance` and `cost`). If a `route` request is tracked the response will also contain a `algorithm` member. The algorithms used are unidirectional & bidirectional A* (`route`) and unidirectional Dijkstra (`isochrone`).

**Note**, for even moderately long routes or isochrones the `/expansion` action can produce gigantic GeoJSON responses of 100s of MB.

![Isochrones for travel times by walking in Lancaster, PA](../images/expansion_dijkstra.png)

## Inputs of the Expansion service

Since this service wraps other services, the request format mostly follows the ones of the [route](../turn-by-turn/api-reference.md#inputs-of-a-route) and [isochrone](../isochrone/api-reference.md#inputs-of-the-isochrone-service). Additionally, it accepts the following parameters:

| Parameter          | Description                           |
| :---------         | :------------------------------------ |
| `action`           | The service whose expansion should be tracked. One of `route` or `isochrone`.  | 
| `skip_opposites`   | If set to `true` the output won't contain an edge's opposing edge. Opposing edges can be thought of as both directions of one road segment. Of the two, we discard the directional edge with higher cost and keep the one with less cost. | 
| `expansion_props`   | A list of the GeoJSON property keys you'd like to have in the response as JSON array of strings. One or multiple of "durations", "distances", "costs", "edge_ids", "statuses" | 

## Outputs of the Expansion service

In the service response, the expanded way segments are returned as [GeoJSON](http://geojson.org/). The geometry is a single `MultiLineString` with each LineString representing one way segment (edge). Due to the verbosity of the GeoJSON format, single geometry features would produce prohibitively huge responses. However, that also means that the `properties` contain arrays of the tracked attributes, where of course the indices are correlating, i.e. the 3rd element in a `properties` array will correspond to the 3rd LineString in the MultiLineString geometry.

| Property   | Description                           |
| :--------- | :------------------------------------ |
| `distances`   | JSON array of the accumulated distance in meters for each edge in order of graph traversal. | 
| `durations`   | JSON array of the accumulated duration in seconds for each edge in order of graph traversal. | 
| `costs`   | JSON array of the accumulated edge cost for each edge in order of graph traversal. | 
| `edge_ids`   | JSON array of the internal edge IDs for each edge in order of graph traversal. | 
| `statuses`   | JSON array of the edge states at the  for each edge in order of graph traversal. | 

An example response for `"action": "isochrone"` is:

```
{"properties":{"algorithm":"unidirectional_dijkstra"},"type":"FeatureCollection","features":[{"type":"Feature","geometry":{"type":"MultiLineString","coordinates":[[[0.00027,-0.00017],[0.00027,0.0]],[[0.00027,-0.00017],[0.00027,-0.00035]],[[0.00027,-0.00035],[0.00027,-0.00017]],[[0.00027,0.0],[0.00027,-0.00017]],[[0.00027,-0.00017],[0.00053,-0.00017]],[[0.00027,-0.00017],[0.0,-0.00017]],[[0.0,-0.00017],[0.00027,-0.00017]],[[0.00053,-0.00017],[0.0008,-0.00017]],[[0.0008,-0.00017],[0.00053,-0.00017]],[[0.00053,-0.00017],[0.00027,-0.00017]],[[0.00053,-0.00017],[0.0008,0.0]]]},"properties":{"distances":[20,20,40,40,30,30,60,60,90,120,80],"durations":[0,0,29,29,1,1,30,2,31,33,5],"costs":[0,0,1,1,1,1,2,2,3,4,11]}}]}
```

## Data credits

The image includes data from [OpenStreetMap](http://www.openstreetmap.org/).
