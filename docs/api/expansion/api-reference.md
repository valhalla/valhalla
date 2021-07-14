# Expansion service API reference

Routing algorithms find the best path by _expanding_ their search from start nodes/edges across the routing network until the destination is reached (unidirectional) or both search branches met (bidirectional).

The expansion service wraps the `route` and `isochrone` services and returns a GeoJSON with all network edges (way segments) the underlying routing algorithm visited during the expansion with relevant properties for each edge (`duration`, `distance` and `cost`). If a `route` request is tracked the response will also contain a `algorithm` member. The algorithms used are unidirectional & bidirectional A* (`route`) and unidirectional Dijkstra (`isochrone`).

**Note**, for even moderately long routes or isochrones the `/expansion` action can produce gigantic GeoJSON responses of 100s of MB.

## Inputs of the Expansion service

Since this service wraps other services, the request format mostly follows the ones of the [route](../turn-by-turn/api-reference.md#inputs-of-a-route) and [isochrone](../isochrone/api-reference.md#inputs-of-the-isochrone-service). Additionally, it accepts the following parameters:

| Parameter | Description                           |
| :--------- | :------------------------------------ |
| `action`   | The service whose expansion should be tracked. One of `route` or `isochrone`.  | 

## Outputs of the Expansion service

In the service response, the expanded way segments are returned as [GeoJSON](http://geojson.org/). The geometry is a single `MultiLineString` with each part of the multi geometry representing one way segment (edge). Due to the verbosity of the GeoJSON format, single geometry features would produce prohibitively huge responses. However, that also means that the `properties` contain arrays of the tracked attributes.

An example response is:

```
```

## Data credits

The image includes data from [OpenStreetMap](http://www.openstreetmap.org/).
