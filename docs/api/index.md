Valhalla is an open-source toolkit for multimodal transportation, powered by open data. The APIs use the standard REST model of interaction with JSON serving as both the request and response formats. Please refer to a specific service's API documentation to learn about the various request/response properties. In addition to JSON request/response formats Valhalla also supports protocol buffers as request/response format. For more information regarding protocol buffer support [see here](./protocol-buffers.md).

The **route** service guides you between points by car, bike, foot, and multimodal combinations involving walking and riding public transit. Your apps can use the results from the route service to plan multimodal journeys with narratives to guide users by text and by voice. Valhalla draws data from OpenStreetMap and from [Transitland](https://transit.land/documentation/), the open transit data aggregation project. See the [api documentation](./turn-by-turn/api-reference.md).

Trying to run more than one errand in the day or start your own delivery service? The **optimized route** service computes the times and distances between many origins and destinations and provides you with an optimized path between the locations. See the [api documentation](./optimized/api-reference.md).

If you want only a table of the times and distances, start with the **matrix** service. See the [api documentation](./matrix/api-reference.md).

Use the **isochrone** service to get a computation of areas that are reachable within specified time periods from a location or set of locations. See the [api documentation](./isochrone/api-reference.md).

The **map-matching** service matches coordinates to known roads so you can turn a path into a route with narrative instructions and get the attribute values from that matched line. See the [api documentation](./map-matching/api-reference.md).

Use the **elevation** service to find the elevation along a path or at specified locations. See the [api documentation](./elevation/api-reference.md).

Use the **transit available** service to check the availability of transit for at least 1 location. See the [api documentation](./transit-available/api-reference.md).

You can use the **expansion** service to return a geojson representation of a graph traversal at a given location. See the [api documentation](./expansion/api-reference.md).

The **locate** service allows you to get detailed metadata about the nodes and edges in the graph. See the [api documentation](./locate/api-reference.md).

The **status** service is a simple service that returns information about the running server or valhalla instance. See the [api documentation](./status/api-reference.md).

The **centroid** service allows you to find the least cost convergence point of routes from multiple locations. Documentation coming soonish.
