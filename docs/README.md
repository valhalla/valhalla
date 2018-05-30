This folder contains the technical documentation and API reference for valhalla.

## Overview

- [Introduction](./valhalla-intro.md) - This is the early history of Valhalla. Introduces the core team and describes overall objectives of the project and some insight on why we chose the name Valhalla.
- [Terminology](./terminology.md) - Contains commonly used terms and definitions within Valhalla.

## API

- [Route API Reference](./turn-by-turn/api-reference.md) - The structure of API requests and responses to a Valhalla routing service is described here. This shows the JSON inputs and describes the JSON responses to form routes and directions.
- [Map Matching API Reference](./map-matching/api-reference.md) - The structure of API requests and responses to a Valhalla map matching service is described here. This shows the JSON inputs and describes the JSON responses to perform map-matching. There are two flavors: 1) `trace_route`: froms a route result from the path that matches the input geometry, and 2) `trace_attributes`: returns detailed attribution along the path that matches the input geometry.
- [Locate API Reference](./locate/api-reference.md) - The structure of API requests and responses to a Valhalla locate service is described here. This shows the JSON inputs and describes the JSON responses to get detailed information about streets and interesections near a location.
- [Matrix API Reference](./matrix/api-reference.md) - The structure of API requests and responses to a Valhalla time distance matrix service is described here. This shows the JSON inputs and describes the JSON responses to retrieve times and distances between locations.
- [Optimized Route API Reference](./optimized/api-reference.md) - The structure of API requests and responses to a Valhalla optimized route service is described here. This shows the JSON inputs and describes the JSON responses to retrieve the route which optimizes the path through the input locations. This is essentially the Traveling Salesman Problem (TSP)
- [Elevation API Reference](./elevation/api-reference.md) - The structure of API requests and responses to a Valhalla elevation service is described here. This shows the JSON inputs and describes the JSON responses to query elevation at specific locations.
- [Isochrone API Reference](./isochrone/api-reference.md) - The structure of API requests and responses to a Valhalla isochrone service is described here. This shows the JSON inputs and describes the JSON responses to query accessibility polygons around specific locations.

## Technical documentation

- [Add Routing to a Map](./add-routing-to-a-map.md) - A tutorial showing how to add Valhalla routing to web based maps using the Leaflet Routing Machine with Valhalla plugins.
- [Decoding Shape](./decoding.md) - Describes how to decode the route path's shape (returned as an encoded polyline). Contains sample code in several languages.
- [Tile Description](./tiles.md) - Describes the tiling system used within Valhalla. Discusses the road hierarchy and tile numbering system.
- [Speed information](./speeds.md) - Describes the use of speed information and how OpenStreetMap tags impact speeds.
- [Why Tiles?](./mjolnir/why_tiles.md) - Some of the objectives and reasons for designing a tiled, routing data set are included here.
- [OSM Connectivity Map](./connectivity.md) - Discusses creation of a "Connectivity Map" of OSM that uses Valhalla routing tiles to provide a first order of approximation of connectivity between locations.
- [Use of Administrative Data in Valhalla](./mjolnir/admins.md) - Discusses the importance of administrative information to routing and some of the ways that Valhalla uses adminstrative information.
- [Dynamic Costing](./sif/dynamic-costing.md) - Describes the basics of the dynamic, run-time path costing provided within the sif repository.
- [Elevation Influenced Bicycle Routing](./sif/elevation_costing.md) - Discusses how elevation is factored into bicycle costing to allow features such as "avoid hills".

## Data sources

- [Data sources](./mjolnir/data_sources.md) - A listing of data sources used within Valhalla routing tiles.
- [Attribution requirements](./mjolnir/attribution.md).


