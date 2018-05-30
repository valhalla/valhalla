# Transit Availability service API reference

The Transit Availability service provides a simple and quick check for one or more locations with an optional radius and returns a boolean value for if transit is available.

## Transit Availability service action

You can request the following action from the Transit Availability service: `/transit_available?`.

| Action type | Description |
| :--------- | :----------- |
| `transit_available` | Returns an isTransit boolean value of true or false, depending on if transit is available for the location or set of locations and radius. |

## Inputs of the transit availability service

An example request takes the form of `TBD/transit_available?json={}`, where the `transit_available?` represents the type of query and the JSON inputs inside the ``{}`` include an array of at least one location and with optional radius per location.

Here is an example of an Transit Availability request:
```
TBD/transit_available?json={"locations":[{"lat":35.647452, "lon":-79.597477, "radius":20}, {"lat":34.766908, "lon":-80.325936,"radius":10}]}
```

### Location parameters

A location must include a latitude and longitude in decimal degrees. The coordinates can come from many input sources, such as a GPS location, a point or a click on a map, a geocoding service, and so on. External search services, such as [Mapbox Geocoding](https://www.mapbox.com/api-documentation/#geocoding) can be used to find places and geocode addresses, whose coordinates can be used as input to the service.

| Location parameters | Description |
| :--------- | :----------- |
| `lat` | Latitude of the location in degrees. |
| `lon` | Longitude of the location in degrees. |
| `radius` | The number of meters about this input location within which edges (roads between intersections) will be considered as candidates for said location. When correlating this location to the route network, try to only return results within this distance (meters) from this location. If there are no candidates within this distance it will return the closest candidate within reason. If this value is larger than the configured service limit it will be clamped to that limit. The default is 0 meters. |

Refer to the [route location documentation](/turn-by-turn/api-reference.md#locations) for more information on specifying locations.

## Outputs of the transit availability service

These are the results of a request to the Transit Availability service.

| Item | Description |
| :---- | :----------- |
| `transit_available` | Returns a boolean value for if transit is available, along with the input a list of locations and radius used for the check.|
| `istransit` | A boolean value for whether or not transit exists for a particular location and radius.
| `locations` | The specified array of lat/lngs from the input request.  Locations may also contain an optional radius. |

See the [HTTP return codes](/turn-by-turn/api-reference.md#http-status-codes-and-conditions) for more on messages you might receive from the service.
