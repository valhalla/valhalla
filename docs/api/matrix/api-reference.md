# Time-Distance Matrix service API reference

Valhalla's time-distance matrix service provides a quick computation of time and distance between a set of locations and returns them to you in the resulting matrix table.

## Matrix service

The time distance matrix service takes a `sources` and `targets` to list locations. This allows you to set the source (origin) locations separately from the target (destination) locations. The set of origins may be disjoint (not overlapping) with the set of destinations. In other words, the target locations do not have to include any locations from source locations. The time-distance matrix can return a row matrix, a column matrix, or a general matrix of computed time and distance, depending on your input for the sources and targets parameters. The general case is a row ordered matrix with the time and distance from each source location to each target location. A row vector is considered a *one_to_many* time-distance matrix where there is one source location and multiple target locations. The time and distance from the source location to all target locations is returned. A column matrix represents a *many_to_one* time-distance matrix where there are many sources and one target. Another special case is when the source location list is the same as the target location list. Here, a diagonal (square matrix with [0,0.00] on the diagonal elements) matrix is returned. The is special case is often used as the input to optimized routing problems.

## Inputs of the matrix service

The matrix request run locally takes the form of `localhost:8002/sources_to_targets?json={}`, where the JSON inputs inside the `{}` includes at least one location for both sources and for targets as well as the route costing type and options for the route costing model.


For example, while at your office, you want to know the times and distances to walk to several restaurants where you could have dinner, as well as the times and distances from each restaurant to the train station for your commute home. This will help you determine where to eat. 

`one-to-many using /sources_to_targets?`

```
{"sources":[{"lat":40.744014,"lon":-73.990508}],"targets":[{"lat":40.744014,"lon":-73.990508},{"lat":40.739735,"lon":-73.979713},{"lat":40.752522,"lon":-73.985015},{"lat":40.750117,"lon":-73.983704},{"lat":40.750552,"lon":-73.993519}],"costing":"pedestrian"}&id=ManyToMany_NYC_work_dinner
```

`many-to-one using /sources_to_targets?`

```
{"sources":[{"lat":40.744014,"lon":-73.990508},{"lat":40.739735,"lon":-73.979713},{"lat":40.752522,"lon":-73.985015},{"lat":40.750117,"lon":-73.983704},{"lat":40.750552,"lon":-73.993519}],"targets":[{"lat":40.750552,"lon":-73.993519}],"costing":"pedestrian"}&id=ManyToMany_NYC_work_dinner
```

`many-to-many using /sources_to_targets?`

```
{"sources":[{"lat":40.744014,"lon":-73.990508},{"lat":40.739735,"lon":-73.979713},{"lat":40.752522,"lon":-73.985015},{"lat":40.750117,"lon":-73.983704},{"lat":40.750552,"lon":-73.993519}],"targets":[{"lat":40.744014,"lon":-73.990508},{"lat":40.739735,"lon":-73.979713},{"lat":40.752522,"lon":-73.985015},{"lat":40.750117,"lon":-73.983704},{"lat":40.750552,"lon":-73.993519}],"costing":"pedestrian"}&id=ManyToMany_NYC_work_dinner
```

There is an option to name your matrix request.  You can do this by appending the following to your request `&id=`.  The `id` is returned with the response so a user could match to the corresponding request.

### Source and target parameters

When using the `sources_to_targets` action, you specify sources and targets as ordered lists of one or more locations within a JSON array, depending on the type of matrix result you are expecting.

A source and target must include a latitude and longitude in decimal degrees. The coordinates can come from many input sources, such as a GPS location, a point or a click on a map, a geocoding service, and so on.

| Source and target parameters | Description |
| :--------- | :----------- |
| `lat` | Latitude of the source/target in degrees. |
| `lon` | Longitude of the source/target in degrees. |

You can refer to the [route location documentation](/docs/api/turn-by-turn/api-reference.md#locations) for more information on specifying locations.  NOTE: Using `type` in addition to the `lat` and `lon` within the location parameter has no meaning for matrices.

### Costing parameters

The Time-Distance Matrix service uses the `auto`, `bicycle`, `pedestrian` and `bikeshare` and other costing models available in the route service. Exception: **multimodal costing is not supported** for the time-distance matrix service at this time.  Refer to the [route costing models](/docs/api/turn-by-turn/api-reference.md#costing-models) and [costing options](/docs/api/turn-by-turn/api-reference.md#costing-options) documentation for more on how to specify this input.

### Other request options

| Options | Description |
| :------------------ | :----------- |
| `id` | Name your matrix request. If `id` is specified, the naming will be sent thru to the response. |

## Outputs of the matrix service

If a matrix request has been named using the optional `&id=` input, then the name will be returned as a string `id`.

These are the results of a request to the Time-Distance Matrix service.

| Item | Description |
| :---- | :----------- |
| `sources_to_targets` | Returns an array of time and distance between the sources and the targets. The array is **row-ordered**. This means that the time and distance from the first location to all others forms the first row of the array, followed by the time and distance from the second source location to all target locations, etc. |
| `distance` | The computed distance between each set of points. Distance will always be 0.00 for the first element of the time-distance array for `one_to_many`, the last element in a `many_to_one`, and the first and last elements of a `many_to_many`. |
| `time` | The computed time between each set of points. Time will always be 0 for the first element of the time-distance array for `one_to_many`, the last element in a `many_to_one`, and the first and last elements of a `many_to_many`.  |
| `to_index` | The destination index into the locations array. |
| `from_index` | The origin index into the locations array. |
| `locations` | The specified array of lat/lngs from the input request.
| `units` | Distance units for output. Allowable unit types are mi (miles) and km (kilometers). If no unit type is specified, the units default to kilometers. |

See the [HTTP return codes](/docs/api/turn-by-turn/api-reference.md#http-status-codes-and-conditions) for more on messages you might receive from the service.

## Demonstration

[View an interactive demo](http://valhalla.github.io/demos/matrix//).

