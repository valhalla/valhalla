# Isochrone service API reference

An isochrone is a line that connects points of equal travel time about a given location, from the Greek roots of `iso` for equal and `chrone` for time. Valhalla's isochrone service computes areas that are reachable within specified time intervals from a location, and returns the reachable regions as contours of polygons or lines that you can display on a map.

Isochrone maps share some of the same concepts and terminology with familiar topographic maps, which depict contour lines for points of equal elevation. For this reason other terms common in topography apply, such as contours or isolines.

This is an example of isochrones showing the travel times by driving from a location in Melbourne, as depicted in Mobility Explorer.

![Isochrones for travel times by walking in Lancaster, PA](/images/isochrone.png)

## Inputs of the Isochrone service

An isochrone request run locally takes the form of `localhost:8002/isochrone?json={}`, where the JSON inputs inside the `{}` includes an array of at least one location and options for the [route costing model](/turn-by-turn/api-reference/#costing-models).

For example, you can use the isochrone service to find out where you can travel within a 15-minute walk from your office building. The API request for this uses `isochrone?` as the request action, `pedestrian` costing, and a single contour for a 15-minute time interval. The response is GeoJSON, which you can display on a map to visualize where you might be able to walk.

```
{"locations":[{"lat":40.744014,"lon":-73.990508}],"costing":"pedestrian","contours":[{"time":15,"color":"ff0000"}]}&id=Walk_From_Office
```

There is an option to name your isochrone request by appending `&id=`. The `id` is returned with the response so you can match it to your corresponding request.

### Location parameters

The `locations` must include a latitude and longitude in decimal degrees. The coordinates can come from many input sources, such as a GPS location, a point or a click on a map, a geocoding service, and so on. External search services, such as [Mapbox Geocoding](https://www.mapbox.com/api-documentation/#geocoding) can be used to find places and geocode addresses, whose coordinates can be used as input to the service.

| Location parameters | Description |
| :--------- | :----------- |
| `lat` | Latitude of the location in degrees. |
| `lon` | Longitude of the location in degrees. |

Refer to the [route location documentation](/turn-by-turn/api-reference.md#locations) for more information on specifying locations.

### Costing parameters

The isochrone service uses the `auto`, `bicycle`, `pedestrian`, and `multimodal` costing models available in the Valhalla Turn-by-Turn service. Refer to the [route costing models](/turn-by-turn/api-reference.md#costing-models) and [costing options](/turn-by-turn/api-reference.md#costing-options) documentation for more on how to specify this input.

### Other request parameters

| Parameter | Description |
| :------------------ | :----------- |
| `date_time` | The local date and time at the location. These parameters apply only for multimodal requests and are not used with other costing methods.<ul><li>`type`<ul><li>0 - Current departure time for multimodal requests.</li><li>1 - Specified departure time for multimodal requests.</li><li>2 - Specified arrival time. Note: This is not yet implemented.</li></ul></li><li>`value` - the date and time specified in ISO 8601 format (YYYY-MM-DDThh:mm) in the local time zone of departure or arrival. For example, "2016-07-03T08:06"</li></ul> |
| `id` | Name of the isochrone request. If `id` is specified, the name is returned with the response. |
| `contours` | A JSON array of contour objects with the time in minutes and color to use for each isochrone contour. You can specify up to four contours. <ul><li>`time` - A floating point value specifying the time in minutes for the contour.<li>`color` - The color for the output of the contour. Specify it as a [Hex value](http://www.w3schools.com/colors/colors_hexadecimal.asp), but without the `#`, such as `"color":"ff0000"` for red. If no color is specified, the isochrone service will assign a default color to the output.</li></ul>  |
| `polygons` | A Boolean value to determine whether to return geojson polygons or linestrings as the contours. The default is `false`, which returns lines; when `true`, polygons are returned. Note: When `polygons` is `true`, any contour that forms a ring is returned as a polygon. |
| `denoise` | A floating point value from `0` to `1` (default of `1`) which can be used to remove smaller contours. A value of `1` will only return the largest contour for a given time value. A value of `0.5` drops any contours that are less than half the area of the largest contour in the set of contours for that same time value. |
| `generalize` | A floating point value in meters used as the tolerance for [Douglas-Peucker](https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm) generalization. Note: Generalization of contours can lead to self-intersections, as well as intersections of adjacent contours. |

## Outputs of the Isochrone service

In the service response, the isochrone contours are returned as [GeoJSON](http://geojson.org/), which can be integrated into mapping applications.

The contours are calculated using rasters and are returned as either polygon or line features, depending on your input setting for the `polygons` parameter. If an isochrone request has been named using the optional `&id=` input, then the `id` is returned as a name property for the feature collection within the GeoJSON response.

See the [HTTP return codes](/turn-by-turn/api-reference.md#http-status-codes-and-conditions) for more on messages you might receive from the service.

### Draw isochrones on a map

Most JavaScript-based GeoJSON renderers, including [Leaflet](http://leafletjs.com/), can use the isochrone styling information directly from the response. At present, you cannot control the opacity through the API.

When making a map, drawing the isochrone contours as lines is more straightforward than polygons, and, therefore, currently is the default and recommended method. When deciding between the output as lines and polygons, consider your use case and the additional styling considerations involved with polygons. For example, fills should be rendered as semi-transparent over the other map layers so they are visible, although you may have more flexibility when using a vector-based map. In addition, polygons from multiple contour levels do not have overlapping areas cut out or removed. In other words, the outer contours include the areas of any inner contours, causing the colors and transparencies to blend when multiple contour polygons are drawn at the same time.

The Valhalla team has plans to improving the polygon isochrone output and rendering capabilities, including by demoting some rings to be inners of other rings and removing potential self-intersections in polygon geometries.

## Isochrone demonstration in Mobility Explorer

The open-source [Mobility Explorer](https://github.com/transitland/mobility-explorer) tool helps you understand transportation networks around the world and has tools for adding isochrones to a map. Search for a location and add a point to the map, then generate isochrones for certain modes of transit from that location.

## Future work on the isochrone service

The Isochrone service is in active development. To report software issues or suggest enhancements, open an issue in the [Valhalla GitHub repository](https://github.com/valhalla/valhalla/issues).

Several other options are being considered as future service enhancements. These include:

* Using distance rather than time for each unit.
* Generating outer contours or contours with interior holes for regions that cannot be accessed within the specified time, including with options to control the minimum size of interior holes.
* Removing self intersections from polygonal contours.
* Allowing multiple locations to compute the region reachable from any of the locations within a specified time.
* Generating contours with reverse access logic to see the region that can reach a specific location within the specified time.
* Returning raster data for potential animation using OpenGL shaders. This also has analysis use for being able to query thousands of locations to determine the time to each location, including improvements with one-to-many requests to the Valhalla Time-Distance Matrix service.

## Data credits

The image includes data from [OpenStreetMap](http://www.openstreetmap.org/).
