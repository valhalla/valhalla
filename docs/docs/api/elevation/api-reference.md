# Elevation service API reference

Valhalla's elevation lookup service provides digital elevation model (DEM) data as the result of a query. The elevation service data has many applications when combined with other routing and navigation data, including computing the steepness of roads and paths or generating an elevation profile chart along a route.

For example, you can get elevation data for a point, a trail, or a trip. You might use the results to consider hills for your bicycle trip, or when estimating battery usage for trips in electric vehicles.

View an interactive demo [here](http://valhalla.github.io/demos/elevation).

## Inputs of the elevation service

The elevation service currently has a single action, `/height?`, that can be requested. The `height` provides the elevation at a set of input locations, which are specified as either a `shape` or an `encoded_polyline`. The shape option uses an ordered list of one or more locations within a JSON array, while an encoded polyline stores multiple locations within a single string. If you include a `range` parameter and set it to `true`, both the height and cumulative distance are returned for each point.

The elevation service also supports sampling the input shape or encoded polyline in uniform intervals along the path defined by the successive locations. If you include a `resample_distance` parameter with a distance in meters the elevation service will resample the input polyline at the requested resample_distance and return heights (and cumulative distance if requested) at the resampled locations. The resampled polyline is also returned.

An elevation service request takes the form of `servername/height?json={}`, where the JSON inputs inside the ``{}`` includes location information and the optional range parameter.

There is an option to name your elevation request. You can do this by appending the following to your request `&id=`.  The `id` is returned with the response so a user could match to the corresponding request.

### Height Precision
By default, all height values are returned as integer values. This works fine for most cases. However, using integer precision when charting elevation results along a nearly flat road can lead to "stair step" changes in elevation. Height data can be returned with 1 or 2 digits decimal precision by specifying `height_precision`. 

| Height precision | Description |
| :--------- | :----------- |
| `height_precision` | Specifies the precision (number of decimal places) of all returned height values. Values of `0`, `1`, or `2` are admissable. Defaults to 0 (integer precision). Any other value will result in integer precision (0 decimal places).

### Use a shape list for input locations

The elevation request run locally takes the form of `localhost:8002/height?json={}`, where the JSON inputs inside the `{}` are described below.

A `shape` request must include a latitude and longitude in decimal degrees, and the locations are visited in the order specified. The input coordinates can come from many input sources, such as a GPS location, a point or a click on a map, a geocoding service, and so on.

These parameters are available for `shape`.

| Shape parameters | Description |
| :--------- | :----------- |
| `lat` | Latitude of the location in degrees. |
| `lon` | Longitude of the location in degrees. |

Here is an example JSON payload for a profile request using `shape`:

```json
{"range":true,"shape":[{"lat":40.712431,"lon":-76.504916},{"lat":40.712275,"lon":-76.605259},{"lat":40.712122,"lon":-76.805694},{"lat":40.722431,"lon":-76.884916},{"lat":40.812275,"lon":-76.905259},{"lat":40.912122,"lon":-76.965694}]}&id=Pottsville
```

This request provides `shape` points near Pottsville, Pennsylvania. The resulting profile response displays the input shape, as well as the `range` and `height` (as `range_height` in the response) for each point.

```json
{"shape":[{"lat":40.712433,"lon":-76.504913},{"lat":40.712276,"lon":-76.605263},{"lat":40.712124,"lon":-76.805695},{"lat":40.722431,"lon":-76.884918},{"lat":40.812275,"lon":-76.905258},{"lat":40.912121,"lon":-76.965691}],"range_height":[[0,307],[8467,272],[25380,204],[32162,204],[42309,180],[54533,198]]}
```

Without the `range`, the result looks something like this, with only a `height`:

```json
{"shape":[{"lat":40.712433,"lon":-76.504913},{"lat":40.712276,"lon":-76.605263},{"lat":40.712124,"lon":-76.805695},{"lat":40.722431,"lon":-76.884918},{"lat":40.812275,"lon":-76.905258},{"lat":40.912121,"lon":-76.965691}],"height":[307,272,204,204,180,198]}
```

### Use an encoded polyline for input locations

The `encoded_polyline` parameter is a string of a polyline-encoded, with **the specified precision**, shape and has the following parameters. Details on polyline encoding and decoding can be found [here](../../decoding.md).

| Encoded polyline parameters | Description |
| :--------- | :----------- |
| `shape_format` | `polyline6` or `polyline5`. Specifies whether the polyline is encoded with 6 digit precision (polyline6) or 5 digit precision (polyline5). If `shape_format` is not specified, the encoded polyline is expected to be 6 digit precision.
| `encoded_polyline` | A set of encoded latitude, longitude pairs of a line or shape.|

Here is an example of the JSON payload for an `encoded_polyline` POST request:

```json
{"range":true,"encoded_polyline":"s{cplAfiz{pCa]xBxBx`AhC|gApBrz@{[hBsZhB_c@rFodDbRaG\\ypAfDec@l@mrBnHg|@?}TzAia@dFw^xKqWhNe^hWegBfvAcGpG{dAdy@_`CpoBqGfC_SnI{KrFgx@?ofA_Tus@c[qfAgw@s_Agc@}^}JcF{@_Dz@eFfEsArEs@pHm@pg@wDpkEx\\vjT}Djj@eUppAeKzj@eZpuE_IxaIcF~|@cBngJiMjj@_I`HwXlJuO^kKj@gJkAeaBy`AgNoHwDkAeELwD|@uDfC_i@bq@mOjUaCvDqBrEcAbGWbG|@jVd@rPkAbGsAfDqBvCaIrFsP~RoNjWajBlnD{OtZoNfXyBtE{B~HyAtEsFhL_DvDsGrF_I`HwDpGoH|T_IzLaMzKuOrFqfAbPwCl@_h@fN}OnI"}
```

### Get height and distance with the range parameter

The `range` parameter is a boolean value that controls whether or not the returned array is one-dimensional (height only) or two-dimensional (with a range and height). This can be used to generate a graph along a route, because a 2D-array has values for x (the range) and y (the height) at each shape point. Steepness or gradient can also be computed from a profile request (e.g. when range = `true`).

The `range` is optional and assumed to be `false` if omitted.

| Range parameters | Description |
| :--------- | :----------- |
| `range` | `true` or `false`. Defaults to `false`.|

### Resampling

The `resample_distance` parameter is a numeric value specifying the distance at which the input polyline is sampled in order to provide uniform distances between samples along the polyline.

| Sampling parameter | Description |
| :--------- | :----------- |
| `resample_distance` | Numeric value indicating the sampling distance in meters. |

### Other request options

| Options | Description |
| :------------------ | :----------- |
| `id` | Name your elevation request. If `id` is specified, the naming will be sent thru to the response. |

## Outputs of the elevation service

If an elevation request has been named using the optional `&id=` input, then the name will be returned as a string `id`.

The profile results are returned with the form of shape (shape points or encoded polylines) that was supplied in the request, along with a 2D array representing the x and y of each input point in the elevation profile.

| Item | Description |
| :---- | :----------- |
| `shape` | The specified shape coordinates from the input request. |
| `encoded_polyline` | The specified encoded polyline, with six degrees of precision, coordinates from the input request. |
| `range_height` | The 2D array of range (x) and height (y) per input latitude, longitude coordinate. |
| `x coordinate` | The range or distance along the input locations. It is the cumulative distance along the previous latitiude, longitude coordinates up to the current coordinate. The x-value for the first coordinate in the shape will always be 0. |
| `y coordinate` | The height or elevation of the associated latitude, longitude pair. The height is returned as `null` if no height data exists for a given location. |
| `height` | An array of height for the associated latitude, longitude coordinates. |
| `warnings` (optional) | This array may contain warning objects informing about deprecated request parameters, clamped values etc. | 


## Data sources

Elevation data is obtained from the [Amazon Web Services Public Datasets](https://aws.amazon.com/public-datasets/terrain/). 


