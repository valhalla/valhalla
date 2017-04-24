# Service API

*The API is still in testing and will be changed any time. Any
 suggestions are welcome to share at
 [GitHub Issues](https://github.com/valhalla/valhalla/issues).*

## Request

The service accepts a GeoJSON feature or geometry (of type
[`MultiPoint`](http://geojson.org/geojson-spec.html#multipoint) or
[`LineString`](http://geojson.org/geojson-spec.html#linestring)) in
the `POST` request body.

URL Parameter            | Description                                                                                                                        | Default
------------------------ | ---------------------------------------------------------------------------------------------------------------------------------- | ----------
`mode`                   | Transport mode of the sequence. Possible modes are: `auto`, `bycicle`, `pedestrian` and `multimodal`.                              | `multimodal`
`search_radius`          | A numeric value in range `[0, 100]` to specify a radius (in meters) within which to search road candidates for each measurement.   | 40

* Specifying a transport mode can limit the type of roads to match
  (e.g. `auto` will only consider drivable roads), and therefore
  improve the matching accuracy and speed. If the mode is unknown, use
  the default `multimodal`, i.e. consider all types of roads,
  even a
  [tree row](http://wiki.openstreetmap.org/wiki/Tag:natural%3Dtree_row).

* When GPS accuracy information is unknown, specifying a large
  `search_radius` may slow down the matching procedure while a small
  one may miss possible road candidates.

## Response

The service returns matched routes of the sequence as a
[GeoJSON `MultiLineString` feature](http://geojson.org/geojson-spec.html#multilinestring). Matched
coordinates are saved in the property `matched_coordinates` as a JSON
array. If a measurement is not matched to any road, then the
corresponding matched coordinate is `null`.


## Examples

<!-- Example at #loc=19,52.439056,13.288740 -->

Example request:

```sh
curl -X POST "http://localhost:8002?search_radius=35&mode=auto"
```

Example request body:
```JSON
{
  "coordinates": [
    [ 13.288925, 52.438512 ],
    [ 13.288938, 52.438938 ],
    [ 13.288904, 52.439169 ],
    [ 13.288821, 52.439398 ],
    [ 13.288824, 52.439491 ],
    [ 13.288824, 52.439563 ]
  ]
}
```

Example response:
```JSON
{
  "status": 200,
  "message": "OK",
  "data": {
    "type": "Feature",
    "geometry": {
      "type": "MultiLineString",
      "coordinates": [
        [
          [ 13.288884, 52.438507 ],
          [ 13.288852, 52.438835 ],
          [ 13.288844, 52.439090 ],
          [ 13.288825, 52.439136 ],
          [ 13.288805, 52.439159 ],
          [ 13.288601, 52.439365 ],
          [ 13.288538, 52.439384 ],
          [ 13.288719, 52.439636 ]
        ]
      ]
    },
    "properties": {
      "matched_coordinates": [
        [ 13.288884, 52.438507 ],
        [ 13.288848, 52.438934 ],
        [ 13.288805, 52.439159 ],
        [ 13.288601, 52.439365 ],
        [ 13.288685, 52.439590 ],
        [ 13.288719, 52.439640 ]
      ]
    }
  }
}
```
