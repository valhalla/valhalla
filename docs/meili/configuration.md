# Configuration

To launch a
[Meili service](service_api.md)
or instantiate a
[`MapMatcherFactor`](library_api.md#map-matcher-factory),
you need to pass it the
[Valhalla configuration file](https://github.com/valhalla/conf), which
holds all configurations for Meili at the node `meili`.

## Map Matching Parameters

The map matching parameters control accuracy and performance of the
map matching process. All transport mode nodes (`auto`, `pedestrian`,
`bicycle`, `multimodal`) can hold its own settings of these
parameters, otherwise the setting in `default` node will be used.

All transport modes can specify following parameters:

Parameters                  | Description                                                                                                                        | Default
----------------------------|------------------------------------------------------------------------------------------------------------------------------------|-----
`sigma_ z`                  | A non-negative value to specify the GPS accuracy (the variance of the normal distribution) of an incoming GPS sequence. It is also used to weight emission costs of measurements.  | 4.07
`beta`                      | A non-negative emprical value to weight the transition cost of two successive candidates.                                                      | 3
`max_route_distance_factor` | A non-negative value used to limit the routing search range which is the distance to next measurement multiplied by this factor.               | 5
`max_route_time_factor` | A non-negative value used to limit the routing search range which is the time to next measurement multiplied by this factor.               | 5
`breakage_distance`         | A non-negative value. If two successive measurements are far than this distance, then connectivity in between will not be considered.          | 2000 (meters)
`interpolation_distance`    | If two successive measurements are closer than this distance, then the later one will be interpolated into the matched route.                   | 10 (meters)
`search_radius`             | A non-negative value to specify the search radius (in meters) within which to search road candidates for each measurement.                     | 50 (meters)
`max_search_radius`         | Specify the upper bound of `search_radius`                                                                                                      | 100 (meters)
`turn_penalty_factor`       | A non-negative value to penalize turns from one road segment to next.                                                                          | 0 (meters)

## Service Parameters

The service parameters below are only used in the Meili service:

Parameters                  | Description                                                                                                                        | Default
----------------------------|------------------------------------------------------------------------------------------------------------------------------------|-----
`mode`                      | Specify the default transport mode.                                                                                                | `multimodal`
`customizable`              | Specify which parameters are allowed to be customized by URL query parameters.                                                     | `["mode", "search_radius"]`
`verbose`                   | Control verbose output for debugging.                                                                                              | `false`
