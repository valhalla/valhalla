# Decode a route shape

Valhalla routing, map-matching, and elevation services use an encoded polyline format to store a series of latitude, longitude coordinates as a single string. Polyline encoding greatly reduces the size of the route response or map-matching request, especially for longer routes or GPS traces. A description is found here: [polyline encoding](https://developers.google.com/maps/documentation/utilities/polylinealgorithm).

**Note: Valhalla APIs use six digits of decimal precision.**

It is very important that you use six digits, rather than five as referenced in the Google algorithms documentation. With fewer than six digits, your locations are incorrectly placed (commonly, in the middle of an ocean), and you may receive errors with your API requests.

Below are some sample algorithms to decode the string to create a list of latitude,longitude coordinates. Using this [demo tool](http://valhalla.github.io/demos/polyline/), you can also paste an encoded polyline string, decode it, and see the locations on a map (and save to GeoJSON). Use it to test and verify that your points are placed where you expected them.

## JavaScript

Here is an example of decoding in JavaScript.

``` javascript
// This is adapted from the implementation in Project-OSRM
// https://github.com/DennisOSRM/Project-OSRM-Web/blob/master/WebContent/routing/OSRM.RoutingGeometry.js
polyline.decode = function(str, precision) {
    var index = 0,
        lat = 0,
        lng = 0,
        coordinates = [],
        shift = 0,
        result = 0,
        byte = null,
        latitude_change,
        longitude_change,
        factor = Math.pow(10, precision || 6);

    // Coordinates have variable length when encoded, so just keep
    // track of whether we've hit the end of the string. In each
    // loop iteration, a single coordinate is decoded.
    while (index < str.length) {

        // Reset shift, result, and byte
        byte = null;
        shift = 0;
        result = 0;

        do {
            byte = str.charCodeAt(index++) - 63;
            result |= (byte & 0x1f) << shift;
            shift += 5;
        } while (byte >= 0x20);

        latitude_change = ((result & 1) ? ~(result >> 1) : (result >> 1));

        shift = result = 0;

        do {
            byte = str.charCodeAt(index++) - 63;
            result |= (byte & 0x1f) << shift;
            shift += 5;
        } while (byte >= 0x20);

        longitude_change = ((result & 1) ? ~(result >> 1) : (result >> 1));

        lat += latitude_change;
        lng += longitude_change;

        coordinates.push([lat / factor, lng / factor]);
    }

    return coordinates;
};

```

## C++ 11

Here is an example of decoding in C++11

``` c++
#include <vector>

constexpr double kPolylinePrecision = 1E6;
constexpr double kInvPolylinePrecision = 1.0 / kPolylinePrecision;

struct PointLL {
  float lat;
  float lon;
};

std::vector<PointLL> decode(const std::string& encoded) {
  size_t i = 0;     // what byte are we looking at

  // Handy lambda to turn a few bytes of an encoded string into an integer
  auto deserialize = [&encoded, &i](const int previous) {
    // Grab each 5 bits and mask it in where it belongs using the shift
    int byte, shift = 0, result = 0;
    do {
      byte = static_cast<int>(encoded[i++]) - 63;
      result |= (byte & 0x1f) << shift;
      shift += 5;
    } while (byte >= 0x20);
    // Undo the left shift from above or the bit flipping and add to previous
    // since its an offset
    return previous + (result & 1 ? ~(result >> 1) : (result >> 1));
  };

  // Iterate over all characters in the encoded string
  std::vector<PointLL> shape;
  int last_lon = 0, last_lat = 0;
  while (i < encoded.length()) {
    // Decode the coordinates, lat first for some reason
    int lat = deserialize(last_lat);
    int lon = deserialize(last_lon);

    // Shift the decimal point 5 places to the left
    shape.emplace_back(static_cast<float>(static_cast<double>(lat) *
                                          kInvPolylinePrecision),
                       static_cast<float>(static_cast<double>(lon) *
                                          kInvPolylinePrecision));

    // Remember the last one we encountered
    last_lon = lon;
    last_lat = lat;
  }
  return shape;
}
```

## Python

Here is an example of decoding in Python

``` python
#!/usr/bin/env python

import sys

#six degrees of precision in valhalla
inv = 1.0 / 1e6;

#decode an encoded string
def decode(encoded):
  decoded = []
  previous = [0,0]
  i = 0
  #for each byte
  while i < len(encoded):
    #for each coord (lat, lon)
    ll = [0,0]
    for j in [0, 1]:
      shift = 0
      byte = 0x20
      #keep decoding bytes until you have this coord
      while byte >= 0x20:
        byte = ord(encoded[i]) - 63
        i += 1
        ll[j] |= (byte & 0x1f) << shift
        shift += 5
      #get the final value adding the previous offset and remember it for the next
      ll[j] = previous[j] + (~(ll[j] >> 1) if ll[j] & 1 else (ll[j] >> 1))
      previous[j] = ll[j]
    #scale by the precision and chop off long coords also flip the positions so
    #its the far more standard lon,lat instead of lat,lon
    decoded.append([float('%.6f' % (ll[1] * inv)), float('%.6f' % (ll[0] * inv))])
  #hand back the list of coordinates
  return decoded

print decode(sys.argv[1])
```
