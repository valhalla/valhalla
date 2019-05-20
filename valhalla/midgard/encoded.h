#ifndef VALHALLA_MIDGARD_ENCODED_H_
#define VALHALLA_MIDGARD_ENCODED_H_

#include <valhalla/midgard/shape_decoder.h>

#include <cmath>
#include <string>
#include <type_traits>
#include <vector>

namespace valhalla {
namespace midgard {

// specialized implementation for std::vector with reserve
template <class container_t, class ShapeDecoder = Shape5Decoder<typename container_t::value_type>>
typename std::enable_if<
    std::is_same<std::vector<typename container_t::value_type>, container_t>::value,
    container_t>::type
decode(const char* encoded, size_t length, const double precision = 1e-6) {
  ShapeDecoder shape(encoded, length, precision);
  container_t c;
  c.reserve(length / 4);
  while (!shape.empty()) {
    c.emplace_back(shape.pop());
  }
  return c;
}

// implementation for non std::vector
template <class container_t, class ShapeDecoder = Shape5Decoder<typename container_t::value_type>>
typename std::enable_if<
    !std::is_same<std::vector<typename container_t::value_type>, container_t>::value,
    container_t>::type
decode(const char* encoded, size_t length, const double precision = 1e-6) {
  ShapeDecoder shape(encoded, length, precision);
  container_t c;
  while (!shape.empty()) {
    c.emplace_back(shape.pop());
  }
  return c;
}

/**
 * Polyline decode a string into a container of points
 *
 * @param encoded    the encoded points
 * @param precision  decoding precision (1/encoding precision)
 * @return points   the container of points
 */
template <class container_t, class ShapeDecoder = Shape5Decoder<typename container_t::value_type>>
container_t decode(const std::string& encoded, const double precision = 1e-6) {
  return decode<container_t, ShapeDecoder>(encoded.c_str(), encoded.length(), precision);
}

template <class container_t>
container_t decode7(const char* encoded, size_t length, const double precision = 1e-7) {
  return decode<container_t, Shape7Decoder<typename container_t::value_type>>(encoded, length,
                                                                              precision);
}

/**
 * Varint decode a string into a container of points
 *
 * @param encoded    the encoded points
 * @return points   the container of points
 */
template <class container_t> container_t decode7(const std::string& encoded) {
  return decode7<container_t>(encoded.c_str(), encoded.length());
}

/**
 * Polyline encode a container of points into a string suitable for web use
 * Note: newer versions of this algorithm allow one to specify a zoom level
 * which allows displaying simplified versions of the encoded linestring
 *
 * @param points    the list of points to encode
 * @param precision Precision of the encoded polyline. Defaults to 6 digit precision.
 * @return string   the encoded container of points
 */
template <class container_t>
std::string encode(const container_t& points, const int precision = 1e6) {
  // a place to keep the output
  std::string output;
  // unless the shape is very course you should probably only need about 3 bytes
  // per coord, which is 6 bytes with 2 coords, so we overshoot to 8 just in case
  output.reserve(points.size() * 8);

  // handy lambda to turn an integer into an encoded string
  auto serialize = [&output](int number) {
    // move the bits left 1 position and flip all the bits if it was a negative number
    number = number < 0 ? ~(number << 1) : (number << 1);
    // write 5 bit chunks of the number
    while (number >= 0x20) {
      int nextValue = (0x20 | (number & 0x1f)) + 63;
      output.push_back(static_cast<char>(nextValue));
      number >>= 5;
    }
    // write the last chunk
    number += 63;
    output.push_back(static_cast<char>(number));
  };

  // this is an offset encoding so we remember the last point we saw
  int last_lon = 0, last_lat = 0;
  // for each point
  for (const auto& p : points) {
    // shift the decimal point 5 places to the right and truncate
    int lon = static_cast<int>(floor(static_cast<double>(p.first) * precision));
    int lat = static_cast<int>(floor(static_cast<double>(p.second) * precision));
    // encode each coordinate, lat first for some reason
    serialize(lat - last_lat);
    serialize(lon - last_lon);
    // remember the last one we encountered
    last_lon = lon;
    last_lat = lat;
  }
  return output;
}

/**
 * Varint encode a container of points into a string
 *
 * @param points    the list of points to encode
 * @return string   the encoded container of points
 */
template <class container_t> std::string encode7(const container_t& points) {
  // a place to keep the output
  std::string output;
  // unless the shape is very course you should probably only need about 3 bytes
  // per coord, which is 6 bytes with 2 coords, so we overshoot to 8 just in case
  output.reserve(points.size() * 8);

  // handy lambda to turn an integer into an encoded string
  auto serialize = [&output](int number) {
    // get the sign bit down on the least significant end to
    // make the most significant bits mostly zeros
    number = number < 0 ? ~(number << 1) : number << 1;
    // we take 7 bits of this at a time
    while (number > 0x7f) {
      // marking the most significant bit means there are more pieces to come
      int nextValue = (0x80 | (number & 0x7f));
      output.push_back(static_cast<char>(nextValue));
      number >>= 7;
    }
    // write the last chunk
    output.push_back(static_cast<char>(number & 0x7f));
  };

  // this is an offset encoding so we remember the last point we saw
  int last_lon = 0, last_lat = 0;
  // for each point
  for (const auto& p : points) {
    // shift the decimal point x places to the right and truncate
    int lon = static_cast<int>(floor(static_cast<double>(p.first) * 1e6));
    int lat = static_cast<int>(floor(static_cast<double>(p.second) * 1e6));
    // encode each coordinate, lat first for some reason
    serialize(lat - last_lat);
    serialize(lon - last_lon);
    // remember the last one we encountered
    last_lon = lon;
    last_lat = lat;
  }
  return output;
}

} // namespace midgard
} // namespace valhalla

#endif // VALHALLA_MIDGARD_ENCODED_H_
