#pragma once

#include <cmath>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

// we store 6 digits of precision in the tiles, changing to 7 digits is a breaking change
// if you want to try out 7 digits of precision you can uncomment this definition
//#define USE_7DIGITS_DEFAULT
#ifdef USE_7DIGITS_DEFAULT
constexpr double DECODE_PRECISION = 1e-7;
constexpr int ENCODE_PRECISION = 1e7;
constexpr size_t DIGITS_PRECISION = 7;
#else
constexpr double DECODE_PRECISION = 1e-6;
constexpr int ENCODE_PRECISION = 1e6;
constexpr size_t DIGITS_PRECISION = 6;
#endif

namespace valhalla {
namespace midgard {

template <typename Point> class Shape7Decoder {
public:
  Shape7Decoder(const char* begin, const size_t size, const double precision = DECODE_PRECISION)
      : begin(begin), end(begin + size), prec(precision) {
  }
  Point pop() noexcept(false) {
    lat = next(lat);
    lon = next(lon);
    return Point(double(lon) * prec, double(lat) * prec);
  }
  bool empty() const {
    return begin == end;
  }

private:
  const char* begin;
  const char* end;
  int32_t lat = 0;
  int32_t lon = 0;
  double prec;

  int32_t next(const int32_t previous) noexcept(false) {
    int32_t byte, shift = 0, result = 0;
    do {
      if (empty()) {
        throw std::runtime_error("Bad encoded polyline");
      }
      // take the least significant 7 bits shifted into place
      byte = int32_t(*begin++);
      result |= (byte & 0x7f) << shift;
      shift += 7;
      // if the most significant bit is set there is more to this number
    } while (byte & 0x80);
    // handle the bit flipping and add to previous since its an offset
    return previous + ((result & 1 ? ~result : result) >> 1);
  }
};

template <typename Point> class Shape5Decoder {
public:
  Shape5Decoder(const char* begin, const size_t size, const double precision = DECODE_PRECISION)
      : begin(begin), end(begin + size), prec(precision) {
  }
  Point pop() noexcept(false) {
    lat = next(lat);
    lon = next(lon);
    return Point(double(lon) * prec, double(lat) * prec);
  }
  bool empty() const {
    return begin == end;
  }

private:
  const char* begin;
  const char* end;
  int32_t lat = 0;
  int32_t lon = 0;
  double prec;

  int32_t next(const int32_t previous) noexcept(false) {
    // grab each 5 bits and mask it in where it belongs using the shift
    int byte, shift = 0, result = 0;
    do {
      if (empty()) {
        throw std::runtime_error("Bad encoded polyline");
      }
      // take the least significant 5 bits shifted into place
      byte = int32_t(*begin++) - 63;
      result |= (byte & 0x1f) << shift;
      shift += 5;
      // if the most significant bit is set there is more to this number
    } while (byte >= 0x20);
    // handle the bit flipping and add to previous since its an offset
    return previous + (result & 1 ? ~(result >> 1) : (result >> 1));
  }
};

// specialized implementation for std::vector with reserve
template <class container_t, class ShapeDecoder = Shape5Decoder<typename container_t::value_type>>
typename std::enable_if<
    std::is_same<std::vector<typename container_t::value_type>, container_t>::value,
    container_t>::type
decode(const char* encoded, size_t length, const double precision = DECODE_PRECISION) {
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
decode(const char* encoded, size_t length, const double precision = DECODE_PRECISION) {
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
container_t decode(const std::string& encoded, const double precision = DECODE_PRECISION) {
  return decode<container_t, ShapeDecoder>(encoded.c_str(), encoded.length(), precision);
}

template <class container_t>
container_t decode7(const char* encoded, size_t length, const double precision = DECODE_PRECISION) {
  return decode<container_t, Shape7Decoder<typename container_t::value_type>>(encoded, length,
                                                                              precision);
}

/**
 * Varint decode a string into a container of points
 *
 * @param encoded    the encoded points
 * @return points   the container of points
 */
template <class container_t>
container_t decode7(const std::string& encoded, const double precision = DECODE_PRECISION) {
  return decode7<container_t>(encoded.c_str(), encoded.length(), precision);
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
std::string encode(const container_t& points, const int precision = ENCODE_PRECISION) {
  // a place to keep the output
  std::string output;
  // unless the shape is very course you should probably only need about 3 bytes
  // per coord, which is 6 bytes with 2 coords, so we overshoot to 8 just in case
  output.reserve(points.size() * 8);

  // handy lambda to turn an integer into an encoded string
  auto serialize = [&output](int number) {
    // move the bits left 1 position and flip all the bits if it was a negative number
    number = number < 0 ? ~(static_cast<unsigned int>(number) << 1) : (number << 1);
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
    int lon = static_cast<int>(round(static_cast<double>(p.first) * precision));
    int lat = static_cast<int>(round(static_cast<double>(p.second) * precision));
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
template <class container_t>
std::string encode7(const container_t& points, const int precision = ENCODE_PRECISION) {
  // a place to keep the output
  std::string output;
  // unless the shape is very course you should probably only need about 3 bytes
  // per coord, which is 6 bytes with 2 coords, so we overshoot to 8 just in case
  output.reserve(points.size() * 8);

  // handy lambda to turn an integer into an encoded string
  auto serialize = [&output](int number) {
    // get the sign bit down on the least significant end to
    // make the most significant bits mostly zeros
    number = number < 0 ? ~(static_cast<unsigned int>(number) << 1) : number << 1;
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
    int lon = static_cast<int>(round(static_cast<double>(p.first) * precision));
    int lat = static_cast<int>(round(static_cast<double>(p.second) * precision));
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
