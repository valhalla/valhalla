#pragma once

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

// we store 6 digits of precision in the tiles, changing to 7 digits is a breaking change
// if you want to try out 7 digits of precision you can uncomment this definition
// #define USE_7DIGITS_DEFAULT
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

// Move sign bit to the least significant bit, making negative numbers use fewer bits when encoded.
template <typename T> inline auto zigzag_encode(T value) -> typename std::make_unsigned<T>::type {
  static_assert(std::is_signed_v<T>, "zigzag_encode requires signed integer type");
  using unsigned_t = typename std::make_unsigned<T>::type;
  constexpr int shift_bits = sizeof(T) * 8 - 1;
  return static_cast<unsigned_t>(value << 1) ^ static_cast<unsigned_t>(value >> shift_bits);
}

// Restore sign bit from least significant bit.
template <typename T> inline auto zigzag_decode(T value) -> typename std::make_signed<T>::type {
  static_assert(std::is_unsigned_v<T>, "zigzag_decode requires unsigned integer type");
  using signed_t = typename std::make_signed<T>::type;
  return static_cast<signed_t>(value >> 1) ^ (-static_cast<signed_t>(value & 1));
}

template <typename Point> class Shape7Decoder {
public:
  Shape7Decoder(const char* begin, const size_t size, const double precision = DECODE_PRECISION)
      : begin(begin), end(begin + size), prec(precision) {
  }
  Point pop() noexcept(false) {
    auto lat_diff = read_varint();
    auto lon_diff = read_varint();
    lat += zigzag_decode(lat_diff);
    lon += zigzag_decode(lon_diff);
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

  uint32_t read_varint() noexcept(false) {
    uint32_t byte, shift = 0, result = 0;
    do {
      if (empty()) {
        throw std::runtime_error("Bad encoded polyline");
      }
      // take the least significant 7 bits shifted into place
      byte = static_cast<uint32_t>(*begin++);
      result |= (byte & 0x7f) << shift;
      shift += 7;
      // if the most significant bit is set there is more to this number
    } while (byte & 0x80);
    return result;
  }
};

template <typename value_type> class Int7Decoder {
public:
  Int7Decoder(const char* begin, const size_t size) : begin(begin), end(begin + size) {
  }
  value_type pop() noexcept(false) {
    auto diff = read_varint();
    value += zigzag_decode(diff);
    return value;
  }
  bool empty() const {
    return begin == end;
  }

private:
  const char* begin;
  const char* end;
  value_type value = 0;

  using uvalue_t = typename std::make_unsigned<value_type>::type;

  uvalue_t read_varint() noexcept(false) {
    uvalue_t byte, shift = 0, result = 0;
    do {
      if (empty()) {
        throw std::runtime_error("Bad varint offset encoding");
      }
      // take the least significant 7 bits shifted into place
      byte = static_cast<uvalue_t>(static_cast<unsigned char>(*begin++));
      result |= (byte & 0x7f) << shift;
      shift += 7;
      // if the most significant bit is set there is more to this number
    } while (byte & 0x80);
    return result;
  }
};

template <typename Point> class Shape5Decoder {
public:
  Shape5Decoder(const char* begin, const size_t size, const double precision = DECODE_PRECISION)
      : begin(begin), end(begin + size), prec(precision) {
  }
  Point pop() noexcept(false) {
    auto lat_diff = read_varint();
    auto lon_diff = read_varint();
    lat += zigzag_decode(lat_diff);
    lon += zigzag_decode(lon_diff);
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

  uint32_t read_varint() noexcept(false) {
    // grab each 5 bits and mask it in where it belongs using the shift
    uint32_t byte, shift = 0, result = 0;
    do {
      if (empty()) {
        throw std::runtime_error("Bad encoded polyline");
      }
      // take the least significant 5 bits shifted into place
      byte = uint32_t(*begin++) - 63;
      result |= (byte & 0x1f) << shift;
      shift += 5;
      // if the most significant bit is set there is more to this number
    } while (byte >= 0x20);
    return result;
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

// specialized implementation for std::vector with reserve
template <class container_t, class IntDecoder = Int7Decoder<typename container_t::value_type>>
typename std::enable_if<
    std::is_same<std::vector<typename container_t::value_type>, container_t>::value,
    container_t>::type
decode7int(const char* encoded, size_t length) {
  IntDecoder decoder(encoded, length);
  container_t c;
  c.reserve(length / 8);
  while (!decoder.empty()) {
    c.emplace_back(decoder.pop());
  }
  return c;
}

/**
 * Varint decode a string into a container of points
 *
 * @param encoded    the encoded points
 * @param precision  the multiplier to turn the encoded integers back into floating point default 1e-6
 * @return points   the container of points
 */
template <class container_t>
container_t decode7(const std::string& encoded, const double precision = DECODE_PRECISION) {
  return decode7<container_t>(encoded.c_str(), encoded.length(), precision);
}

/**
 * Varint decode a string into a container of integral values
 *
 * @param encoded  the encoded points in string form
 * @return integer values in the templated container type
 */
template <class container_t> container_t decode7int(const std::string& encoded) {
  return decode7int<container_t>(encoded.c_str(), encoded.length());
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
  auto write_varint = [&output](uint32_t number) {
    // write 5 bit chunks of the number
    while (number >= 0x20) {
      auto nextValue = (0x20 | (number & 0x1f)) + 63;
      output.push_back(static_cast<char>(nextValue));
      number >>= 5;
    }
    // write the last chunk
    output.push_back(static_cast<char>(number + 63));
  };

  // this is an diff encoding so we remember the last point we saw
  int32_t last_lon = 0, last_lat = 0;
  // for each point
  for (const auto& p : points) {
    // shift the decimal point 5 places to the right and truncate
    auto lon = static_cast<int32_t>(round(static_cast<double>(p.first) * precision));
    auto lat = static_cast<int32_t>(round(static_cast<double>(p.second) * precision));
    // encode each coordinate, lat first for some reason
    write_varint(zigzag_encode(lat - last_lat));
    write_varint(zigzag_encode(lon - last_lon));
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
  auto write_varint = [&output](uint32_t number) {
    // we take 7 bits of this at a time
    while (number > 0x7f) {
      auto nextValue = (0x80 | (number & 0x7f));
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
    auto lon = static_cast<int32_t>(round(static_cast<double>(p.first) * precision));
    auto lat = static_cast<int32_t>(round(static_cast<double>(p.second) * precision));
    // encode each coordinate, lat first for some reason
    write_varint(zigzag_encode(lat - last_lat));
    write_varint(zigzag_encode(lon - last_lon));
    // remember the last one we encountered
    last_lon = lon;
    last_lat = lat;
  }
  return output;
}

template <class container_t> std::string encode7int(const container_t& values) {
  // a place to keep the output
  using value_t = typename container_t::value_type;
  using uvalue_t = typename std::make_unsigned<value_t>::type;
  std::string output;
  output.reserve(values.size() * 8);

  // handy lambda to turn an integer into an encoded string
  auto write_varint = [&output](uvalue_t number) {
    while (number > 0x7f) {
      auto nextValue = static_cast<std::string::value_type>(0x80 | (number & 0x7f));
      output.push_back(nextValue);
      number >>= 7;
    }
    output.push_back(static_cast<std::string::value_type>(number & 0x7f));
  };

  // this is an offset encoding so we remember the last value we saw
  value_t last_value = 0;
  for (const auto& value : values) {
    auto diff = static_cast<typename std::make_signed<value_t>::type>(value - last_value);
    write_varint(zigzag_encode(diff));
    last_value = value;
  }
  return output;
}

} // namespace midgard
} // namespace valhalla
