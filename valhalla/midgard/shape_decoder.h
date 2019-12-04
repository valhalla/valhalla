#ifndef VALHALLA_MIDGARD_SHAPE_DECODER_H_
#define VALHALLA_MIDGARD_SHAPE_DECODER_H_

#include <stdexcept>

namespace valhalla {
namespace midgard {

template <typename Point> class Shape7Decoder {
public:
  Shape7Decoder(const char* begin, const size_t size, const int = 7)
      : begin(begin), end(begin + size) {
  }
  Point pop() noexcept(false) {
    lat = next(lat);
    lon = next(lon);
    return Point(double(lon) * 1e-6, double(lat) * 1e-6);
  }
  bool empty() const {
    return begin == end;
  }

private:
  const char* begin;
  const char* end;
  int32_t lat = 0;
  int32_t lon = 0;

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
  Shape5Decoder(const char* begin, const size_t size, const double precision = 1e-6)
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

} // namespace midgard
} // namespace valhalla

#endif // VALHALLA_MIDGARD_SHAPE_DECODER_H_
