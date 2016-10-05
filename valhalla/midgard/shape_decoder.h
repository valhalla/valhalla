#ifndef VALHALLA_MIDGARD_SHAPE_DECODER_H_
#define VALHALLA_MIDGARD_SHAPE_DECODER_H_

namespace valhalla {
namespace midgard {

template<typename Point>
class Shape7Decoder {
 public:
  Shape7Decoder(const char* begin, const size_t size)
    : begin(begin), end(begin + size) {
  }
  Point pop() {
    lat = next(lat);
    lon = next(lon);
    return Point(typename Point::first_type(double(lon) * 1e-6),
                 typename Point::second_type(double(lat) * 1e-6));
  }
  bool empty() const {
    return begin == end;
  }

 private:
  const char* begin;
  const char* end;
  int32_t lat = 0;
  int32_t lon = 0;

  int32_t next(const int32_t previous) {
    //TODO: a bogus polyline could cause reading from out of bounds
    int32_t byte, shift = 0, result = 0;
    do {
      //take the least significant 7 bits shifted into place
      byte = int32_t(*begin++);
      result |= (byte & 0x7f) << shift;
      shift += 7;
      //if the most significant bit is set there is more to this number
    }while (byte & 0x80);
    //handle the bit flipping and add to previous since its an offset
    return previous + ((result & 1 ? ~result : result) >> 1);
  }
};

template<typename Point>
class Shape5Decoder {
 public:
  Shape5Decoder(const char* begin, const size_t size)
    : begin(begin), end(begin + size) {
  }
  Point pop() {
    lat = next(lat);
    lon = next(lon);
    return Point(typename Point::first_type(double(lon) * 1e-6),
                 typename Point::second_type(double(lat) * 1e-6));
  }
  bool empty() const {
    return begin == end;
  }

 private:
  const char* begin;
  const char* end;
  int32_t lat = 0;
  int32_t lon = 0;

  int32_t next(const int32_t previous) {
    //grab each 5 bits and mask it in where it belongs using the shift
    int byte, shift = 0, result = 0;
    do {
      //TODO: could use a check here for out of bounds
      //which could happen on improper polyline string data
      byte = int32_t(*begin++) - 63;
      result |= (byte & 0x1f) << shift;
      shift += 5;
    }while (byte >= 0x20);
    //handle the bit flipping and add to previous since its an offset
    return previous + (result & 1 ? ~(result >> 1) : (result >> 1));
  }
};

}
}

#endif // VALHALLA_MIDGARD_SHAPE_DECODER_H_
