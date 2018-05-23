#ifndef VALHALLA_MIDGARD_OPENLR_H_
#define VALHALLA_MIDGARD_OPENLR_H_

#include <valhalla/midgard/pointll.h>

#include <assert.h>
#include <bitset>

namespace valhalla {
namespace midgard {
namespace OpenLR {

namespace {

template <typename T, typename std::enable_if<std::is_signed<T>::value, int>::type = 0>
inline int sgn(const T val) {
  return (T(0) < val) - (val < T(0));
}

template <typename T, unsigned B> inline T signextend(const T x) {
  struct {
    T x : B;
  } s;
  return s.x = x;
}

template <typename T, unsigned B> inline double fixed(const unsigned char*& raw) {
  T value = std::uint8_t(*raw++);
  for (unsigned i = 1; i < B; ++i) {
    value = (value << 8) | std::uint8_t(*raw++);
  }
  return signextend<T, B * 8>(value);
}

inline std::int32_t
decimal2integer(const double value) { // p. 44 Equation 1:  Transformation from decimal coordinates
                                      // into integer values (Binary)
  return static_cast<std::int32_t>(sgn(value) * 0.5 + value * (std::int32_t(1) << 24) / 360.);
}

inline double
integer2decimal(const std::int32_t value) { // p. 45 Equation 2: Transformation from integer values
                                            // into decimal coordinates (Binary)
  return (value - sgn(value) * 0.5) * 360. / (std::int32_t(1) << 24);
}

inline std::int32_t
bearing2integer(const float value) { // p. 48 Equation 6: Calculation of the bearing value (Binary)
  return static_cast<std::int32_t>(value / 11.25f);
}

inline float integer2bearing(
    const std::int32_t value) { // p. 48 Equation 6: Calculation of the bearing value (Binary)
  return value * 11.25f + 11.25f / 2.f;
}

inline std::int32_t
distance2integer(const float value) { // p. 48 Equation 7: Calculation of the DNP value (Binary)
  return static_cast<std::int32_t>(value / 58.6f);
}

inline float integer2distance(
    const std::uint32_t value) { // p. 48 Equation 7: Calculation of the DNP value (Binary)
  return value * 58.6f;
}

} // namespace

// Reference: OpenLR™ White Paper Version: 1.5 revision 2
// http://www.openlr.org/data/docs/OpenLR-Whitepaper_v1.5.pdf

// Location Reference Point, p.35, section 5.4
struct LocationReferencePoint {

  enum FormOfWay : unsigned char {
    UNDEFINED = 0,
    MOTORWAY,
    MULTIPLE_CARRIAGEWAY,
    SINGLE_CARRIAGEWAY,
    ROUNDABOUT,
    TRAFFICSQUARE,
    SLIPROAD,
    OTHER
  };
  double longitude;
  double latitude;
  float bearing;        // 5.2.4. Bearing
  float distance;       // 5.2.5. Distance to next LR-point
  unsigned char frc;    // 5.2.2. Functional Road Class: 0 – Main road,  1 – First class road, ...
  unsigned char lfrcnp; // 5.2.6. Lowest FRC to next LR-point
  FormOfWay fow;        // 5.2.3. Form of way
};

// Line locations, p.19, section 3.1
// Only line location with 2 location reference points are supported
struct LineLocation {
  LineLocation(const std::string& reference) {
    //  Line location data size: 16 + (n-2)*7 + [0/1/2] bytes
    if (reference.size() < 16)
      throw std::invalid_argument("OpenLR reference is too small");
    if (reference.size() > 18)
      throw std::invalid_argument("OpenLR references with only 2 LRP are supported");

    auto raw = reinterpret_cast<const unsigned char*>(reference.data());

    // Status
    auto status = *raw++ & 0x7f;
    (void)status;
    assert(status == 0x0b); // version 3, has attributes, ArF 'Circle or no area location'

    // First location reference point
    auto longitude = integer2decimal(fixed<std::int32_t, 3>(raw));
    auto latitude = integer2decimal(fixed<std::int32_t, 3>(raw));
    auto attribute1 = *raw++;
    auto attribute2 = *raw++;
    auto distance = *raw++;

    first = LocationReferencePoint{longitude,
                                   latitude,
                                   integer2bearing(attribute2 & 0x1f),
                                   integer2distance(distance),
                                   static_cast<unsigned char>((attribute1 >> 3) & 0x07),
                                   static_cast<unsigned char>((attribute2 >> 5) & 0x07),
                                   static_cast<LocationReferencePoint::FormOfWay>(attribute1 & 0x07)};

    // Last location reference point
    longitude += fixed<std::int32_t, 2>(raw) / 100000;
    latitude += fixed<std::int32_t, 2>(raw) / 100000;
    attribute1 = *raw++;
    auto attribute4 = *raw++;

    last = LocationReferencePoint{longitude,
                                  latitude,
                                  integer2bearing(attribute4 & 0x1f),
                                  0.f,
                                  static_cast<unsigned char>((attribute1 >> 3) & 0x07),
                                  0,
                                  static_cast<LocationReferencePoint::FormOfWay>(attribute1 & 0x07)};

    // Offsets
    // TODO: add out-of-bounds checks and clarify if noff location is 17th byte
    std::bitset<8> flags(attribute4);
    poff = flags[6] ? first.distance * (*raw++) / 256 : 0.f;
    noff = flags[5] ? first.distance * (*raw++) / 256 : 0.f;
  }

  PointLL getFirstCoordinate() const {
    return {first.longitude, first.latitude};
  }

  PointLL getLastCoordinate() const {
    return {last.longitude, last.latitude};
  }

  std::string to_binary() const {
    std::string result;

    // Status
    result.push_back(0b00001011);

    // First longitude
    auto longitude = decimal2integer(first.longitude);
    result.push_back((longitude >> 16) & 0xff);
    result.push_back((longitude >> 8) & 0xff);
    result.push_back(longitude & 0xff);

    // First latitude
    auto latitude = decimal2integer(first.latitude);
    result.push_back((latitude >> 16) & 0xff);
    result.push_back((latitude >> 8) & 0xff);
    result.push_back(latitude & 0xff);

    // Attributes
    result.push_back(((first.frc & 0x7) << 3) | (first.fow & 0x7));
    result.push_back(((first.lfrcnp & 0x7) << 5) | bearing2integer(first.bearing) & 0x1f);
    result.push_back(distance2integer(first.distance));

    // Last longitude
    auto relative_longitude =
        static_cast<std::int32_t>(std::round(100000 * (last.longitude - first.longitude)));
    result.push_back((relative_longitude >> 8) & 0xff);
    result.push_back(relative_longitude & 0xff);

    // Last latitude
    auto relative_latitude =
        static_cast<std::int32_t>(std::round(100000 * (last.latitude - first.latitude)));
    result.push_back((relative_latitude >> 8) & 0xff);
    result.push_back(relative_latitude & 0xff);

    // Attributes
    result.push_back(((last.frc & 0x7) << 3) | (last.fow & 0x7));

    auto pofff = poff != 0.f;
    auto nofff = noff != 0.f;
    result.push_back((pofff << 6) | (nofff << 5) | bearing2integer(last.bearing) & 0x1f);

    // Offsets
    if (pofff) {
      result.push_back(static_cast<std::int32_t>(256 * poff / first.distance) & 0xff);
    }

    if (nofff) {
      result.push_back(static_cast<std::int32_t>(256 * noff / first.distance) & 0xff);
    }

    return result;
  }

  LocationReferencePoint first;
  LocationReferencePoint last;
  float poff; // 5.2.9.1 Positive offset
  float noff; // 5.2.9.2 Negative offset
};

} // namespace OpenLR
} // namespace midgard
} // namespace valhalla

#endif
