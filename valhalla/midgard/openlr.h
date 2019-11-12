#ifndef VALHALLA_MIDGARD_OPENLR_H_
#define VALHALLA_MIDGARD_OPENLR_H_

#include <valhalla/midgard/pointll.h>

#include <assert.h>
#include <bitset>
#include <numeric>
#include <vector>

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

template <typename T, unsigned B> inline double fixed(const unsigned char* raw, std::size_t& index) {
  T value = std::uint8_t(raw[index++]);
  for (unsigned i = 1; i < B; ++i) {
    value = (value << 8) | std::uint8_t(raw[index++]);
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

  LocationReferencePoint() = default;

  LocationReferencePoint(double longitude,
                         double latitude,
                         unsigned char attribute1,
                         unsigned char attribute2,
                         unsigned char attribute3)
      : longitude(longitude), latitude(latitude), bearing(integer2bearing(attribute2 & 0x1f)),
        distance(integer2distance(attribute3)),
        frc(static_cast<unsigned char>((attribute1 >> 3) & 0x07)),
        lfrcnp(static_cast<unsigned char>((attribute2 >> 5) & 0x07)),
        fow(static_cast<LocationReferencePoint::FormOfWay>(attribute1 & 0x07)) {
  }

  LocationReferencePoint(double longitude,
                         double latitude,
                         unsigned char attribute1,
                         unsigned char attribute4)
      : longitude(longitude), latitude(latitude), bearing(integer2bearing(attribute4 & 0x1f)),
        distance(0.f), frc(static_cast<unsigned char>((attribute1 >> 3) & 0x07)), lfrcnp(0),
        fow(static_cast<LocationReferencePoint::FormOfWay>(attribute1 & 0x07)) {
  }

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

    const auto raw = reinterpret_cast<const unsigned char*>(reference.data());
    const auto size = reference.size();
    std::size_t index = 0;

    // Status, version 3, has attributes, ArF 'Circle or no area location'
    auto status = raw[index++] & 0x7f;
    if (status != 0x0b)
      throw std::invalid_argument("OpenLR reference ");

    // First location reference point
    auto longitude = integer2decimal(fixed<std::int32_t, 3>(raw, index));
    auto latitude = integer2decimal(fixed<std::int32_t, 3>(raw, index));
    auto attribute1 = raw[index++];
    auto attribute2 = raw[index++];
    auto attribute3 = raw[index++];

    first = {longitude, latitude, attribute1, attribute2, attribute3};

    // Intermediate location reference points
    while (index + 7 + 6 <= size) {
      longitude += fixed<std::int32_t, 2>(raw, index) / 100000;
      latitude += fixed<std::int32_t, 2>(raw, index) / 100000;
      attribute1 = raw[index++];
      attribute2 = raw[index++];
      attribute3 = raw[index++];

      intermediate.push_back({longitude, latitude, attribute1, attribute2, attribute3});
    }

    // Last location reference point
    longitude += fixed<std::int32_t, 2>(raw, index) / 100000;
    latitude += fixed<std::int32_t, 2>(raw, index) / 100000;
    attribute1 = raw[index++];
    auto attribute4 = raw[index++];

    last = {longitude, latitude, attribute1, attribute4};

    // Offsets
    std::bitset<8> flags(attribute4);
    poff = (index < size) && flags[6] ? first.distance * raw[index++] / 256 : 0.f;
    noff = (index < size) && flags[5] ? first.distance * raw[index++] / 256 : 0.f;
  }

  PointLL getFirstCoordinate() const {
    return {first.longitude, first.latitude};
  }

  PointLL getLastCoordinate() const {
    return {last.longitude, last.latitude};
  }

  float getLength() const {
    return std::accumulate(intermediate.begin(), intermediate.end(), first.distance,
                           [](float distance, const LocationReferencePoint& lrp) {
                             return distance + lrp.distance;
                           });
  }

  std::string toBinary() const {
    std::string result;

    const auto append2 = [&result](std::int32_t value) {
      result.push_back((value >> 8) & 0xff);
      result.push_back(value & 0xff);
    };

    const auto append3 = [&result](std::int32_t value) {
      result.push_back((value >> 16) & 0xff);
      result.push_back((value >> 8) & 0xff);
      result.push_back(value & 0xff);
    };

    // Status
    result.push_back(0b00001011);

    // First location reference point
    auto longitude = first.longitude;
    auto latitude = first.latitude;
    append3(decimal2integer(longitude));
    append3(decimal2integer(latitude));
    result.push_back(((first.frc & 0x7) << 3) | (first.fow & 0x7));
    result.push_back(((first.lfrcnp & 0x7) << 5) | (bearing2integer(first.bearing) & 0x1f));
    result.push_back(distance2integer(first.distance));

    for (const auto& lrp : intermediate) {
      // First longitude
      append2(static_cast<std::int32_t>(std::round(100000 * (lrp.longitude - longitude))));
      append2(static_cast<std::int32_t>(std::round(100000 * (lrp.latitude - latitude))));
      result.push_back(((lrp.frc & 0x7) << 3) | (lrp.fow & 0x7));
      result.push_back(((lrp.lfrcnp & 0x7) << 5) | (bearing2integer(lrp.bearing) & 0x1f));
      result.push_back(distance2integer(lrp.distance));

      longitude = lrp.longitude;
      latitude = lrp.latitude;
    }

    // Last location reference point
    const auto pofff = poff != 0.f;
    const auto nofff = noff != 0.f;
    append2(static_cast<std::int32_t>(std::round(100000 * (last.longitude - longitude))));
    append2(static_cast<std::int32_t>(std::round(100000 * (last.latitude - latitude))));
    result.push_back(((last.frc & 0x7) << 3) | (last.fow & 0x7));
    result.push_back((pofff << 6) | (nofff << 5) | (bearing2integer(last.bearing) & 0x1f));

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
  std::vector<LocationReferencePoint> intermediate;
  float poff; // 5.2.9.1 Positive offset
  float noff; // 5.2.9.2 Negative offset
};

} // namespace OpenLR
} // namespace midgard
} // namespace valhalla

#endif
