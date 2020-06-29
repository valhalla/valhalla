#ifndef VALHALLA_MIDGARD_OPENLR_H_
#define VALHALLA_MIDGARD_OPENLR_H_

#include <valhalla/midgard/pointll.h>

#include <assert.h>
#include <bitset>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace valhalla {
namespace midgard {
namespace OpenLR {

namespace {

constexpr double geom_scale = 100000;

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

  /**
   * Useful for generating an openlr record from non-openlr data. You can create LRPs with this
   * constructor and then feed them to the special constructor for the LineLocation
   * @param longitude
   * @param latitude
   * @param bearing
   * @param frc
   * @param fow
   * @param distance
   * @param lfrcnp
   */
  LocationReferencePoint(double longitude,
                         double latitude,
                         float bearing,
                         unsigned char frc,
                         FormOfWay fow,
                         LocationReferencePoint* prev,
                         float distance = 0.f,
                         unsigned char lfrcnp = 0)
      : longitude(!prev ? integer2decimal(decimal2integer(longitude))
                        : prev->longitude +
                              (std::round((longitude - prev->longitude) * geom_scale) / geom_scale)),
        latitude(!prev ? integer2decimal(decimal2integer(latitude))
                       : prev->latitude +
                             (std::round((latitude - prev->latitude) * geom_scale) / geom_scale)),
        bearing(integer2bearing(bearing2integer(bearing))), frc(frc), fow(fow),
        distance(integer2distance(distance2integer(distance))), lfrcnp(lfrcnp) {
  }

  /**
   * Strict equality here is ok because the floating point values came from integers
   * @param lrp
   * @return true if the provided lrp exactly matchs this lrp
   */
  bool operator==(const LocationReferencePoint& lrp) const {
    return longitude == lrp.longitude && latitude == lrp.latitude && bearing == lrp.bearing &&
           distance == lrp.distance && frc == lrp.frc && lfrcnp == lrp.lfrcnp && fow == lrp.fow;
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
      throw std::invalid_argument("OpenLR reference is too small " + reference);

    const auto raw = reinterpret_cast<const unsigned char*>(reference.data());
    const auto size = reference.size();
    std::size_t index = 0;

    // Status, version 3, has attributes, ArF 'Circle or no area location'
    auto status = raw[index++] & 0x7f;
    if (status != 0x0b)
      throw std::invalid_argument("OpenLR reference invalid status " + reference);

    // First location reference point
    auto longitude = integer2decimal(fixed<std::int32_t, 3>(raw, index));
    auto latitude = integer2decimal(fixed<std::int32_t, 3>(raw, index));
    auto attribute1 = raw[index++];
    auto attribute2 = raw[index++];
    auto attribute3 = raw[index++];

    lrps.emplace_back(longitude, latitude, attribute1, attribute2, attribute3);

    // Intermediate location reference points
    while (index + 7 + 6 <= size) {
      longitude += fixed<std::int32_t, 2>(raw, index) / geom_scale;
      latitude += fixed<std::int32_t, 2>(raw, index) / geom_scale;
      attribute1 = raw[index++];
      attribute2 = raw[index++];
      attribute3 = raw[index++];

      lrps.emplace_back(longitude, latitude, attribute1, attribute2, attribute3);
    }

    // Last location reference point
    longitude += fixed<std::int32_t, 2>(raw, index) / geom_scale;
    latitude += fixed<std::int32_t, 2>(raw, index) / geom_scale;
    attribute1 = raw[index++];
    auto attribute4 = raw[index++];

    lrps.emplace_back(longitude, latitude, attribute1, attribute4);

    // Offsets
    std::bitset<8> flags(attribute4);
    poff = (index < size) && flags[6] ? raw[index++] : 0;
    noff = (index < size) && flags[5] ? raw[index++] : 0;
  }

  LineLocation(const std::vector<LocationReferencePoint>& lrps,
               uint8_t positive_offset_bucket,
               uint8_t negative_offset_bucket)
      : lrps(lrps), poff(positive_offset_bucket), noff(negative_offset_bucket) {
    if (lrps.size() < 2) {
      throw std::invalid_argument(
          "Only descriptors with at least 2 LRPs are supported by this implementation");
    }

    if (lrps.size() == 2 && 255 - poff < noff) {
      throw std::invalid_argument(
          "Positive offset cannot be greater than the negative offset when there are only two LRPs, as they would overlap");
    }
  }

  PointLL getFirstCoordinate() const {
    return {lrps.front().longitude, lrps.front().latitude};
  }

  PointLL getLastCoordinate() const {
    return {lrps.back().longitude, lrps.back().latitude};
  }

  float getLength() const {
    return std::accumulate(std::next(lrps.begin()), std::prev(lrps.end()), lrps[0].distance,
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
    const auto& first = lrps.front();
    auto longitude = first.longitude;
    auto latitude = first.latitude;
    append3(decimal2integer(longitude));
    append3(decimal2integer(latitude));
    result.push_back(((first.frc & 0x7) << 3) | (first.fow & 0x7));
    result.push_back(((first.lfrcnp & 0x7) << 5) | (bearing2integer(first.bearing) & 0x1f));
    result.push_back(distance2integer(first.distance));

    // Subsequent location reference points are offset lon lat from the first
    for (auto lrp = std::next(lrps.begin()); lrp != std::prev(lrps.end()); ++lrp) {
      // First longitude
      append2(static_cast<std::int32_t>(std::round(geom_scale * (lrp->longitude - longitude))));
      append2(static_cast<std::int32_t>(std::round(geom_scale * (lrp->latitude - latitude))));
      result.push_back(((lrp->frc & 0x7) << 3) | (lrp->fow & 0x7));
      result.push_back(((lrp->lfrcnp & 0x7) << 5) | (bearing2integer(lrp->bearing) & 0x1f));
      result.push_back(distance2integer(lrp->distance));

      longitude = lrp->longitude;
      latitude = lrp->latitude;
    }

    // Last location reference point
    const auto pofff = poff != 0;
    const auto nofff = noff != 0;
    const auto& last = lrps.back();
    append2(static_cast<std::int32_t>(std::round(geom_scale * (last.longitude - longitude))));
    append2(static_cast<std::int32_t>(std::round(geom_scale * (last.latitude - latitude))));
    result.push_back(((last.frc & 0x7) << 3) | (last.fow & 0x7));
    result.push_back((pofff << 6) | (nofff << 5) | (bearing2integer(last.bearing) & 0x1f));

    // Offsets
    if (pofff) {
      result.push_back(poff);
    }

    if (nofff) {
      result.push_back(noff);
    }

    return result;
  }

  bool operator==(const LineLocation& lloc) const {
    return lrps.size() == lloc.lrps.size() && poff == lloc.poff && noff == lloc.noff &&
           std::equal(lrps.begin(), lrps.end(), lloc.lrps.begin());
  }

  std::vector<LocationReferencePoint> lrps;
  uint8_t poff; // 5.2.9.1 Positive offset - stored as 1/256ths of the path length
  uint8_t noff; // 5.2.9.2 Negative offset - stored as 1/256ths of the path length
};

} // namespace OpenLR
} // namespace midgard
} // namespace valhalla

#endif
