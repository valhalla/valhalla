#ifndef VALHALLA_MIDGARD_OPENLR_H_
#define VALHALLA_MIDGARD_OPENLR_H_

#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/util.h>
#include <valhalla/proto/trip.pb.h>

#include <assert.h>
#include <bitset>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace valhalla {
namespace baldr {
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
bearing2integer(const double value) { // p. 48 Equation 6: Calculation of the bearing value (Binary)
  return static_cast<std::int32_t>(value / 11.25);
}

inline double integer2bearing(
    const std::int32_t value) { // p. 48 Equation 6: Calculation of the bearing value (Binary)
  return value * 11.25 + 11.25 / 2.;
}

inline std::int32_t
distance2integer(const double value) { // p. 48 Equation 7: Calculation of the DNP value (Binary)
  return static_cast<std::int32_t>(value / 58.6);
}

inline double integer2distance(
    const std::uint32_t value) { // p. 48 Equation 7: Calculation of the DNP value (Binary)
  return value * 58.6;
}

} // namespace

// Reference: OpenLR™ White Paper Version: 1.5 revision 2
// https://www.openlr-association.com/fileadmin/user_upload/openlr-whitepaper_v1.5.pdf

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
   * constructor and then feed them to the special constructor for the OpenLr
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
                         double bearing,
                         unsigned char frc,
                         FormOfWay fow,
                         LocationReferencePoint* prev,
                         double distance = 0.,
                         unsigned char lfrcnp = 0)
      : longitude(!prev ? integer2decimal(decimal2integer(longitude))
                        : prev->longitude +
                              (std::round((longitude - prev->longitude) * geom_scale) / geom_scale)),
        latitude(!prev ? integer2decimal(decimal2integer(latitude))
                       : prev->latitude +
                             (std::round((latitude - prev->latitude) * geom_scale) / geom_scale)),
        bearing(integer2bearing(bearing2integer(bearing))),
        distance(integer2distance(distance2integer(distance))), frc(frc), lfrcnp(lfrcnp), fow(fow) {
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
  double bearing;       // 5.2.4. Bearing
  double distance;      // 5.2.5. Distance to next LR-point
  unsigned char frc;    // 5.2.2. Functional Road Class: 0 – Main road,  1 – First class road, ...
  unsigned char lfrcnp; // 5.2.6. Lowest FRC to next LR-point
  FormOfWay fow;        // 5.2.3. Form of way
};

// 5.3.1
// The side of road information (SOR) describes the relationship between the
// point of interest and a referenced line. The point can be on the right
// side of the line, on the left side of the line, on both sides of
// the line or directly on the line.
enum SideOfTheRoad {
  DirectlyOnRoadOrNotApplicable = 0,
  RightSideOfRoad = 1,
  LeftSideOfRoad = 2,
  BothSidesOfRoad = 3,
};

// 5.3.2
// The orientation information (ORI) describes the relationship between the point
// of interest and the direction of a referenced line. The point may be directed
// in the same direction as the line, against that direction, both directions,
// or the direction of the point might be unknown.
enum Orientation {
  NoOrientation = 0,
  FirstLrpTowardsSecond = 1,
  SecondLrpTowardsFirst = 2,
  BothDirections = 3,
};

const uint8_t OPENLR_VERSION = 3;

// The first byte of the OpenLr identifier, page 55
struct OpenLrStatus {
  uint8_t version : 3;
  uint8_t has_attributes : 1;
  uint8_t ArF0 : 1;
  uint8_t is_point : 1;
  uint8_t ArF1 : 1;
  uint8_t rfu : 1;

  constexpr OpenLrStatus()
      : version(OPENLR_VERSION), has_attributes(0), ArF0(0), is_point(0), ArF1(0), rfu(0){};

  // Status-byte of a Line segment (page 53)
  constexpr static OpenLrStatus Line() {
    OpenLrStatus status;
    status.version = OPENLR_VERSION;
    status.has_attributes = 1;
    status.ArF0 = 0;
    status.is_point = 0;
    status.ArF1 = 0;
    status.rfu = 0;
    return status;
  }
  // Status-byte of a PointAlongLine segment (page 55)
  constexpr static OpenLrStatus PointAlongLine() {
    OpenLrStatus status;
    status.version = OPENLR_VERSION;
    status.has_attributes = 1;
    status.ArF0 = 0;
    status.is_point = 1;
    status.ArF1 = 0;
    status.rfu = 0;
    return status;
  }
};

// The first metadata byte after the first coordinate
struct Attribute5 {
  uint8_t form_of_way : 3;
  uint8_t functional_road_class : 3;
  uint8_t orientation : 2; // Only defined for PointAlongLine
};
// The first metadata byte after the non-first coordinate
struct Attribute6 {
  uint8_t form_of_way : 3;
  uint8_t functional_road_class : 3;
  uint8_t side_of_the_road : 2; // Only defined for PointAlongLine
};

// Line locations, p.19, section 3.1
// Only line location with 2 location reference points are supported
struct OpenLr {
  OpenLr(const std::string& reference, bool base64_encoded = false) {
    const std::string& decoded = base64_encoded ? midgard::decode64(reference) : reference;
    const size_t size = decoded.size();

    const auto raw = reinterpret_cast<const uint8_t*>(decoded.data());
    std::size_t index = 0;

    // Status, version 3, has attributes, ArF 'Circle or no area location'
    const OpenLrStatus& status = *reinterpret_cast<const OpenLrStatus*>(&raw[index++]);
    if (status.version != 3) {
      throw std::invalid_argument("invalid_version " + std::to_string(status.version) +
                                  ": Can only parse openlr version 3");
    }
    if (!status.ArF1 && !status.ArF0 && status.has_attributes) {
      isPointAlongLine = status.is_point;
    } else {
      throw std::invalid_argument("OpenLR reference invalid status " + decoded);
    }

    if (isPointAlongLine) {
      if (size != 17 && size != 16) {
        throw std::invalid_argument(
            "OpenLR PointAlongLine reference is not the expected 17 bytes: size=" +
            std::to_string(size) + " reference=" + reference);
      }
    } else {
      // LineLocation
      //  Line location data size: 16 + (n-2)*7 + [0/1/2] bytes
      if (size < 16) {
        throw std::invalid_argument("OpenLR Line reference is too small reference=" + reference +
                                    "size=" + std::to_string(decoded.size()));
      }
    }

    // First location reference point
    auto longitude = integer2decimal(fixed<std::int32_t, 3>(raw, index));
    auto latitude = integer2decimal(fixed<std::int32_t, 3>(raw, index));
    auto attribute1 = raw[index++];
    auto attribute2 = raw[index++];
    auto attribute3 = raw[index++];

    lrps.emplace_back(longitude, latitude, attribute1, attribute2, attribute3);

    if (isPointAlongLine) {
      const Attribute5& attr5 = *reinterpret_cast<const Attribute5*>(&attribute1);
      orientation = static_cast<Orientation>(attr5.orientation);
    } else {
      orientation = Orientation::NoOrientation;
    }

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

    if (isPointAlongLine) {
      const Attribute6 attr6 = *reinterpret_cast<const Attribute6*>(&attribute1);
      sideOfTheRoad = static_cast<SideOfTheRoad>(attr6.side_of_the_road);
    } else {
      sideOfTheRoad = SideOfTheRoad::DirectlyOnRoadOrNotApplicable;
    }

    lrps.emplace_back(longitude, latitude, attribute1, attribute4);

    // Offsets
    std::bitset<8> flags(attribute4);
    poff = (index < size) && flags[6] ? raw[index++] : 0;
    noff = (index < size) && flags[5] ? raw[index++] : 0;
  }

  OpenLr(const std::vector<LocationReferencePoint>& lrps,
         uint8_t positive_offset_bucket,
         uint8_t negative_offset_bucket,
         bool point_along_line = false,
         const Orientation& orientation = Orientation::NoOrientation,
         const SideOfTheRoad& side_of_the_road = SideOfTheRoad::DirectlyOnRoadOrNotApplicable)
      : lrps(lrps), poff(positive_offset_bucket), noff(negative_offset_bucket),
        isPointAlongLine(point_along_line), orientation(orientation),
        sideOfTheRoad(side_of_the_road) {
    if (lrps.size() < 2) {
      throw std::invalid_argument(
          "Only descriptors with at least 2 LRPs are supported by this implementation");
    }

    if (lrps.size() == 2 && 255 - poff < noff) {
      throw std::invalid_argument(
          "Positive offset cannot be greater than the negative offset when there are only two LRPs, as they would overlap");
    }
  }

  midgard::PointLL getFirstCoordinate() const {
    return {lrps.front().longitude, lrps.front().latitude};
  }

  midgard::PointLL getLastCoordinate() const {
    return {lrps.back().longitude, lrps.back().latitude};
  }

  double getLength() const {
    return std::accumulate(std::next(lrps.begin()), std::prev(lrps.end()), lrps[0].distance,
                           [](double distance, const LocationReferencePoint& lrp) {
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
    {
      const OpenLrStatus status =
          isPointAlongLine ? OpenLrStatus::PointAlongLine() : OpenLrStatus::Line();
      result.push_back(*reinterpret_cast<const uint8_t*>(&status));
    }

    // First location reference point
    const auto& first = lrps.front();
    auto longitude = first.longitude;
    auto latitude = first.latitude;
    append3(decimal2integer(longitude));
    append3(decimal2integer(latitude));
    {
      uint8_t attr1 = ((first.frc & 0x7) << 3) | (first.fow & 0x7);
      if (isPointAlongLine) {
        attr1 |= (orientation << 6);
      }
      result.push_back(attr1);
    }
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
    {
      const uint8_t attr6 = (sideOfTheRoad << 6) | ((last.frc & 0x7) << 3) | (last.fow & 0x7);
      result.push_back(attr6);
    }
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

  std::string toBase64() const {
    return midgard::encode64(toBinary());
  }

  bool operator==(const OpenLr& lloc) const {
    return lrps.size() == lloc.lrps.size() && poff == lloc.poff && noff == lloc.noff &&
           std::equal(lrps.begin(), lrps.end(), lloc.lrps.begin()) &&
           orientation == lloc.orientation && sideOfTheRoad == lloc.sideOfTheRoad;
  }

  std::vector<LocationReferencePoint> lrps;
  uint8_t poff; // 5.2.9.1 Positive offset - stored as 1/256ths of the path length
  uint8_t noff; // 5.2.9.2 Negative offset - stored as 1/256ths of the path length
  bool isPointAlongLine;
  Orientation orientation;
  SideOfTheRoad sideOfTheRoad;
};

} // namespace OpenLR
} // namespace baldr
} // namespace valhalla

#endif
