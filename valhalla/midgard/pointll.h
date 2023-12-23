#pragma once

#include <cmath>
#include <limits>
#include <tuple>
#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/distanceapproximator.h>
#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/point2.h>

namespace valhalla {
namespace midgard {

namespace {
constexpr double RAD_PER_METER = 1.0 / 6378160.187;
} // namespace
constexpr double INVALID_LL = (double)0xBADBADBAD;
/**
 * Longitude, Latitude  point. Derives from Point2 and allows access methods
 * using lng,lat naming. Extends functionality to add heading, curvature,
 * and distance based on spherical geometry. Note that the order in the pair
 * is LONGITUDE first, LATITUDE second.
 */
template <typename PrecisionT> class GeoPoint : public PointXY<PrecisionT> {

public:
  /**
   * Use the constructors provided by pair
   */
  using PointXY<PrecisionT>::PointXY;
  using PointXY<PrecisionT>::first;
  using PointXY<PrecisionT>::second;
  using typename PointXY<PrecisionT>::value_type;

  /**
   * Default constructor.  Sets longitude and latitude to INVALID_LL.
   */
  GeoPoint() : PointXY<PrecisionT>(INVALID_LL, INVALID_LL) {
  }
  /**
   * You can encode lon, lat (with 7 digit precision) into 64 bits with the lowest 31 bits for lat and
   * upper 32 for lon and 1 extra unused bit for whatever you need.. this constructor throws if the
   * values are out of valid lon and lat ranges
   * @param encoded
   */
  explicit GeoPoint(uint64_t encoded)
      : PointXY<PrecisionT>(static_cast<PrecisionT>(
                                (int64_t((encoded >> 31) & ((1ull << 32) - 1)) - 180 * 1e7) * 1e-7),
                            static_cast<PrecisionT>(
                                (int64_t(encoded & ((1ull << 31) - 1)) - 90 * 1e7) * 1e-7)) {
  }
  /**
   * Parent constructor. Forwards to parent.
   */
  GeoPoint(const PointXY<PrecisionT>& p) : PointXY<PrecisionT>(p) {
  }

  /**
   * cast the lon lat to a 64bit value with 7 decimals places of precision the layout is:
   * bitfield {
   *  lat  31;
   *  lon  32;
   *  spare 1;
   * }
   * @return
   */
  explicit operator uint64_t() const {
    return (((uint64_t(first * 1e7) + uint64_t(180 * 1e7)) & ((1ull << 32) - 1)) << 31) |
           ((uint64_t(second * 1e7) + uint64_t(90 * 1e7)) & ((1ull << 31) - 1));
  }

  /**
   * Get the longitude in degrees.
   * @return  Returns longitude in degrees.
   */
  PrecisionT lng() const {
    return first;
  }

  /**
   * Get the latitude in degrees.
   * @return  Returns latitude (degrees).
   */
  PrecisionT lat() const {
    return second;
  }

  /**
   * Checks for validity of the coordinates.
   * @return  Returns the false if lat or lon coordinates are set to INVALID_LL.
   */
  bool IsValid() const {
    return first != INVALID_LL && second != INVALID_LL;
  };

  /**
   * Checks whether the lon and lat coordinates fall within -180/180 and -90/90 respectively
   * @return
   */
  bool InRange() const {
    return -180 <= first && first <= 180 && -90 <= second && second <= 90;
  }

  /**
   * Sets the coordinates to an invalid state
   */
  void Invalidate() {
    first = INVALID_LL;
    second = INVALID_LL;
  }

  /**
   * Returns the point along the segment between this point and the provided point using the provided
   * distance along. A distance of .5 would be the point halfway between the two points. A distance of
   * .25 would be 25% of the way from this point to the provided point. The default distance is .5, so
   * the midpoint
   * @param   p1         second point of the line segment
   * @param   distance   the percentage along the segment to place the output point
   * @return  returns the point along the line segment at the specified distance
   */
  GeoPoint PointAlongSegment(const GeoPoint& p, PrecisionT distance = .5) const;

  /**
   * Calculates the distance between two lng,lat's in meters. Uses spherical
   * geometry (law of cosines).
   * Note - this method loses precision when the points are within ~1m of each
   *        other, and cannot meaningfully meausre distances less than that.
   * @param   ll2   Second lng,lat position to calculate distance to.
   * @return  Returns the distance in meters.
   */
  PrecisionT Distance(const GeoPoint& ll2) const;

  /**
   * Approximates the distance squared between two lng,lat points - uses
   * the DistanceApproximator.
   * @param   ll2   Second lng,lat position to calculate distance to.
   * @return  Returns the distance squared in meters.
   */
  PrecisionT DistanceSquared(const GeoPoint& ll2) const {
    DistanceApproximator<GeoPoint<PrecisionT>> approx(*this);
    return approx.DistanceSquared(ll2);
  }

  /**
   * Calculates the curvature using this position and 2 others. Found by
   * computing the radius of the circle that circumscribes the 3 positions.
   * @param   ll1   Second lng,lat position
   * @param   ll2   Third lng,lat position
   * @return  Returns the curvature in meters. Returns max value_type if the points
   *          are collinear.
   */
  PrecisionT Curvature(const GeoPoint& ll1, const GeoPoint& ll2) const;

  /**
   * Calculates the heading or azimuth from the current lng,lat to the
   * specified lng,lat. This uses Haversine method (spherical geometry).
   * @param    ll2   lng,lat position to calculate the heading to.
   * @return   Returns the heading in degrees with range [0,360] where 0 is
   *           due north, 90 is east, 180 is south, and 270 is west.
   */
  PrecisionT Heading(const GeoPoint& ll2) const;

  /**
   * Finds the closest point to the supplied polyline as well as the distance
   * to that point and the (floor) index of the segment where the closest
   * point lies. In the case of a tie where the closest point is a point in the
   * linestring, the most extreme index (closest to the end of the linestring
   * in the direction (forward/reverse) of the search) will win
   * @param  pts                  List of points on the polyline.
   * @param  pivot_index          Index where the processing of closest point should start.
   *                              Default value is 0.
   * @param  forward_dist_cutoff  Minimum linear distance along pts that should be considered
   *                              before giving up.
   * @param  reverse_dist_cutoff  Minimum linear distance along pts that should be considered
   *                              before giving up.
   *
   * @return tuple of <Closest point along the polyline,
   *                   Distance in meters of the closest point,
   *                   Index of the segment of the polyline which contains the closest point >
   */
  std::tuple<GeoPoint, PrecisionT, int>
  ClosestPoint(const std::vector<GeoPoint>& pts,
               int pivot_index = 0,
               PrecisionT forward_dist_cutoff = std::numeric_limits<PrecisionT>::infinity(),
               PrecisionT reverse_dist_cutoff = 0) const;

  /**
   * Calculate the heading from the start index within a polyline of lng,lat
   * points to a point at the specified distance from the start.
   * @param  pts   Polyline - list of lng,lat points.
   * @param  dist  Distance in meters from start to find heading to.
   * @param  idx0  Start index within the polyline.
   * @param  idx1  End index within the polyline
   */
  static PrecisionT HeadingAlongPolyline(const std::vector<GeoPoint>& pts,
                                         const PrecisionT dist,
                                         const uint32_t idx0,
                                         const uint32_t idx1);

  /**
   * Calculate the heading from the start of a polyline of lng,lat points to a
   * point at the specified distance from the start.
   * @param  pts   Polyline - list of lng,lat points.
   * @param  dist  Distance in meters from start to find heading to.
   */
  static PrecisionT HeadingAlongPolyline(const std::vector<GeoPoint>& pts, const PrecisionT dist) {
    return HeadingAlongPolyline(pts, dist, 0, pts.size() - 1);
  }

  /**
   * Calculate the heading from a point at a specified distance from the end
   * of a polyline of lng,lat points to the end point of the polyline.
   * @param  pts   Polyline - list of lng,lat points.
   * @param  dist  Distance in meters from end. A point that distance is
   *               used to find the heading to the end point.
   * @param  idx0  Start index within the polyline.
   * @param  idx1  End index within the polyline
   */
  static PrecisionT HeadingAtEndOfPolyline(const std::vector<GeoPoint>& pts,
                                           const PrecisionT dist,
                                           const uint32_t idx0,
                                           const uint32_t idx1);

  /**
   * Calculate the heading from a point at a specified distance from the end
   * of a polyline of lng,lat points to the end point of the polyline.
   * @param  pts   Polyline - list of lng,lat points.
   * @param  dist  Distance in meters from end. A point that distance is
   *               used to find the heading to the end point.
   */
  static PrecisionT HeadingAtEndOfPolyline(const std::vector<GeoPoint>& pts, const PrecisionT dist) {
    return HeadingAtEndOfPolyline(pts, dist, 0, pts.size() - 1);
  }

  /**
   * Test whether this point is to the left of a segment from p1 to p2.
   * The determinant of the vectors p1->p2 p1->this
   * @param  p1  First point of the segment.
   * @param  p2  End point of the segment.
   * @return  Returns a positive value if this point is left of the segment.
   */
  value_type IsLeft(const GeoPoint& p1, const GeoPoint& p2) const {
    return (p2.lng() - p1.lng()) * (lat() - p1.lat()) - (lng() - p1.lng()) * (p2.lat() - p1.lat());
  }

  /**
   * Tests whether this point is within a polygon.
   * @param  poly  List of vertices that form a polygon. Assumes
   *               the following:
   *                  Only the first and last vertices may be duplicated.
   * @return  Returns true if the point is within the polygon, false if not.
   */
  template <class container_t> bool WithinPolygon(const container_t& poly) const;

  /**
   * Handy for templated functions that use both Point2 or PointLL to know whether or not
   * the classes coordinate system is spherical or in the plane
   *
   * @return true if the system is spherical false if not
   */
  static bool IsSpherical() {
    return true;
  }

  /**
   * Project this point onto the line from u to v
   * @param u          first point of segment
   * @param v          second point of segment
   * @return p  the projected point of this onto the segment uv
   */
  GeoPoint Project(const GeoPoint& u, const GeoPoint& v) const {
    auto lon_scale = cosf(second * kRadPerDeg);
    return Project(u, v, lon_scale);
  }

  /**
   * Project this point onto the line from u to v
   * @param u          first point of segment
   * @param v          second point of segment
   * @param lon_scale  needed for spherical projections. dont pass this parameter unless
   *                   you cached it and want to avoid trig functions in a tight loop
   * @return p  the projected point of this onto the segment uv
   */
  GeoPoint Project(const GeoPoint& u, const GeoPoint& v, PrecisionT lon_scale) const;

  /**
   * Project this point to the supplied polyline as well as the distance
   * to that point and the (floor) index of the segment where the projected location lies.
   * @param  pts                  List of points on the polyline.
   * @return tuple of <Closest point along the polyline,
   *                   Distance in meters of the closest point,
   *                   Index of the segment of the polyline which contains the closest point >
   */
  std::tuple<GeoPoint, PrecisionT, int> Project(const std::vector<GeoPoint>& pts) const;
};

using PointLL = GeoPoint<double>;
} // namespace midgard
} // namespace valhalla

namespace std {
template <> struct hash<valhalla::midgard::PointLL> {
  size_t operator()(const valhalla::midgard::PointLL& p) const;
};
} // namespace std
