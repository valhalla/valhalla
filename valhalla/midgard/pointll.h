#ifndef VALHALLA_MIDGARD_POINTLL_H_
#define VALHALLA_MIDGARD_POINTLL_H_

#include <valhalla/midgard/point2.h>
#include <tuple>

namespace valhalla {
namespace midgard {

/**
 * Latitude, Longitude point. Derives from Point2 and allows access methods
 * using lat,lng naming. Extends functionality to add heading, curvature,
 * and distance based on spherical geometry.
 */
class PointLL : public Point2 {
 public:
  /**
   * Use the constructors provided by pair
   */
  using Point2::Point2;

  /**
   * Default constructor.  Sets latitude and longitude to INVALID.
   */
  PointLL();

  /**
   * Get the latitude in degrees.
   * @return  Returns latitude (degrees).
   */
  float lat() const;

  /**
   * Get the longitude in degrees.
   * @return  Returns longitude in degrees.
   */
  float lng() const;

  /**
   * Checks for validity of the coordinates.
   * @return  Returns the false if lat or lon coordinates are set to INVALID.
   */
  bool IsValid() const;

  /**
   * Sets the coordinates to an invalid state
   */
  void Invalidate();

  /**
   * Gets the midpoint on a line segment between this point and point p1.
   * @param   p1  Point
   * @return  Returns the midpoint between this point and p1.
   */
  PointLL MidPoint(const PointLL& p1) const;

  /**
   * Calculates the distance between two lat/lng's in meters. Uses spherical
   * geometry (law of cosines).
   * @param   ll2   Second lat,lng position to calculate distance to.
   * @return  Returns the distance in meters.
   */
  float Distance(const PointLL& ll2) const;

  /**
   * Approximates the distance squared between two lat,lng points - uses
   * the DistanceApproximator.
   * @param   ll2   Second lat,lng position to calculate distance to.
   * @return  Returns the distance squared in meters.
   */
  float DistanceSquared(const PointLL& ll2) const;

  /**
   * Calculates the curvature using this position and 2 others. Found by
   * computing the radius of the circle that circumscribes the 3 positions.
   * @param   ll1   Second lat,lng position
   * @param   ll2   Third lat,lng position
   * @return  Returns the curvature in meters.
   */
  float Curvature(const PointLL& ll1, const PointLL& ll2) const;

  /**
   * Calculates the heading or azimuth from the current lat,lng to the
   * specified lat,lng. This uses Haversine method (spherical geometry).
   * @param    ll2   Lat,lng position to calculate the heading to.
   * @return   Returns the heading in degrees with range [0,360] where 0 is
   *           due north, 90 is east, 180 is south, and 270 is west.
   */
  float Heading(const PointLL& ll2) const;

  /**
   * Finds the closest point to the supplied polyline as well as the distance
   * squared to that point and the index of the segment where the closest
   * point lies.
   * @param  pts     List of points on the polyline.
   * @return tuple of <Closest point along the polyline,
   *                   distance squared (meters) of the closest point,
   *                   Index of the segment of the polyline which contains
   *                      the closest point >
   */
  std::tuple<PointLL, float, int> ClosestPoint(const std::vector<PointLL>& pts) const;

  /**
   * Calculate the heading from the start of a polyline of lat,lng points to a
   * point at the specified distance from the start.
   * @param  pts   Polyline - list of lat,lng points.
   * @param  dist  Distance in meters from start to find heading to.
   */
  static float HeadingAlongPolyline(const std::vector<PointLL>& pts,
                                    const float dist);

  /**
   * Calculate the heading from a point at a specified distance from the end
   * of a polyline of lat,lng points to the end point of the polyline.
   * @param  pts   Polyline - list of lat,lng points.
   * @param  dist  Distance in meters from end. A point that distance is
   *               used to find the heading to the end point.
   */
  static float HeadingAtEndOfPolyline(const std::vector<PointLL>& pts,
                                      const float dist);

  /**
   * Test whether this point is to the left of a segment from p1 to p2.
   * @param  p1  First point of the segment.
   * @param  p2  End point of the segment.
   * @return  Returns true if this point is left of the segment.
   */
  virtual float IsLeft(const PointLL& p1, const PointLL& p2) const;

  /**
   * Tests whether this point is within a polygon.
   * @param  poly  List of vertices that form a polygon. Assumes
   *               the following:
   *                  Only the first and last vertices may be duplicated.
   * @return  Returns true if the point is within the polygon, false if not.
   */
  template <class container_t>
  bool WithinPolygon(const container_t& poly) const;

  /**
   * Handy for templated functions that use both Point2 or PointLL to know whether or not
   * the classes coordinate system is spherical or in the plane
   *
   * @return true if the system is spherical false if not
   */
  static bool IsSpherical();
};

}
}

namespace std {
  template <> struct hash<valhalla::midgard::PointLL> {
    size_t operator()(const valhalla::midgard::PointLL& p) const {
      uint64_t h;
      char* b = static_cast<char*>(static_cast<void*>(&h));
      std::memcpy(b, &p.first, 4);
      std::memcpy(b + 4, &p.second, 4);
      return std::hash<uint64_t>()(h);
    }
  };
}

#endif  // VALHALLA_MIDGARD_POINTLL_H_
