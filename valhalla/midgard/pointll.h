#ifndef VALHALLA_MIDGARD_POINTLL_H_
#define VALHALLA_MIDGARD_POINTLL_H_

#include <valhalla/midgard/point2.h>
#include <tuple>

namespace valhalla {
namespace midgard {

/**
 * Latitude, Longitude point. Derives from Point2 and allows access methods
 * using lat,lng naming. Extends functionality to add heading, distance based
 * on spherical geometry.
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
   * Checks for validity of the coordinates
   * @return  Returns the false if lat or lon coordinates are outside of
   *          the valid range
   */
  bool IsValid() const;

  /**
   * Sets the coordinates to an invalid state
   */
  void Invalidate();

  /**
   * Calculates the distance between two lat/lng's in meters. Uses spherical
   * geometry (law of cosines).
   * @param   ll2   Second lat,lng position to calculate distance to.
   * @return  Returns the distance in meters.
   */
  float Distance(const PointLL& ll2) const;

  /**
   * Calculates the distance squared between two lat/lng's in meters.
   * Uses spherical geometry. No benefit when using squared distances
   * over a spherical earth. May want to use DistanceApproximator for
   * squared distance approximations.
   * @param   ll2   Second lat,lng position to calculate distance to.
   * @return  Returns the distance squared in meters.
   */
  float DistanceSquared(const PointLL& ll2) const;

  /**
   * Compute the length of the polyline represented by a set of
   * lat,lng points. Avoids having to copy the points into the
   * polyline.
   * @param  pts  List of lat,lng points.
   * @return  Returns the length in meters
   */
  float Length(const std::vector<PointLL>& pts) const;

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
   * squared to that point and the index of the segment where the closest point lies.
   * @param  pts     List of points on the polyline.
   * @return  tuple of <Closest point along the polyline,
   *                    Returns the distance squared (meters) of the closest point,
   *                    Index of the segment of the polyline which contains the closest point
   *                   >
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
};

}
}

#endif  // VALHALLA_MIDGARD_POINTLL_H_
