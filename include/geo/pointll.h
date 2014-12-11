#ifndef VALHALLA_GEO_POINTLL_H_
#define VALHALLA_GEO_POINTLL_H_

#include "point2.h"
#include "util.h"

namespace valhalla{
namespace geo{

/**
 * Latitude, Longitude point. Derives from Point2 and allows access methods
 * using lat,lng naming. Extends functionality to add heading, distance based
 * on spherical geometry.
 * @author  David W. Nesbitt
 */
class PointLL : public Point2 {
 public:
  /**
   * Default constructor.  Sets latitude and longitude to 0.
   */
  PointLL();

  /**
   * Constructor using a latitude,longitude pair to initialize.
   * @param   lat    Latitude in degrees
   * @param   lng    Longitude in degrees
   */
  PointLL(const float lat, const float lng);

  /**
   * Copy constructor.
   * @param   ll    Latitude, longitude pair to copy.
   */
  PointLL(const PointLL& ll);

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
   * Set the coordinate components to the specified values.
   * @param   x   x coordinate position.
   * @param   y   y coordinate position.
   */
  void Set(const float x, const float y);

  /**
   * Calculates the distance between two lat/lng's in km. Uses spherical
   * geometry (law of cosines).
   * @param   ll2   Second lat,lng position to calculate distance to.
   * @return  Returns the distance in km.
   */
  float Distance(const PointLL& ll2) const;

  /**
   * Calculates the distance squared between two lat/lng's in km.
   * Uses spherical geometry. No benefit when using squared distances
   * over a spherical earth. May want to use DistanceApproximator for
   * squared distance approximations.
   * @param   ll2   Second lat,lng position to calculate distance to.
   * @return  Returns the distance squared in km.
   */
  float DistanceSquared(const PointLL& ll2) const;


  /**
   * Compute the length of the polyline represented by a set of
   * lat,lng points. Avoids having to copy the points into the
   * polyline.
   * @param  pts  List of lat,lng points.
   * @return  Returns the length in kilometers
   */
  float Length(const std::vector<PointLL>& pts) const;

  /**
   * Calculates the curvature using this position and 2 others. Found by
   * computing the radius of the circle that circumscribes the 3 positions.
   * @param   ll1   Second lat,lng position
   * @param   ll2   Third lat,lng position
   * @return  Returns the curvature in km.
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
   * squared to that point.
   * @param  pts     List of points on the polyline.
   * @param  closest (OUT) Closest point along the polyline
   * @param  idx     (OUT) Index of the segment of the polyline which contains
   *                       the closest point.
   * @return   Returns the distance squared of the closest point.
   */
  inline float ClosestPoint(const std::vector<PointLL>& pts, PointLL& closest,
            int& idx) const;
};

}
}

#endif  // VALHALLA_GEO_POINTLL_H_
