#ifndef __pointll_h__
#define __pointll_h__

namespace valhalla{
namespace geo{

/**
 * Latitude, Longitude point. Derives from Point2 and allows access methods
 * using lat,lng naming. Extends functionality to add bearing, distance based
 * on Haversine method.
 * @author  David W. Nesbitt
 */
class PointLL : public Point2 {
 public:
  /**
   * Default constructor.  Sets latitude and longitude to 0.
   */
  PointLL() {
    y_ = 0.0f;
    x_ = 0.0f;
  }

  /**
   * Constructor using a latitude,longitude pair to initialize.
   * @param   lt    Latitude in degrees
   * @param   ln    Longitude in degrees
   */
  PointLL(const float lat, const float lng) {
    y_ = lat;
    x_ = lng;
  }

  /**
   * Copy constructor.
   * @param   ll    Latitude, longitude pair to copy.
   */
  PointLL(const PointLL& ll) {
    y_ = ll.lat();
    x_ = ll.lng();
  }

  /**
   * Get the latitude in degrees.
   * @return  Returns latitude (degrees).
   */
  float lat() const {
    return y_;
  }

  /**
   * Get the longitude in degrees.
   * @return  Returns longitude in degrees.
   */
  float lng() const {
    return x_;
  }

  /**
   * Calculates the distance between two lat/lng's in km. Uses spherical
   * geometry.
   * @param   ll2   Second lat,lng position to calculate distance to.
   * @return  Returns the distance in km.
   */
  float Distance(const PointLL& ll2) const {
    // If points are the same, return 0
    if (*this == ll2)
      return 0.0f;

    // Get the longitude difference.  Don't need to worry about
    // crossing 180 since cos(x) = cos(-x)
    float deltalng = ll2.lng() - lng();
    float a = degrees_to_radians(lat());
    float c = degrees_to_radians(ll2.lat());
    float cosb = (sinf(a) * sinf(c)) +
                 (cosf(a) * cosf(c) * cosf(degrees_to_radians(deltalng)));

    // Find the angle subtended in radians. Protect against cosb being outside
    // -1 to 1 range.
    if (cosb >= 1.0f)
      return 0.00001f;
    else if (cosb < -1.0f)
      return kPi * kRadEarthKm;
    else
      return acosf(cosb) * kRadEarthKm;
  }

  /**
   * Calculates the curvature using this position and 2 others. Found by
   * computing the radius of the circle that circumscribes the 3 positions.
   * @param   ll1   Second lat,lng position
   * @param   ll2   Third lat,lng position
   * @return  Returns the curvature in km.
   */
  float Curvature(const PointLL& ll1, const PointLL& ll2) const {
    // Find the 3 distances between positions
    float a = Distance(ll1);
    float b = ll1.Distance(ll2);
    float c = Distance(ll2);
    float s = (a + b + c) * 0.5f;
    float k = sqrtf(s * (s - a) * (s - b) * (s - c));
    return ((a * b * c) / (4.0f * k));
  }

  /**
   * Calculates the azimuth or bearing from the current lat,lng to the
   * specified lat,lng. This uses Haversine method (spherical geometry).
   * @param    ll2   Lat,lng position to calculate the bearing to.
   * @return   Returns the bearing in degrees with range [0,360] where 0 is
   *           due north, 90 is east, 180 is south, and 270 is west.
   */
  float Bearing(const PointLL& ll2) const {
    // If points are the same, return 0
    if (*this == ll2)
      return 0.0f;

    // Convert to radians and compute bearing
    float lat1 = degrees_to_radians(lat());
    float lat2 = degrees_to_radians(ll2.lat());
    float dlng = degrees_to_radians(ll2.lng() - lng());
    float y = sinf(dlng) * cosf(lat2);
    float x = cosf(lat1) * sinf(lat2) -
              sinf(lat1) * cosf(lat2) * cosf(dlng);
    float bearing = radians_to_degrees(atan2f(y, x));
    return (bearing < 0.0f) ? bearing + 360.0f : bearing;
  }

  // Method defined in geo.h

  /**
   * Finds the closest point to the supplied polyline as well as the distance
   * squared to that point.
   * @param  pts     List of points on the polyline.
   * @param  closest (OUT) Closest point along the polyline
   * @param  idx     (OUT) Index of the segment of the polyline which contains
   *                       the closest point.
   * @return   Returns the distance squared of the closest point.
   */
  float ClosestPoint(const std::vector<PointLL>& pts, PointLL& closest,
            int& idx) const;
};

}
}

#endif
