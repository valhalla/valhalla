
#ifndef VALHALLA_MIDGARD_DISTANCEAPPROXIMATOR_H_
#define VALHALLA_MIDGARD_DISTANCEAPPROXIMATOR_H_

#include <cmath>

#include <valhalla/midgard/constants.h>

namespace valhalla {
namespace midgard {

/**
 * Provides distance approximation in latitude, longitude space. Approximates
 * distance in meters between two points. This method is more efficient
 * than using spherical distance calculations. It computes an approximate
 * distance using the Pythagorean theorem with the meters of latitude change
 * (exact) and the meters of longitude change at the "test point". Longitude
 * is inexact since meters per degree of longitude changes with latitude.
 * This approximation has very little error (less than 1%) if the positions
 * are close to one another (within several hundred meters). Error
 * increases at high (near polar) latitudes. This method will not work if the
 * points cross 180 degrees longitude.
 *
 * The main use of this method is to perform distance comparisons relative to
 * a single location (e.g. in the A* heuristic with path finding algorithms).
 * This method uses an approximation of meters per degree of longitude which
 * varies with latitude. The distance from A to B may not match the distance
 * from B to A if the latitudes of the 2 points differ.
 */
template <typename PointT> class DistanceApproximator {
public:
  /**
   * Constructor.
   *
   * Sets the test point.  This method is used when a distance is to be
   * checked for a series of positions relative to a single point.  This
   * precalculates the meters per degree of longitude.
   * @param   ll    Latitude, longitude of the test point (degrees)
   */
  DistanceApproximator(const PointT& ll)
      : centerlat_(ll.lat()), centerlng_(ll.lng()), m_lng_scale_(LngScalePerLat(centerlat_)),
        m_per_lng_degree_(m_lng_scale_ * kMetersPerDegreeLat) {
  }

  /**
   * Sets the test point.  This method is used when a distance is to be
   * checked for a series of positions relative to a single point.  This
   * precalculates the meters per degree of longitude.
   * @param   ll    Latitude, longitude of the test point (degrees)
   */
  void SetTestPoint(const PointT& ll) {
    centerlat_ = ll.lat();
    centerlng_ = ll.lng();
    m_lng_scale_ = LngScalePerLat(centerlat_);
    m_per_lng_degree_ = m_lng_scale_ * kMetersPerDegreeLat;
  }

  /*
   * Getter for lng scale
   * @return the distance scale for lng at this points latitude
   */
  typename PointT::first_type GetLngScale() const {
    return m_lng_scale_;
  }

  /**
   * Approximates the arc distance between the supplied position and the
   * current test point.  It uses the pythagorean theorem with meters
   * per latitude and longitude degree. Assumes the number of meters per
   * degree of longitude at the test point latitude.
   * @param   ll    Latitude, longitude of the point (degrees)
   * @return  Returns the squared distance in meters by using pythagorean
   *          theorem.  Squared distance is returned for more efficient
   *          searching (avoids sqrt).
   */
  typename PointT::first_type DistanceSquared(const PointT& ll) const {
    return sqr((ll.lat() - centerlat_) * kMetersPerDegreeLat) +
           sqr((ll.lng() - centerlng_) * m_per_lng_degree_);
  }

  /**
   * Approximates arc distance between 2 lat,lng positions using meters per
   * latitude and longitude degree.  Uses the mid latitude of the 2 positions
   * to estimate the number of meters per degree of longitude.
   * @param   ll1  First point (lat,lng)
   * @param   ll2  Second point (lat,lng)
   * @return  Returns the approximate distance squared (in meters)
   */
  static typename PointT::first_type DistanceSquared(const PointT& ll1, const PointT& ll2) {
    auto latm = (ll1.lat() - ll2.lat()) * kMetersPerDegreeLat;
    auto lngm = (ll1.lng() - ll2.lng()) * MetersPerLngDegree((ll1.lat() + ll2.lat()) * 0.5);
    return (latm * latm + lngm * lngm);
  }

  /**
   * Gets the number of meters per degree of longitude for a specified
   * latitude.  While the number of meters per degree of latitude is
   * constant, the number of meters per degree of longitude varies: it has
   * a maximum at the equator and lessens as latitude approaches the poles.
   * @param   lat   Latitude in degrees
   * @return  Returns the number of meters per degree of longitude
   */
  static typename PointT::first_type MetersPerLngDegree(const typename PointT::first_type lat) {
    return LngScalePerLat(lat) * kMetersPerDegreeLat;
  }

  /**
   * Gets the distance scale needed when computing units of longitude
   * at a certain latitude.
   * @param   lat   Latitude in degrees
   * @return  Returns the scale to use for longitude at this degree of latitude
   */
  static typename PointT::first_type LngScalePerLat(const typename PointT::first_type lat) {
    return cosf(lat * kRadPerDeg);
  }

private:
  typename PointT::first_type centerlat_;
  typename PointT::first_type centerlng_;
  typename PointT::first_type m_lng_scale_;
  typename PointT::first_type m_per_lng_degree_;

  typename PointT::first_type sqr(const typename PointT::first_type v) const {
    return v * v;
  }
};

} // namespace midgard
} // namespace valhalla

#endif // VALHALLA_MIDGARD_DISTANCEAPPROXIMATOR_H_
