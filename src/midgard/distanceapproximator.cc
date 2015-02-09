#include "valhalla/midgard/distanceapproximator.h"
#include "valhalla/midgard/constants.h"

namespace valhalla {
namespace midgard {

// Constructor
DistanceApproximator::DistanceApproximator(const PointLL& ll)
    : centerlat_(ll.lat()),
      centerlng_(ll.lng()),
      m_per_lng_degree_(MetersPerLngDegree(centerlat_)) { }

// Set the test point (lat,lng) used for future distance methods
void DistanceApproximator::SetTestPoint(const PointLL& ll) {
  centerlat_ = ll.lat();
  centerlng_ = ll.lng();
  m_per_lng_degree_  = MetersPerLngDegree(centerlat_);
}

// Distance squared (meters) from the test point
float DistanceApproximator::DistanceSquared(const PointLL& ll) const {
  float latm = (ll.lat() - centerlat_) * kMetersPerDegreeLat;
  float lngm = (ll.lng() - centerlng_) * m_per_lng_degree_;
  return (latm * latm + lngm * lngm);
}

// Distance squared (meters) between 2 lat,lngs
float DistanceApproximator::DistanceSquared(const PointLL& ll1,
                                            const PointLL& ll2) {
  float latm = (ll1.lat() - ll2.lat()) * kMetersPerDegreeLat;
  float lngm = (ll1.lng() - ll2.lng()) *
                MetersPerLngDegree((ll1.lat() + ll2.lat()) * 0.5f);
  return (latm * latm + lngm * lngm);
}

// Approximate meters per degree of longitude at the specified latitude
float DistanceApproximator::MetersPerLngDegree(const float lat) {
  return cosf(lat * kRadPerDeg) * kMetersPerDegreeLat;
}

}
}
