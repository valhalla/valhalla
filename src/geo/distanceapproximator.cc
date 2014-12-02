#include "geo/distanceapproximator.h"
#include "geo/constants.h"

namespace valhalla{
namespace geo{
  DistanceApproximator::DistanceApproximator():centerlat_(0),centerlng_(0),km_per_lng_degree_(0) { }

  DistanceApproximator::~DistanceApproximator() { }

  void DistanceApproximator::SetTestPoint(const PointLL& ll) {
    centerlat_ = ll.lat();
    centerlng_ = ll.lng();
    km_per_lng_degree_  = KmPerLngDegree(centerlat_);
  }

  float DistanceApproximator::DistanceSquared(const PointLL& ll) const {
    float latkm = (ll.lat() - centerlat_) * kKmPerDegreeLat;
    float lngkm = (ll.lng() - centerlng_) * km_per_lng_degree_;
    return (latkm * latkm + lngkm * lngkm);
  }

  float DistanceApproximator::DistanceSquared(const PointLL& ll1, const PointLL& ll2) const {
    float latkm = (ll1.lat() - ll2.lat()) * kKmPerDegreeLat;
    float lngkm = (ll1.lng() - ll2.lng()) *
                  KmPerLngDegree((ll1.lat() + ll2.lat()) * 0.5);
    return (latkm * latkm + lngkm * lngkm);
  }

  float DistanceApproximator::KmPerLngDegree(const float lat) const {
    return cosf(degrees_to_radians(lat)) * kKmPerDegreeLat;
  }
}
}
