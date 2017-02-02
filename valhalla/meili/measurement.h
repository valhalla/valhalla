/* -*- mode: c++ -*- */

#ifndef MMP_MEASUREMENT_H_
#define MMP_MEASUREMENT_H_

#include <valhalla/midgard/pointll.h>


namespace valhalla {

namespace meili {

class Measurement
{
 public:
  Measurement(const midgard::PointLL& lnglat,
              float gps_accuracy,
              float search_radius):
      lnglat_(lnglat),
      gps_accuracy_(gps_accuracy),
      search_radius_(search_radius)
  {
    if (gps_accuracy_ < 0.f) {
      throw std::invalid_argument("expect non-negative gps_accuracy");
    }
    if(search_radius_ < 0.f) {
      throw std::invalid_argument("expect non-negative search_radius");
    }
  }

  const midgard::PointLL& lnglat() const
  { return lnglat_; }

  float search_radius() const
  { return search_radius_; }

  float sq_search_radius() const
  { return search_radius_ * search_radius_; }

  float gps_accuracy() const
  { return gps_accuracy_; }

 private:

  midgard::PointLL lnglat_;

  float gps_accuracy_;

  float search_radius_;
};

}

}

#endif // MMP_MEASUREMENT_H_
