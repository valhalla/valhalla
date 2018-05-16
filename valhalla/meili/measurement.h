/* -*- mode: c++ -*- */

#ifndef MMP_MEASUREMENT_H_
#define MMP_MEASUREMENT_H_

#include <cinttypes>
#include <valhalla/midgard/pointll.h>

namespace valhalla {

namespace meili {

class Measurement {
public:
  Measurement(const midgard::PointLL& lnglat,
              float gps_accuracy,
              float search_radius,
              double epoch_time = -1.f)
      : lnglat_(lnglat), gps_accuracy_(gps_accuracy), search_radius_(search_radius),
        epoch_time_(epoch_time) {
    if (gps_accuracy_ < 0.f) {
      throw std::invalid_argument("non-negative gps_accuracy required");
    }
    if (search_radius_ < 0.f) {
      throw std::invalid_argument("non-negative search_radius required");
    }
  }

  const midgard::PointLL& lnglat() const {
    return lnglat_;
  }

  float search_radius() const {
    return search_radius_;
  }

  float sq_search_radius() const {
    return search_radius_ * search_radius_;
  }

  float sq_gps_accuracy() const {
    return gps_accuracy_ * gps_accuracy_;
  }

  float gps_accuracy() const {
    return gps_accuracy_;
  }

  double epoch_time() const {
    return epoch_time_;
  }

private:
  midgard::PointLL lnglat_;

  float gps_accuracy_;

  float search_radius_;

  double epoch_time_;
};

} // namespace meili

} // namespace valhalla

#endif // MMP_MEASUREMENT_H_
