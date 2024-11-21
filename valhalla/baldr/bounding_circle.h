#ifndef VALHALLA_BALDR_BOUNDING_CIRCLE_H_
#define VALHALLA_BALDR_BOUNDING_CIRCLE_H_

#include <valhalla/midgard/pointll.h>

namespace valhalla {
namespace baldr {

class BoundingCircle {
public:
  // TODO - compress to 64 bits
  float lng_;
  float lat_;
  float radius_;

  BoundingCircle(const midgard::PointLL& ll, const float radius) {
    lat_ = ll.lat();
    lng_ = ll.lng();
    radius_ = radius;
  }

  midgard::PointLL center() const {
    return midgard::PointLL(lng_, lat_);
  }

  float radius() const {
    return radius_;
  }

private:
  // Force use of constructor with args
  BoundingCircle() {
  }
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_BOUNDING_CIRCLE_H_
