#include "valhalla/midgard/obb2.h"

namespace valhalla {
namespace midgard {

// Default constructor
template <class coord_t> OBB2<coord_t>::OBB2() : extent0_(0), extent1_(0) {
}

// Construct an oriented bounding box given 4 corners. The center is found by
// the average of the 4 vertex positions and the axes of the OBB are formed
// by a vector from a0 to a1 and the other by a vector from a1 to a2.
template <class coord_t>
OBB2<coord_t>::OBB2(const coord_t& a0, const coord_t& a1, const coord_t& a2, const coord_t& a3) {
  Set(a0, a1, a2, a3);
}

// Set an oriented bounding box given 4 corners. The center is found by
// the average of the 4 vertex positions and the axes of the OBB are formed
// by a vector from a0 to a1 and the other by a vector from a1 to a2.
template <class coord_t>
void OBB2<coord_t>::Set(const coord_t& a0, const coord_t& a1, const coord_t& a2, const coord_t& a3) {
  // Find center positions of each bounding box
  center_.Set((a0.x() + a1.x() + a2.x() + a3.x()) * 0.25f,
              (a0.y() + a1.y() + a2.y() + a3.y()) * 0.25f);

  // Find the 2 extents and set up the 2 basis vectors
  Vector2 v1(a0, a1);
  float norm = v1.Norm();
  extent0_ = norm * 0.5f;
  basis0_.Set(v1.x() / norm, v1.y() / norm);

  Vector2 v2(a1, a2);
  norm = v2.Norm();
  extent1_ = norm * 0.5f;
  basis1_.Set(v2.x() / norm, v2.y() / norm);
}

// Check if two oriented bounding boxes overlap. Uses the separating
// axis theorem.
template <class coord_t> bool OBB2<coord_t>::Overlap(const OBB2<coord_t>& b) const {
  // Translation of B into A's frame
  Vector2 v(center_, b.center_);
  Vector2 t(v.Dot(basis0_), v.Dot(basis1_));

  // B's basis with respect to A's local frame (rotation matrix)
  // Calculate the 10 and 11 components later
  float r00 = basis0_.Dot(b.basis0_);
  float r01 = basis0_.Dot(b.basis1_);

  // A's basis vectors as separating axes
  float rb = b.extent0_ * std::fabs(r00) + b.extent1_ * std::fabs(r01);
  if (std::fabs(t.x()) > (extent0_ + rb)) {
    return false;
  }

  float r10 = basis1_.Dot(b.basis0_);
  float r11 = basis1_.Dot(b.basis1_);
  rb = b.extent0_ * std::fabs(r10) + b.extent1_ * std::fabs(r11);
  if (std::fabs(t.y()) > (extent1_ + rb)) {
    return false;
  }

  // B's basis vectors as separating axes
  float ra = extent0_ * std::fabs(r00) + extent1_ * std::fabs(r10);
  if (std::fabs(t.x() * r00 + t.y() * r10) > (ra + b.extent0_)) {
    return false;
  }

  ra = extent0_ * std::fabs(r01) + extent1_ * std::fabs(r11);
  if (std::fabs(t.x() * r01 + t.y() * r11) > (ra + b.extent1_)) {
    return false;
  }

  // No separating axis found, the two boxes overlap
  return true;
}

// Explicit instantiation
template class OBB2<Point2>;
template class OBB2<PointLL>;

} // namespace midgard
} // namespace valhalla
