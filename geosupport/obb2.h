#ifndef __obb2_h_
#define __obb2_h_

#include <stdarg.h>
#include <math.h>

/**
 * Oriented bounding box (2-D).
 * @author David W. Nesbitt
 */
class OBB2 {
 public:
  /**
   * Constructor
   */
  OBB2() { }

  void Set(const Point2& a0, const Point2& a1,
           const Point2& a2, const Point2& a3) {
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

  /**
   * Check if two oriented bounding boxes overlap
   */
  bool Overlap(const OBB2& b) const {
    // Translation of B into A's frame
    Vector2 v(center_, b.center_);
    Vector2 t(v.Dot(basis0_), v.Dot(basis1_));

    // B's basis with respect to A's local frame (rotation matrix)
    // Calculate the 10 and 11 components later
    float r00 = basis0_.Dot(b.basis0_);
    float r01 = basis0_.Dot(b.basis1_);

    // A's basis vectors as separating axes
    float rb = b.extent0_ * fabs(r00) + b.extent1_ * fabs(r01);
    if (fabs(t.x()) > (extent0_ + rb))
      return false;

    float r10 = basis1_.Dot(b.basis0_);
    float r11 = basis1_.Dot(b.basis1_);
    rb = b.extent0_ * fabs(r10) +  b.extent1_ * fabs(r11);
    if (fabs(t.y()) > (extent1_ + rb))
      return false;

    // B's basis vectors as separating axes
    float ra = extent0_ * fabs(r00) + extent1_ * fabs(r10);
    if (fabs(t.x() * r00 + t.y() * r10) > (ra + b.extent0_))
      return false;

    ra = extent0_ * fabs(r01) + extent1_ * fabs(r11);
    if (fabs(t.x() * r01 + t.y() * r11) > (ra + b.extent1_))
      return false;

    // No separating axis found, the two boxes overlap
    return true;
  }

 private:
   Point2  center_;   // Center of the oriented bounding box
   float   extent0_;  // Half length along the basis vector 0
   float   extent1_;  // Half length along the basis vector 1
   Vector2 basis0_;   // Basis vector defined by 1st edge
   Vector2 basis1_;   // Basis vector defined by 2nd edge
};

#endif
