#ifndef __obb2_h_
#define __obb2_h_

#include <stdarg.h>
#include <math.h>

#include "point2.h"
#include "vector2.h"

namespace valhalla{
namespace geo{

/**
 * Oriented bounding box (2-D).
 * @author David W. Nesbitt
 */
class OBB2 {
 public:
  /**
   * Constructor
   */
  OBB2();

  void Set(const Point2& a0, const Point2& a1,
           const Point2& a2, const Point2& a3);

  /**
   * Check if two oriented bounding boxes overlap
   */
  bool Overlap(const OBB2& b) const;

 private:
   Point2  center_;   // Center of the oriented bounding box
   float   extent0_;  // Half length along the basis vector 0
   float   extent1_;  // Half length along the basis vector 1
   Vector2 basis0_;   // Basis vector defined by 1st edge
   Vector2 basis1_;   // Basis vector defined by 2nd edge
};

}
}

#endif
