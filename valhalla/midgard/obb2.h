#ifndef VALHALLA_MIDGARD_OBB2_H_
#define VALHALLA_MIDGARD_OBB2_H_

#include <math.h>
#include <stdarg.h>

#include <valhalla/midgard/point2.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/vector2.h>

namespace valhalla {
namespace midgard {

/**
 * Oriented bounding box (2-D). Simple collision detection method
 * where an OBB is set by its 4 corners and an overlap/collision check
 * can be performed against another OBB. Template class to work with
 * Point2 (Euclidean x,y) or PointLL (latitude,longitude).
 */
template <class coord_t> class OBB2 {
public:
  /**
   * Constructor
   */
  OBB2();

  /**
   * Construct an oriented bounding box given 4 corners. The center is found by
   * the average of the 4 vertex positions and the axes of the OBB are formed
   * by a vector from a0 to a1 and the other by a vector from a1 to a2.
   * @param  a0  Corner vertex on the bounding box.
   * @param  a1  Corner vertex on the bounding box.
   * @param  a2  Corner vertex on the bounding box.
   * @param  a3  Corner vertex on the bounding box.
   */
  OBB2(const coord_t& a0, const coord_t& a1, const coord_t& a2, const coord_t& a3);

  /**
   * Set an oriented bounding box given 4 corners. The center is found by
   * the average of the 4 vertex positions and the axes of the OBB are formed
   * by a vector from a0 to a1 and the other by a vector from a1 to a2.
   * @param  a0  Corner vertex on the bounding box.
   * @param  a1  Corner vertex on the bounding box.
   * @param  a2  Corner vertex on the bounding box.
   * @param  a3  Corner vertex on the bounding box.
   */
  void Set(const coord_t& a0, const coord_t& a1, const coord_t& a2, const coord_t& a3);

  /**
   * Check if two oriented bounding boxes overlap. Uses the separating
   * axis theorem.
   * @param  b  Other bounding box to check for overlap/collision.
   * @return  Returns true if the OBBs overlap/collide, false if not.
   */
  bool Overlap(const OBB2& b) const;

private:
  coord_t center_; // Center of the oriented bounding box
  float extent0_;  // Half length along the basis vector 0
  float extent1_;  // Half length along the basis vector 1
  Vector2 basis0_; // Basis vector defined by 1st edge
  Vector2 basis1_; // Basis vector defined by 2nd edge
};

} // namespace midgard
} // namespace valhalla

#endif // VALHALLA_MIDGARD_OBB2_H_
