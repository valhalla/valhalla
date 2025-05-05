#include "midgard/linesegment2.h"
#include "midgard/point2.h"
#include "midgard/pointll.h"
#include "midgard/vector2.h"

namespace valhalla {
namespace midgard {

/**
 * Finds the distance squared of a specified point from the line segment
 * and the closest point on the segment to the specified point.
 * @param   p        Test point.
 * @param   closest  (Return) Closest point on the segment to c.
 * @return  Returns the distance squared from pt to the closest point on
 *          the segment.
 */
template <typename coord_t>
typename coord_t::value_type LineSegment2<coord_t>::DistanceSquared(const coord_t& p,
                                                                    coord_t& closest) const {
  using prec_t = typename coord_t::value_type;
  // Construct vector v (ab) and w (ap)
  VectorXY<prec_t> v(a_, b_);
  VectorXY<prec_t> w(a_, p);

  // Numerator of the component of w onto v. If <= 0 then a
  // is the closest point. By separating into the numerator
  // and denominator of the component we avoid a division unless
  // it is necessary.
  prec_t n = w.Dot(v);
  if (n <= 0.0f) {
    closest = a_;
  } else {
    // Get the denominator of the component.  If the component >= 1
    // (d <= n) then point b is the closest point
    prec_t d = v.Dot(v);
    if (d <= n) {
      closest = b_;
    } else {
      // Closest point is along the segment - the projection of w onto v.
      closest = a_ + v * (n / d);
    }
  }
  return closest.DistanceSquared(p);
}

/**
 * Determines if the current segment intersects the specified segment.
 * If an intersect occurs the intersection is computed.  Note: the
 * case where the lines overlap is not considered.
 * @param   segment      Segment to determine intersection with.
 * @param   intersect    (OUT) Intersection point.
 * @return  Returns true if an intersection exists, false if not.
 */
template <typename coord_t>
bool LineSegment2<coord_t>::Intersect(const LineSegment2<coord_t>& segment,
                                      coord_t& intersect) const {
  using prec_t = typename coord_t::value_type;
  // Construct vectors
  VectorXY<prec_t> b = b_ - a_;
  VectorXY<prec_t> d = segment.b() - segment.a();

  // Set 2D perpendicular vector to d
  VectorXY<prec_t> dp = d.GetPerpendicular();

  // Check if denominator will be 0 (lines are parallel)
  prec_t dtb = dp.Dot(b);
  if (dtb == 0) {
    return false;
  }

  // Solve for the parameter t
  VectorXY<prec_t> c = segment.a() - a_;
  prec_t t = dp.Dot(c) / dtb;
  if (t < 0 || t > 1) {
    return false;
  }

  // Solve for the parameter u
  VectorXY<prec_t> bp = b.GetPerpendicular();
  prec_t u = bp.Dot(c) / dtb;
  if (u < 0 || u > 1) {
    return false;
  }

  // An intersect occurs.  Set the intersect point and return true
  intersect = a_ + b * t;
  return true;
}

/**
 * Determines if the line segment intersects specified convex polygon.
 * Based on Cyrus-Beck clipping method.
 * @param   poly   A counter-clockwise oriented polygon.
 * @return  Returns true if any part of the segment intersects the polygon,
 *          false if no intersection.
 */
template <typename coord_t>
bool LineSegment2<coord_t>::Intersect(const std::vector<coord_t>& poly) const {
  using prec_t = typename coord_t::value_type;
  // Initialize the candidate interval
  prec_t t_out = 1;
  prec_t t_in = 0;

  // Iterate through each edge of the polygon
  VectorXY<prec_t> c = b_ - a_;
  auto pt1 = poly.end() - 1;
  auto pt2 = poly.begin();
  for (; pt2 != poly.end(); pt1 = pt2, pt2++) {
    // Set an outward facing normal (polygon is assumed to be CCW)
    VectorXY<prec_t> n(pt2->y() - pt1->y(), pt1->x() - pt2->x());

    // Dot product of the normal to this polygon edge with the ray
    prec_t n_dot_c = n.Dot(c);
    prec_t num = n.Dot((*pt1 - a_));

    // Check for parallel line
    if (std::abs(n_dot_c) < kEpsilon) {
      // No intersection if segment origin is outside this edge
      if (num < 0) {
        return false;
      } else {
        // Skip this edge
        continue;
      }
    }

    // Get intersection and update candidate interval
    prec_t t = num / n_dot_c;
    if (n_dot_c > 0) {
      // Ray is exiting polygon
      if (t < t_out) {
        t_out = t;
      }
    } else {
      // Ray is entering polygon
      if (t > t_in) {
        t_in = t;
      }
    }

    // Early out
    if (t_in > t_out) {
      return false;
    }
  }
  return true;
}

/**
 * Clips the line segment to a specified convex polygon.
 * Based on Cyrus-Beck clipping method.
 * @param  poly           A counter-clockwise oriented polygon.
 * @param  clip_segment   Returns the clipped segment.
 * @return  Returns true if any part of the segment intersects the polygon,
 *          false if no intersection.
 */
template <typename coord_t>
bool LineSegment2<coord_t>::ClipToPolygon(const std::vector<coord_t>& poly,
                                          LineSegment2<coord_t>& clip_segment) const {
  using prec_t = typename coord_t::value_type;
  // Initialize the candidate interval
  prec_t t_out = 1;
  prec_t t_in = 0;

  // Iterate through each edge of the polygon
  VectorXY<prec_t> c = b_ - a_;
  auto pt1 = poly.end() - 1;
  auto pt2 = poly.begin();
  for (; pt2 != poly.end(); pt1 = pt2, pt2++) {
    // Set an outward facing normal (polygon is assumed to be CCW)
    VectorXY<prec_t> n(pt2->y() - pt1->y(), pt1->x() - pt2->x());

    // Dot product of the normal to this polygon edge with the ray
    prec_t n_dot_c = n.Dot(c);
    prec_t num = n.Dot((*pt1 - a_));

    // Check for parallel line
    if (std::abs(n_dot_c) < kEpsilon) {
      // No intersection if segment origin is outside this edge
      if (num < 0) {
        return false;
      } else {
        // Skip this edge
        continue;
      }
    }

    // Get intersection and update candidate interval
    prec_t t = num / n_dot_c;
    if (n_dot_c > 0) {
      // Ray is exiting polygon
      if (t < t_out) {
        t_out = t;
      }
    } else {
      // Ray is entering polygon
      if (t > t_in) {
        t_in = t;
      }
    }

    // Early out
    if (t_in > t_out) {
      return false;
    }
  }

  // If candidate interval is not empty then set the clip segment
  clip_segment = {a_ + c * t_in, a_ + c * t_out};
  return true;
}

// explicit instantiations
template class LineSegment2<PointXY<float>>;
template class LineSegment2<PointXY<double>>;
template class LineSegment2<GeoPoint<float>>;
template class LineSegment2<GeoPoint<double>>;

} // namespace midgard
} // namespace valhalla
