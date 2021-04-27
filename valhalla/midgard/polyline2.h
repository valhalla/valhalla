#pragma once

#include <cstdint>
#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/linesegment2.h>
#include <valhalla/midgard/point2.h>
#include <valhalla/midgard/pointll.h>

#include <list>
#include <tuple>
#include <unordered_set>
#include <vector>

namespace valhalla {
namespace midgard {

/**
 * 2-D polyline. This is a template class that works with Point2
 * (Euclidean x,y) or PointLL (latitude,longitude).
 */
template <class coord_t> class Polyline2 {
public:
  Polyline2() {
  }

  /**
   * Constructor given a list of points.
   * @param  pts  List of points.
   */
  Polyline2(const std::vector<coord_t>& pts) : pts_{pts} {};

  /**
   * Gets the list of points.
   * @return  Returns the list of points.
   */
  std::vector<coord_t>& pts() {
    return pts_;
  };

  /**
   * Add a point to the polyline. Checks to see if the input point is
   * equal to the current endpoint of the polyline and does not add the
   * point if it is equal.
   * @param  p  Point to add to the polyline.
   */
  void Add(const coord_t& p) {
    uint32_t n = pts_.size();
    if (n == 0 || !(p == pts_[n - 1])) {
      pts_.push_back(p);
    }
  }

  /**
   * Finds the length of the polyline by accumulating the length of all
   * segments.
   * @return    Returns the length of the polyline.
   */
  typename coord_t::value_type Length() const;

  /**
   * Compute the length of the specified polyline.
   * @param   pts  Polyline vertices.
   * @return  Returns the length of the polyline.
   */
  template <class container_t> static typename coord_t::value_type Length(const container_t& pts);

  /**
   * In an O(n^2) manner (only useful for debugging/testing), checks if there are any
   * segments in the polyline that are intersecting. If so, returns those intersection
   * points.
   * @param   pts  Polyline vertices.
   * @return  List of intersections found, if any.
   */
  std::vector<coord_t> GetSelfIntersections();

  /**
   * Finds the closest point to the supplied point as well as the distance
   * to that point and the index of the segment where the closest
   * point lies.
   * @param   pt     point to find distance from
   * @return  tuple of <Closest point along the polyline,
   *                    Distance in meters of the closest point,
   *                    Index of the segment of the polyline which contains
   *                      the closest point >
   */
  std::tuple<coord_t, typename coord_t::value_type, int> ClosestPoint(const coord_t& pt) const {
    return pt.ClosestPoint(pts_);
  }

  /**
   * Generalize this polyline.
   * @param   t         Generalization tolerance.
   * @param   indices   List of indices of points not to generalize
   * @param avoid_self_intersection  avoid simplifications that cause self-intersection
   * @return  returns the number of points in the generalized polyline.
   */
  uint32_t Generalize(const typename coord_t::value_type t,
                      const std::unordered_set<size_t>& indices = {},
                      bool avoid_self_intersection = false) {
    // Create a vector for the output shape. Recursively call Douglass-Peucker
    // method to generalize the polyline. Square the error tolerance to avoid
    // sqrts.
    Generalize(pts_, t, indices, avoid_self_intersection);
    return pts_.size();
  }

  /**
   * Get a generalized polyline from this polyline. This polyline remains
   * unchanged.
   * @param    t   Generalization tolerance.
   * @param    indices   List of indices of points not to generalize
   * @param avoid_self_intersection  avoid simplifications that cause self-intersection
   * @return   returns the generalized polyline.
   */
  Polyline2 GeneralizedPolyline(const typename coord_t::value_type t,
                                const std::unordered_set<size_t>& indices = {},
                                bool avoid_self_intersection = false) {
    // Recursively call Douglass-Peucker method to generalize the polyline.
    // Square the error tolerance to avoid sqrts.
    Polyline2 generalized(pts_);
    generalized.Generalize(t, indices, avoid_self_intersection);
    return generalized;
  }

  /**
   * Generalize the given list of points
   *
   * @param polyline    the list of points
   * @param epsilon     the tolerance used in removing points
   * @param  indices    list of indices of points not to generalize
   * @param avoid_self_intersection  avoid simplifications that cause self-intersection
   */
  template <class container_t>
  static void Generalize(container_t& polyline,
                         typename coord_t::value_type epsilon,
                         const std::unordered_set<size_t>& indices = {},
                         bool avoid_self_intersection = false);

  /**
   * Clip this polyline to the specified bounding box.
   * @param box  Bounding box to clip this polyline to.
   * @return  Returns the number of vertices in the clipped polygon.
   */
  uint32_t Clip(const AABB2<coord_t>& box) {
    return box.Clip(pts_, false);
  }

  /**
   * Gets a polyline clipped to the supplied bounding box. This polyline
   * remains unchanged.
   * @param  box  AABB (rectangle) to which the polyline is clipped.
   */
  Polyline2 ClippedPolyline(const AABB2<coord_t>& box) {
    // Copy the polyline points to a temporary vector
    std::vector<coord_t> pts = pts_;
    box.Clip(pts, false);
    return Polyline2(pts);
  }

  /**
   * Checks if the polylines are equal
   * @param   other  the other polyline to check against this one
   * @return         true if they are equal false otherwise
   */
  bool operator==(const Polyline2<coord_t>& other) const {
    return pts_.size() == other.pts_.size() &&
           std::equal(pts_.begin(), pts_.end(), other.pts_.begin());
  }

protected:
  // Polyline points
  std::vector<coord_t> pts_;
};

} // namespace midgard
} // namespace valhalla