#ifndef VALHALLA_MIDGARD_POLYLINE2_H_
#define VALHALLA_MIDGARD_POLYLINE2_H_

#include <cstdint>
#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/linesegment2.h>
#include <valhalla/midgard/point2.h>
#include <valhalla/midgard/pointll.h>

#include <tuple>
#include <unordered_set>

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
  float Length() const {
    float length = 0;
    if (pts_.size() < 2) {
      return length;
    }
    for (auto p = std::next(pts_.cbegin()); p != pts_.cend(); ++p) {
      length += std::prev(p)->Distance(*p);
    }
    return length;
  }

  /**
   * Compute the length of the specified polyline.
   * @param   pts  Polyline vertices.
   * @return  Returns the length of the polyline.
   */
  template <class container_t> static float Length(const container_t& pts) {
    float length = 0;
    if (pts.size() < 2) {
      return length;
    }
    for (auto p = std::next(pts.cbegin()); p != pts.cend(); ++p) {
      length += std::prev(p)->Distance(*p);
    }
    return length;
  }

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
  std::tuple<coord_t, float, int> ClosestPoint(const coord_t& pt) const {
    return pt.ClosestPoint(pts_);
  }

  /**
   * Generalize this polyline.
   * @param   t         Generalization tolerance.
   * @param   indices   List of indices of points not to generalize
   * @return  returns the number of points in the generalized polyline.
   */
  uint32_t Generalize(const float t, const std::unordered_set<size_t>& indices = {}) {
    // Create a vector for the output shape. Recursively call Douglass-Peucker
    // method to generalize the polyline. Square the error tolerance to avoid
    // sqrts.
    Generalize(pts_, t, indices);
    return pts_.size();
  }

  /**
   * Get a generalized polyline from this polyline. This polyline remains
   * unchanged.
   * @param    t   Generalization tolerance.
   * @param    indices   List of indices of points not to generalize
   * @return   returns the generalized polyline.
   */
  Polyline2 GeneralizedPolyline(const float t, const std::unordered_set<size_t>& indices = {}) {
    // Recursively call Douglass-Peucker method to generalize the polyline.
    // Square the error tolerance to avoid sqrts.
    Polyline2 generalized(pts_);
    generalized.Generalize(t, indices);
    return generalized;
  }

  /**
   * Generalize the given list of points
   *
   * @param polyline    the list of points
   * @param epsilon     the tolerance used in removing points
   * @param  indices    list of indices of points not to generalize
   */
  template <class container_t>
  static void
  Generalize(container_t& polyline, float epsilon, const std::unordered_set<size_t>& indices = {}) {
    // any epsilon this low will have no effect on the input nor will any super short input
    if (epsilon <= 0.f || polyline.size() < 3)
      return;

    // the recursive bit
    epsilon *= epsilon;
    std::function<void(typename container_t::iterator, size_t, typename container_t::iterator,
                       size_t)>
        peucker;
    peucker = [&peucker, &polyline, epsilon, &indices](typename container_t::iterator start, size_t s,
                                                       typename container_t::iterator end, size_t e) {
      // find the point furthest from the line
      float dmax = std::numeric_limits<float>::lowest();
      typename container_t::iterator itr;
      LineSegment2<coord_t> l{*start, *end};
      size_t j = e - 1, k;
      coord_t tmp;
      for (auto i = std::prev(end); i != start; --i, --j) {
        // special points we dont want to generalize no matter what take precidence
        if (indices.find(j) != indices.end()) {
          itr = i;
          dmax = epsilon;
          k = j;
          break;
        }

        // if this is the highest frequency detail so far
        auto d = l.DistanceSquared(*i, tmp);
        if (d > dmax) {
          itr = i;
          dmax = d;
          k = j;
        }
      }

      // there are some high frequency details between start and end
      // so we need to look for flatter sections between them
      if (dmax >= epsilon) {
        // we recurse from right to left for two reasons:
        // 1. we want to preserve iterator validity in the vector version
        // 2. its the only way to preserve the indices in the keep set
        if (e - k > 1)
          peucker(itr, k, end, e);
        if (k - s > 1)
          peucker(start, s, itr, k);
      } // nothing sticks out between start and end so simplify everything between away
      else
        polyline.erase(std::next(start), end);
    };

    // recurse!
    peucker(polyline.begin(), 0, std::prev(polyline.end()), polyline.size() - 1);
  }

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

#endif // VALHALLA_MIDGARD_POLYLINE2_H_
