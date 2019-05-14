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
  Polyline2();

  /**
   * Constructor given a list of points.
   * @param  pts  List of points.
   */
  Polyline2(const std::vector<coord_t>& pts);

  /**
   * Gets the list of points.
   * @return  Returns the list of points.
   */
  std::vector<coord_t>& pts();

  /**
   * Add a point to the polyline. Checks to see if the input point is
   * equal to the current endpoint of the polyline and does not add the
   * point if it is equal.
   * @param  p  Point to add to the polyline.
   */
  void Add(const coord_t& p);

  /**
   * Finds the length of the polyline by accumulating the length of all
   * segments.
   * @return    Returns the length of the polyline.
   */
  float Length() const;

  /**
   * Compute the length of the specified polyline.
   * @param   pts  Polyline vertices.
   * @return  Returns the length of the polyline.
   */
  template <class container_t> static float Length(const container_t& pts);

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
  std::tuple<coord_t, float, int> ClosestPoint(const coord_t& pt) const;

  /**
   * Generalize this polyline.
   * @param   t         Generalization tolerance.
   * @param   indices   List of indices of points not to generalize
   * @return  returns the number of points in the generalized polyline.
   */
  uint32_t Generalize(const float t, const std::unordered_set<size_t>& indices = {});

  /**
   * Get a generalized polyline from this polyline. This polyline remains
   * unchanged.
   * @param    t   Generalization tolerance.
   * @param    indices   List of indices of points not to generalize
   * @return   returns the generalized polyline.
   */
  Polyline2 GeneralizedPolyline(const float t, const std::unordered_set<size_t>& indices = {});

  /**
   * Generalize the given list of points
   *
   * @param polyline    the list of points
   * @param epsilon     the tolerance used in removing points
   * @param  indices    list of indices of points not to generalize
   */
  template <class container_t>
  static void
  Generalize(container_t& polyline, float epsilon, const std::unordered_set<size_t>& indices = {});

  /**
   * Clip this polyline to the specified bounding box.
   * @param box  Bounding box to clip this polyline to.
   * @return  Returns the number of vertices in the clipped polygon.
   */
  uint32_t Clip(const AABB2<coord_t>& box);

  /**
   * Gets a polyline clipped to the supplied bounding box. This polyline
   * remains unchanged.
   * @param  box  AABB (rectangle) to which the polyline is clipped.
   */
  Polyline2 ClippedPolyline(const AABB2<coord_t>& box);

  /**
   * Checks if the polylines are equal
   * @param   other  the other polyline to check against this one
   * @return         true if they are equal false otherwise
   */
  bool operator==(const Polyline2<coord_t>& other) const;

protected:
  // Polyline points
  std::vector<coord_t> pts_;
};

} // namespace midgard
} // namespace valhalla

#endif // VALHALLA_MIDGARD_POLYLINE2_H_
