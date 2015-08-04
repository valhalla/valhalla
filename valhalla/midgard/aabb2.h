#ifndef VALHALLA_MIDGARD_AABB2_H_
#define VALHALLA_MIDGARD_AABB2_H_

#include <vector>

#include <valhalla/midgard/point2.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/linesegment2.h>

namespace valhalla {
namespace midgard {

/**
 * Axis Aligned Bounding Box (2 dimensional). This is a template class that
 * works with Point2 (Euclidean x,y) or PointLL (latitude,longitude).
 */
template <class coord_t>
class AABB2 {
 public:
  /**
   * Default constructor.
   */
  AABB2();

  /**
   * Construct an AABB given a minimum and maximum point.
   * @param  minpt  Minimum point (x,y)
   * @param  maxpt  Maximum point (x,y)
   */
  AABB2(const coord_t& minpt, const coord_t& maxpt);

  /**
   * Constructor with specified bounds.
   * @param   minx    Minimum x of the bounding box.
   * @param   miny    Minimum y of the bounding box.
   * @param   maxx    Maximum x of the bounding box.
   * @param   maxy    Maximum y of the bounding box.
   */
  AABB2(const float minx, const float miny,
        const float maxx, const float maxy);

  /**
   * Construct an AABB given a list of points.
   * @param  pts  Vertex list.
   */
  AABB2(const std::vector<coord_t>& pts);

  /**
   * Equality operator.
   * @param   r2  Bounding box to compare to.
   * @return  Returns true if the 2 bounding boxes are equal.
   */
  bool operator ==(const AABB2& r2) const;

  /**
   * Get the minimum x
   * @return  Returns minimum x.
   */
  float minx() const;
  /**
   * Get the maximum x
   * @return  Returns maximum x.
   */
  float maxx() const;

  /**
   * Get the minimum y
   * @return  Returns minimum y.
   */
  float miny() const;

  /**
   * Get the maximum y
   * @return  Returns maximum y.
   */
  float maxy() const;

  /**
   * Get the point at the minimum x,y.
   * @return  Returns the min. point.
   */
  coord_t minpt() const;

  /**
   * Get the point at the maximum x,y.
   * @return  Returns the max. point.
   */
  coord_t maxpt() const;

  /**
   * Creates an AABB given a list of points.
   * @param  pts  Vertex list.
   */
  void Create(const std::vector<coord_t>& pts);

  /**
   * Gets the center of the bounding box.
   * @return  Returns the center point of the bounding box.
   */
  coord_t Center() const;

  /**
   * Tests if a specified point is within the bounding box.
   * @param   pt   Point to test.
   * @return  Returns true if within the bounding box and false if outside
   *          the extent. Points that lie along the minimum x or y edge are
   *          considered to be inside, while points that lie on the maximum
   *          x or y edge are considered to be outside.
   */
  bool Contains(const coord_t& pt) const;

  /**
   * Checks to determine if another bounding box is completely inside this
   * bounding box.
   * @param   r2    Test bounding box
   * @return  Returns false if the bounding box is not entirely inside,
   *          true if it is inside.
   */
  bool Contains(const AABB2& r2) const;

  /**
   * Test if this bounding box intersects another bounding box.
   * @param    r2   Other bounding box to test intersection against.
   * @return   Returns true if the bounding box intersect, false if not.
   */
  bool Intersects(const AABB2& r2) const;

  /**
   * Tests whether the segment intersects the bounding box.
   * @param   seg  Line segment
   * @return  Returns true if the segment intersects (or lies completely
   *          within) the bounding box.
   */
  bool Intersect(const LineSegment2<coord_t>& seg) const;

  /**
   * Tests whether the segment intersects the bounding box.
   * @param   a  Endpoint of the segment
   * @param   b  Endpoint of the segment
   * @return  Returns true if the segment intersects (or lies completely
   *          within) the bounding box.
   */
  bool Intersect(const coord_t& a, const coord_t& b) const;

  /**
   * Gets the width of the bounding box.
   * @return  Returns the width of this bounding box.
   */
  float Width() const;

  /**
   * Gets the height of the bounding box.
   * @return  Returns the height of this bounding box.
   */
  float Height() const;

  /**
   * Expands (if necessary) the bounding box to include the specified
   * bounding box.
   * @param  r2  Bounding bounding box to "combine" with this bounding box.
   */
  void Expand(const AABB2& r2);

 protected:
  // Minimum and maximum x,y values (lower left and upper right corners)
  // of a rectangle / bounding box.
  float minx_;
  float miny_;
  float maxx_;
  float maxy_;
};

}
}

#endif  // VALHALLA_MIDGARD_AABB2_H_
