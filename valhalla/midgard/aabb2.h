#ifndef VALHALLA_MIDGARD_AABB2_H_
#define VALHALLA_MIDGARD_AABB2_H_

#include <cstdint>
#include <valhalla/midgard/linesegment2.h>
#include <valhalla/midgard/point2.h>
#include <valhalla/midgard/pointll.h>
#include <vector>

namespace valhalla {
namespace midgard {

/**
 * Axis Aligned Bounding Box (2 dimensional). This is a template class that
 * works with Point2 (Euclidean x,y) or PointLL (latitude,longitude).
 *
 * TODO - make sure min < max, throw exception if not?
 * TODO: merge clipper2 class into this one, its basically a thin scaffold around
 * this class anyway and has some useful intersection stuff
 */
template <class coord_t> class AABB2 {
public:
  using x_t = typename coord_t::first_type;
  using y_t = typename coord_t::second_type;

  /**
   * Default constructor. Sets all min,max values to 0.
   */
  AABB2() : minx_(0.0f), miny_(0.0f), maxx_(0.0f), maxy_(0.0f) {
  }

  /**
   * Construct an AABB given a minimum and maximum point.
   * @param  minpt  Minimum point (x,y)
   * @param  maxpt  Maximum point (x,y)
   */
  AABB2(const coord_t& minpt, const coord_t& maxpt) {
    minx_ = minpt.x();
    miny_ = minpt.y();
    maxx_ = maxpt.x();
    maxy_ = maxpt.y();
  }

  /**
   * Constructor with specified bounds.
   * @param   minx    Minimum x of the bounding box.
   * @param   miny    Minimum y of the bounding box.
   * @param   maxx    Maximum x of the bounding box.
   * @param   maxy    Maximum y of the bounding box.
   */
  AABB2(const x_t minx, const y_t miny, const x_t maxx, const y_t maxy)
      : minx_(minx), miny_(miny), maxx_(maxx), maxy_(maxy) {
  }

  /**
   * Construct an AABB given a list of points.
   * @param  pts  Vertex list.
   */
  AABB2(const std::vector<coord_t>& pts) {
    Create(pts);
  }

  /**
   * Equality operator.
   * @param   r2  Bounding box to compare to.
   * @return  Returns true if the 2 bounding boxes are equal.
   */
  bool operator==(const AABB2& r2) const {
    return minx_ == r2.minx_ && maxx_ == r2.maxx_ && miny_ == r2.miny_ && maxy_ == r2.maxy_;
  }

  /**
   * Get the minimum x
   * @return  Returns minimum x.
   */
  x_t minx() const {
    return minx_;
  }

  /**
   * Get the maximum x
   * @return  Returns maximum x.
   */
  x_t maxx() const {
    return maxx_;
  }

  /**
   * Get the minimum y
   * @return  Returns minimum y.
   */
  y_t miny() const {
    return miny_;
  }

  /**
   * Get the maximum y
   * @return  Returns maximum y.
   */
  y_t maxy() const {
    return maxy_;
  }

  /**
   * Get the point at the minimum x,y.
   * @return  Returns the min. point.
   */
  coord_t minpt() const {
    return coord_t(minx_, miny_);
  }

  /**
   * Get the point at the maximum x,y.
   * @return  Returns the max. point.
   */
  coord_t maxpt() const {
    return coord_t(maxx_, maxy_);
  }

  /**
   * Creates an AABB given a list of points.
   * @param  pts  Vertex list.
   */
  void Create(const std::vector<coord_t>& pts);

  /**
   * Gets the width of the bounding box.
   * @return  Returns the width of this bounding box.
   */
  x_t Width() const {
    return maxx_ - minx_;
  }

  /**
   * Gets the height of the bounding box.
   * @return  Returns the height of this bounding box.
   */
  y_t Height() const {
    return maxy_ - miny_;
  }

  /**
   * Gets the center of the bounding box.
   * @return  Returns the center point of the bounding box.
   */
  coord_t Center() const {
    return coord_t((minx_ + maxx_) * 0.5f, (miny_ + maxy_) * 0.5f);
  }

  /**
   * Tests if a specified point is within the bounding box.
   * @param   pt   Point to test.
   * @return  Returns true if within the bounding box and false if outside
   *          the extent. Points that lie along the minimum x or y edge are
   *          considered to be inside, while points that lie on the maximum
   *          x or y edge are considered to be outside.
   */
  bool Contains(const coord_t& pt) const {
    return (pt.x() >= minx_ && pt.y() >= miny_ && pt.x() < maxx_ && pt.y() < maxy_);
  }

  /**
   * Checks to determine if another bounding box is completely inside this
   * bounding box.
   * @param   r2    Test bounding box
   * @return  Returns false if the bounding box is not entirely inside,
   *          true if it is inside.
   */
  bool Contains(const AABB2& r2) const {
    return (Contains(r2.minpt()) && Contains(r2.maxpt()));
  }

  /**
   * Computes the intersection of this bounding box with another.
   * @param  bbox  Other bounding box.
   * @return Returns a bounding box that is the intersection of the 2 bounding
   *         boxes. If the bounding boxes do not intersect a bounding box with
   *         no area is returned (all min,max values are 0).
   */
  AABB2 Intersection(const AABB2& bbox) const;

  /**
   * Test if this bounding box intersects another bounding box. The bounding
   * boxes do NOT intersect if the other bounding box (r2) is entirely LEFT,
   * BELOW, RIGHT, or ABOVE this bounding box. Otherwise they intersect.
   * @param    r2   Other bounding box to test intersection against.
   * @return   Returns true if the bounding box intersect, false if not.
   */
  bool Intersects(const AABB2& r2) const {

    return !((r2.minx() < minx_ && r2.maxx() < minx_) || (r2.miny() < miny_ && r2.maxy() < miny_) ||
             (r2.minx() > maxx_ && r2.maxx() > maxx_) || (r2.miny() > maxy_ && r2.maxy() > maxy_));
  }

  /**
   * Tests whether the segment intersects the bounding box.
   * @param   seg  Line segment
   * @return  Returns true if the segment intersects (or lies completely
   *          within) the bounding box.
   */
  bool Intersects(const LineSegment2<coord_t>& seg) const {
    return Intersects(seg.a(), seg.b());
  }

  /**
   * Tests whether the segment intersects the bounding box.
   * @param   a  Endpoint of the segment
   * @param   b  Endpoint of the segment
   * @return  Returns true if the segment intersects (or lies completely
   *          within) the bounding box.
   */
  bool Intersects(const coord_t& a, const coord_t& b) const;

  /**
   * Tests whether the circle intersects the bounding box.
   * @param   center  center of circle
   * @param   radius  radius of the circle
   * @return  Returns true if the circle intersects (or lies completely
   *          within) the bounding box.
   */
  bool Intersects(const coord_t& center, float radius) const;

  /**
   * Clips the input set of vertices to the specified boundary.  Uses a
   * method where the shape is clipped against each edge in succession.
   * @param    pts      In/Out. List of points in the polyline/polygon.
   *                    After clipping this list is clipped to the boundary.
   * @param    closed   Is the shape closed?
   * @return   Returns the number of vertices in the clipped shape. May have
   *           0 vertices if none of the input polyline intersects or lies
   *           within the boundary.
   */
  uint32_t Clip(std::vector<coord_t>& pts, const bool closed) const;

  /**
   * Expands (if necessary) the bounding box to include the specified
   * bounding box.
   * @param  r2  Bounding bounding box to "combine" with this bounding box.
   */
  void Expand(const AABB2& r2) {
    if (r2.minx() < minx_) {
      minx_ = r2.minx();
    }
    if (r2.miny() < miny_) {
      miny_ = r2.miny();
    }
    if (r2.maxx() > maxx_) {
      maxx_ = r2.maxx();
    }
    if (r2.maxy() > maxy_) {
      maxy_ = r2.maxy();
    }
  }

  /**
   * Expands (if necessary) the bounding box to include the specified point.
   * @param  point  Point to "add" to this bounding box.
   * @return returns true if the bbox was expanded
   */
  bool Expand(const coord_t& point) {
    bool expanded = false;
    if (point.x() < minx_) {
      minx_ = point.x();
      expanded = true;
    }
    if (point.y() < miny_) {
      miny_ = point.y();
      expanded = true;
    }
    if (point.x() > maxx_) {
      maxx_ = point.x();
      expanded = true;
    }
    if (point.y() > maxy_) {
      maxy_ = point.y();
      expanded = true;
    }
    return expanded;
  }

protected:
  // Edge to clip against
  enum ClipEdge { kLeft, kRight, kBottom, kTop };

  // Minimum and maximum x,y values (lower left and upper right corners)
  // of a rectangle / bounding box.
  x_t minx_;
  y_t miny_;
  x_t maxx_;
  y_t maxy_;

  /**
   * Clips the polyline/polygon against a single edge.
   * @param  edge    Edge to clip against.
   * @param  closed  True if the vertices form a polygon.
   * @param  vin     List of input vertices.
   * @param  vout    Output vertices.
   * @return  Returns the number of vertices after clipping. The list
   *          of vertices is in vout.
   */
  uint32_t ClipAgainstEdge(const ClipEdge edge,
                           const bool closed,
                           const std::vector<coord_t>& vin,
                           std::vector<coord_t>& vout) const;

  /**
   * Finds the intersection of the segment from insidept to outsidept with the
   * specified boundary edge.  Finds the intersection using the parametric
   * line equation.
   * @param  edge  Edge to intersect.
   * @param  insidept  Vertex inside with respect to the edge.
   * @param  outsidept Vertex outside with respect to the edge.
   * @return  Returns the intersection of the segment with the edge.
   */
  coord_t
  ClipIntersection(const ClipEdge edge, const coord_t& insidept, const coord_t& outsidept) const;

  /**
   * Tests if the vertex is inside the rectangular boundary with respect to
   * the specified edge.
   * @param   edge   Edge to test.
   * @param   v      Vertex to test.
   * @return  Returns true if the point is inside with respect to the edge,
   *          false if it is outside.
   */
  bool Inside(const ClipEdge edge, const coord_t& v) const {
    switch (edge) {
      case kLeft:
        return (v.x() > minx_);
      case kRight:
        return (v.x() < maxx_);
      case kBottom:
        return (v.y() > miny_);
      default:
      case kTop:
        return (v.y() < maxy_);
    }
  }

  /**
   * Adds a vertex to the output vector if not equal to the prior.
   * @param  pt    Vertex to add.
   * @param  vout  Vertex list.
   */
  void Add(const coord_t& pt, std::vector<coord_t>& vout) const {
    if (vout.size() == 0 || vout.back() != pt) {
      vout.push_back(pt);
    }
  }
};

} // namespace midgard
} // namespace valhalla

#endif // VALHALLA_MIDGARD_AABB2_H_
