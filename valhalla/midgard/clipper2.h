#ifndef VALHALLA_MIDGARD_CLIPPER2_H_
#define VALHALLA_MIDGARD_CLIPPER2_H_

#include <vector>

#include <valhalla/midgard/point2.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/aabb2.h>

namespace valhalla {
namespace midgard {

/**
 * Clipping support. Clips a polygon or polyline to a rectangular
 * boundary. This is a template class that works with Point2
 * (Euclidean x,y) or PointLL (latitude,longitude).
 */
template <class coord_t>
class Clipper2 {
 public:
  /**
   * Clips the input set of vertices to the specified boundary.  Uses a
   * method where the shape is clipped against each edge in succession.
   * @param    bbox     Rectangular boundary (min,max x,y)
   * @param    pts      In/Out. List of points in the polyline/polygon.
   *                    After clipping this list is clipped to the boundary.
   * @param    closed   Is the shape closed?
   * @return   Returns the number of vertices in the clipped shape. May have
   *           0 vertices if none of the input polyline intersects or lies
   *           within the boundary.
   */
  uint32_t Clip(const AABB2<coord_t>& bbox, std::vector<coord_t>& pts,
                const bool closed);

 private:
  // Edge to clip against
  enum ClipEdge { kLeft, kRight, kBottom, kTop };

  // Boundary stored as individual min,max for simplicity
  float minx_;
  float maxx_;
  float miny_;
  float maxy_;

  /**
   * Clips the polyline/polygon against a single edge.
   * @param  edge    Edge to clip against.
   * @param  closed  True if the vertices form a polygon.
   * @param  vin     List of input vertices.
   * @param  vout    Output vertices.
   * @return  Returns the number of vertices after clipping. The list
   *          of vertices is in vout.
   */
  uint32_t ClipAgainstEdge(const ClipEdge edge, const bool closed,
            const std::vector<coord_t>& vin,
            std::vector<coord_t>& vout);

  /**
   * Finds the intersection of the segment from insidept to outsidept with the
   * specified boundary edge.  Finds the intersection using the parametric
   * line equation.
   * @param  edge  Edge to intersect.
   * @param  insidept  Vertex inside with respect to the edge.
   * @param  outsidept Vertex outside with respect to the edge.
   * @return  Returns the intersection of the segment with the edge.
   */
  coord_t ClipIntersection(const ClipEdge edge, const coord_t& insidept,
                          const coord_t& outsidept);

  /**
   * Tests if the vertex is inside the rectangular boundary with respect to
   * the specified edge.
   * @param   edge   Edge to test.
   * @param   v      Vertex to test.
   * @return  Returns true if the point is inside with respect to the edge,
   *          false if it is outside.
   */
  bool Inside(const ClipEdge edge, const coord_t& v) const;

  /**
   * Adds a vertex to the output vector if not equal to the prior.
   * @param  pt    Vertex to add.
   * @param  vout  Vertex list.
   */
  void Add(const coord_t& pt, std::vector<coord_t>& vout);
};

}
}

#endif  // VALHALLA_MIDGARD_CLIPPER2_H_

