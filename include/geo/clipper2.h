#ifndef __clipper2_h__
#define __clipper2_h__

#include <vector>

#include "aabb2.h"
#include "point2.h"

namespace valhalla{
namespace geo{

/**
 * Clipping support. Clips a polygon or polyline to a rectangular
 * boundary. If this is used to clip a polyline to a display region,
 * need to set the boundary N pixels from the display boundary since
 * segments may be created along the boundary edges. Template class, so
 * the supplied shape can be D_POINT, F_POINT, L_POINT, whatever has an
 * x, y data member.
 * @author  David W. Nesbitt
 */
class Clipper2 {
 public:
  /**
   * Constructor
   */
  Clipper2();

  /**
   * Destructor
   */
  ~Clipper2();

  /**
   * Clips the input set of vertices to the specified boundary.  Uses a
   * method where the shape is clipped against each edge in succession.
   * @param    bdry     Rectangular boundary (min,max x,y)
   * @param    pts      In/Out. List of points in the polyline/polygon.
   *                    After clipping this list is clipped to the boundary.
   * @param    closed   Is the shape closed?
   * @return   Returns the number of vertices in the clipped shape. May have
   *           0 vertices if none of the input polyline intersects or lies
   *           within the boundary.
   */
   unsigned int Clip(const AABB2& bdry, std::vector<Point2>& pts, bool closed);

 private:
  // Edge to clip against
  enum ClipEdge { kLeft, kRight, kBottom, kTop };

  // Boundary stored as individual min,max for simplicity
  float minx_;
  float maxx_;
  float miny_;
  float maxy_;

  // Clips the polyline/polygon against a single edge
  int clipAgainstEdge(const ClipEdge bdry, std::vector<Point2>& vin,
            std::vector<Point2>& vout);

  // Finds the intersection of the segment from insidept to outsidept with the
  // specified boundary edge.  It finds the intersection using the
  // parametric line equation.
  Point2 clipIntersection(const ClipEdge bdry, const Point2& insidept,
                         const Point2& outsidept);

  // Tests if the vertex is inside the rectangular boundary with respect to
  // the specified edge. Returns true if the point is inside the view
  // boundary, false if it is outside.
  bool inside(const ClipEdge edge, const Point2& v) const;
};

}
}

#endif

