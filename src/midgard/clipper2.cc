#include "valhalla/midgard/clipper2.h"

namespace valhalla {
namespace midgard {

// Clips the input set of vertices to the specified boundary.  Uses a
// method where the shape is clipped against each edge in succession.
template <class coord_t>
uint32_t Clipper2<coord_t>::Clip(const AABB2<coord_t>& bbox,
                                 std::vector<coord_t>& pts,
                                 const bool closed) {
  // Save the boundary
  minx_ = bbox.minx();
  maxx_ = bbox.maxx();
  miny_ = bbox.miny();
  maxy_ = bbox.maxy();

  // Temporary vertex list
  std::vector<coord_t> tmp_pts;

  // Clip against each edge in succession. At each step we swap the roles
  // of the 2 vertex lists. If at any time there are no points remaining we
  // return 0 (everything outside)
  if (ClipAgainstEdge(kLeft, closed, pts, tmp_pts) == 0) {
    return 0;
  }
  if (ClipAgainstEdge(kRight, closed, tmp_pts, pts) == 0) {
    return 0;
  }
  if (ClipAgainstEdge(kBottom, closed, pts, tmp_pts) == 0) {
    return 0;
  }
  if (ClipAgainstEdge(kTop, closed, tmp_pts, pts) == 0) {
    return 0;
  }

  // Return number of vertices in the clipped shape
  return pts.size();
}

// Clips the polyline/polygon against a single edge.
template <class coord_t>
uint32_t Clipper2<coord_t>::ClipAgainstEdge(const ClipEdge bdry,
                              const bool closed,
                              const std::vector<coord_t>& vin,
                              std::vector<coord_t>& vout) {
  // Clear the output vector
  vout.clear();

  // Special case for the 1st vertex. For polygons (closed) connect
  // last vertex to first vertex. For polylines repeat the first vertex
  uint32_t n  = vin.size();
  uint32_t v1 = closed ? n - 1 : 0;

  // Loop through all vertices (edges are created from v1 to v2).
  for (uint32_t v2 = 0; v2 < n; v1 = v2, v2++) {
    // Relation of v1 and v2 with the bdry
    bool v1in = Inside(bdry, vin[v1]);
    bool v2in = Inside(bdry, vin[v2]);

    // Add vertices to the output list based on the 4 cases
    if (v1in && v2in) {
      // Both vertices inside - output v2
      Add(vin[v2], vout);
    } else if (!v1in && v2in) {
      // v1 is outside and v2 is inside - clip and add intersection
      // followed by v2
      Add(ClipIntersection(bdry, vin[v2], vin[v1]), vout);
      Add(vin[v2], vout);
    } else if (v1in && !v2in) {
      // v1 is inside and v2 is outside - clip and add the intersection
      Add(ClipIntersection(bdry, vin[v1], vin[v2]), vout);
    }
    // Both are outside - do nothing
  }
  return vout.size();
}

// Finds the intersection of the segment from insidept to outsidept with the
// specified boundary edge.  Finds the intersection using the parametric
// line equation.
template <class coord_t>
coord_t Clipper2<coord_t>::ClipIntersection(const ClipEdge bdry,
                                            const coord_t& insidept,
                                            const coord_t& outsidept) {
  float t = 0.0f;
  float inx = insidept.x();
  float iny = insidept.y();
  float dx = outsidept.x() - inx;
  float dy = outsidept.y() - iny;
  switch (bdry) {
    case kLeft:
      t = (minx_ - inx) / dx;
      break;
    case kRight:
      t = (maxx_ - inx) / dx;
      break;
    case kBottom:
      t = (miny_ - iny) / dy;
      break;
    case kTop:
      t = (maxy_ - iny) / dy;
      break;
  }

  // Return the intersection point.
  return coord_t(inx + t * dx, iny + t * dy);
}

// Tests if the vertex is inside the rectangular boundary with respect to
// the specified edge.
template <class coord_t>
bool Clipper2<coord_t>::Inside(const ClipEdge edge, const coord_t& v) const {
  switch (edge) {
    case kLeft:
      return (v.x() > minx_);
    case kRight:
      return (v.x() < maxx_);
    case kBottom:
      return (v.y() > miny_);
    case kTop:
      return (v.y() < maxy_);
  }
  return false;
}

// Add vertex to clip output (if not same as prior vertex)
template <class coord_t>
void Clipper2<coord_t>::Add(const coord_t& pt, std::vector<coord_t>& vout) {
  if (vout.size() == 0 || vout.back() != pt) {
    vout.push_back(pt);
  }
}

// Explicit instantiation
template class Clipper2<Point2>;
template class Clipper2<PointLL>;

}
}
