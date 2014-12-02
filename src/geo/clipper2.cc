#include "geo/clipper2.h"

namespace valhalla{
namespace geo{

  Clipper2::Clipper2():minx_(0),maxx_(0),miny_(0),maxy_(0) { }

  Clipper2::~Clipper2() { }

  unsigned int Clipper2::Clip(const AABB2& bdry, std::vector<Point2>& pts, bool closed) {
    // Save the boundary
    minx_ = bdry.minx();
    maxx_ = bdry.maxx();
    miny_ = bdry.miny();
    maxy_ = bdry.maxy();

    // If the shape is closed make sure the last vertex equals the first
    // or force closure by appending a vertex
    int n = pts.size();
    if (pts[0] != pts[n-1]) {
      pts.push_back(pts[0]);
    }

    // Temporary vertex list
    std::vector<Point2> tmp_pts;

    // Clip against each edge in succession. If at any time there are
    // no points kLeft we return 0 (everything outside)
    if (clipAgainstEdge(kLeft, pts, tmp_pts) == 0) {
      return 0;
    }
    if (clipAgainstEdge(kRight, tmp_pts, pts) == 0) {
      return 0;
    }
    if (clipAgainstEdge(kBottom, pts, tmp_pts) == 0) {
      return 0;
    }
    if (clipAgainstEdge(kTop, tmp_pts, pts) == 0) {
      return 0;
    }
    // Return number of vertices in the clipped shape
    return pts.size();
  }

  int Clipper2::clipAgainstEdge(const ClipEdge bdry, std::vector<Point2>& vin,
            std::vector<Point2>& vout) {
    // Clear the output vector
    vout.clear();

    // Loop through all vertices (edges are created from v1 to v2)
    bool v1in, v2in;
    for (int v1 = 0, v2 = 1, n = vin.size(); v2 < n; v1++, v2++) {
      // Relation of v1 and v2 with the bdry
      v1in = inside(bdry, vin[v1]);
      v2in = inside(bdry, vin[v2]);

      // Add vertices to the output list based on the 4 cases
      if (v1in && v2in) {
        // Case 1: both vertices inside - output v2
        vout.push_back(vin[v2]);
      }
      else if (!v1in && v2in) {
        // Case 2: v1 is outside and v2 is inside - clip and
        // add intersection followed by v2
        vout.push_back(clipIntersection(bdry, vin[v2], vin[v1]));
        vout.push_back(vin[v2]);
      }
      else if (v1in && !v2in) {
        // Case 3: v1 is inside and v2 is outside - clip and
        // add the intersection
        vout.push_back(clipIntersection(bdry, vin[v1], vin[v2]));
      }
      // Case 4: both are outside - do nothing
    }
    return vout.size();
  }

  Point2 Clipper2::clipIntersection(const ClipEdge bdry, const Point2& insidept,
                         const Point2& outsidept) {
    float t   = 0.0;
    float inx = insidept.x();
    float iny = insidept.y();
    float dx  = outsidept.x() - inx;
    float dy  = outsidept.y() - iny;
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
    return Point2(inx + t * dx, iny + t * dy);
  }

  bool Clipper2::inside(const ClipEdge edge, const Point2& v) const {
    switch (edge) {
      case kLeft:   return (v.x() > minx_);
      case kRight:  return (v.x() < maxx_);
      case kBottom: return (v.y() > miny_);
      case kTop:    return (v.y() < maxy_);
    }
    return false;
  }

}
}
