#include "geo/clipper2.h"

namespace valhalla {
namespace geo {

Clipper2::Clipper2()
    : minx_(0),
      maxx_(0),
      miny_(0),
      maxy_(0) {
}

Clipper2::~Clipper2() {
}

unsigned int Clipper2::Clip(const AABB2& bdry, std::vector<Point2>& pts,
                            bool closed) {
  // Save the boundary
  minx_ = bdry.minx();
  maxx_ = bdry.maxx();
  miny_ = bdry.miny();
  maxy_ = bdry.maxy();

  // Temporary vertex list
  std::vector<Point2> tmp_pts;

  // Clip against each edge in succession. If at any time there are
  // no points kLeft we return 0 (everything outside)
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

int Clipper2::ClipAgainstEdge(const ClipEdge bdry, bool closed,
                              std::vector<Point2>& vin,
                              std::vector<Point2>& vout) {
  // Clear the output vector
  vout.clear();

  // Special case for the 1st vertex. For polygons (closed) connect
  // last vertex to first vertex. For polylines repeat the first vertex
  unsigned int n = vin.size();
  unsigned int v1 = closed ? n - 1 : 0;

  // Loop through all vertices (edges are created from v1 to v2).
  bool v1in, v2in;
  for (unsigned int v2 = 0; v2 < n; v1 = v2, v2++) {
    // Relation of v1 and v2 with the bdry
    v1in = Inside(bdry, vin[v1]);
    v2in = Inside(bdry, vin[v2]);

    // Add vertices to the output list based on the 4 cases
    if (v1in && v2in) {
      // Case 1: both vertices inside - output v2
      Add(vin[v2], vout);
    } else if (!v1in && v2in) {
      // Case 2: v1 is outside and v2 is inside - clip and
      // add intersection followed by v2
      Add(ClipIntersection(bdry, vin[v2], vin[v1]), vout);
      Add(vin[v2], vout);
    } else if (v1in && !v2in) {
      // Case 3: v1 is inside and v2 is outside - clip and
      // add the intersection
      Add(ClipIntersection(bdry, vin[v1], vin[v2]), vout);
    }
    // Case 4: both are outside - do nothing
  }
  return vout.size();
}

Point2 Clipper2::ClipIntersection(const ClipEdge bdry, const Point2& insidept,
                                  const Point2& outsidept) {
  float t = 0.0;
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
  return Point2(inx + t * dx, iny + t * dy);
}

bool Clipper2::Inside(const ClipEdge edge, const Point2& v) const {
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
void Clipper2::Add(const Point2& pt, std::vector<Point2>& vout) {
  if (vout.size() == 0 || *(vout.end() - 1) != pt) {
    vout.push_back(pt);
  }
}

}
}
