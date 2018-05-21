#include "midgard/aabb2.h"
#include "midgard/linesegment2.h"
#include "midgard/point2.h"
#include "midgard/pointll.h"
#include "midgard/util.h"

#include <cmath>
#include <list>

namespace {

template <class T> bool between(T v, T a, T b) {
  auto d = std::abs(a - b);
  return std::abs(v - a) <= d && std::abs(v - b) <= d;
}

} // namespace

namespace valhalla {
namespace midgard {

// Creates an AABB given a list of points.
template <class coord_t> void AABB2<coord_t>::Create(const std::vector<coord_t>& pts) {
  auto p = pts.begin();
  minx_ = p->x();
  maxx_ = minx_;
  miny_ = p->y();
  maxy_ = miny_;
  p++;
  for (; p < pts.end(); p++) {
    x_t x = p->x();
    if (x < minx_) {
      minx_ = x;
    } else if (x > maxx_) {
      maxx_ = x;
    }
    y_t y = p->y();
    if (y < miny_) {
      miny_ = y;
    } else if (y > maxy_) {
      maxy_ = y;
    }
  }
}

// Computes the intersection of this bounding box with another.
template <class coord_t> AABB2<coord_t> AABB2<coord_t>::Intersection(const AABB2& bbox) const {
  // If the bounding boxes do not intersect a bounding box with
  // no area is returned (all min,max values are 0).
  if (!Intersects(bbox)) {
    return {0.0f, 0.0f, 0.0f, 0.0f};
  }
  return {std::max(minx(), bbox.minx()), std::max(miny(), bbox.miny()), std::min(maxx(), bbox.maxx()),
          std::min(maxy(), bbox.maxy())};
}

// Tests whether the segment intersects the bounding box.
template <class coord_t> bool AABB2<coord_t>::Intersects(const coord_t& a, const coord_t& b) const {
  // Trivial case - either point within the bounding box
  if (Contains(a) || Contains(b)) {
    return true;
  }

  // Trivial rejection - both points outside any one bounding edge
  if ((a.x() < minx_ && b.x() < minx_) || // Both left
      (a.y() < miny_ && b.y() < miny_) || // Both below
      (a.x() > maxx_ && b.x() > maxx_) || // Both right
      (a.y() > maxy_ && b.y() > maxy_)) { // Both above
    return false;
  }

  // For LineSegment from a to b check which half plane each corner lies
  // in. If there is a change (different sign returned from the IsLeft
  // 2-D cross product for any one corner) then the AABB intersects the
  // segment. Any corner point on the segment half plane will have
  // IsLeft == 0 and we count as an intersection (trivial rejection cases
  // above mean the segment reaches the corner point)
  LineSegment2<coord_t> s(a, b);
  float s1 = s.IsLeft(coord_t(minx_, miny_));
  return ((s1 * s.IsLeft(coord_t(minx_, maxy_)) <= 0.0f) ||
          (s1 * s.IsLeft(coord_t(maxx_, maxy_)) <= 0.0f) ||
          (s1 * s.IsLeft(coord_t(maxx_, miny_)) <= 0.0f));
}

template <class coord_t> bool AABB2<coord_t>::Intersects(const coord_t& c, float r) const {
  // Trivial case - center of circle is within the bounding box
  if (Contains(c)) {
    return true;
  }

  // Trivial rejection - if the center is more than radius away from
  // from any box edge (in the direction perpendicular to the edge
  // and away from the box center) it cannot intersect
  if (c.first < minx_ - r || c.second < miny_ - r || c.first > maxx_ + r || c.second > maxy_ + r) {
    return false;
  }

  // If closest point on the box to the center is within radius
  // of the center then we intersected. We just project the center
  // onto each edge and check the distance
  r *= r;
  y_t horizontal = clamp(c.second, miny_, maxy_);
  x_t vertical = clamp(c.first, minx_, maxx_);
  return c.DistanceSquared(coord_t{minx_, horizontal}) <= r || // intersects the left side
         c.DistanceSquared(coord_t{maxx_, horizontal}) <= r || // intersects the right side
         c.DistanceSquared(coord_t{vertical, miny_}) <= r ||   // intersects the bottom side
         c.DistanceSquared(coord_t{vertical, maxy_}) <= r;     // intersects the top side
}

// Intersects the segment formed by u,v with the bounding box
template <class coord_t> bool AABB2<coord_t>::Intersect(coord_t& u, coord_t& v) const {
  // which do we need to move
  bool need_u = u.first < minx_ || u.first > maxx_ || u.second < miny_ || u.second > maxy_;
  bool need_v = v.first < minx_ || v.first > maxx_ || v.second < miny_ || v.second > maxy_;
  if (!(need_u || need_v)) {
    return true;
  }
  // find intercepts with each box edge
  std::list<coord_t> intersections;
  x_t x;
  y_t y;
  // intersect with each edge keeping it if its on this box and on the segment uv
  if (!std::isnan(x = y_intercept(u, v, miny_)) && x >= minx_ && x <= maxx_ &&
      between(x, u.first, v.first)) {
    intersections.emplace_back(x, miny_);
  }
  if (!std::isnan(x = y_intercept(u, v, maxy_)) && x >= minx_ && x <= maxx_ &&
      between(x, u.first, v.first)) {
    intersections.emplace_back(x, maxy_);
  }
  if (!std::isnan(y = x_intercept(u, v, maxx_)) && y >= miny_ && y <= maxy_ &&
      between(y, u.second, v.second)) {
    intersections.emplace_back(maxx_, y);
  }
  if (!std::isnan(y = x_intercept(u, v, minx_)) && y >= miny_ && y <= maxy_ &&
      between(y, u.second, v.second)) {
    intersections.emplace_back(minx_, y);
  }
  // pick the best one for each that needs it
  float u_dist = std::numeric_limits<float>::infinity(),
        v_dist = std::numeric_limits<float>::infinity(), d;
  for (const auto& intersection : intersections) {
    if (need_u && (d = u.DistanceSquared(intersection)) < u_dist) {
      u = intersection;
      u_dist = d;
    }
    if (need_v && (d = v.DistanceSquared(intersection)) < v_dist) {
      v = intersection;
      v_dist = d;
    }
  }
  // are we inside now?
  return intersections.size();
}

// Clips the input set of vertices to the specified boundary.  Uses a
// method where the shape is clipped against each edge in succession.
template <class coord_t>
uint32_t AABB2<coord_t>::Clip(std::vector<coord_t>& pts, const bool closed) const {
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
uint32_t AABB2<coord_t>::ClipAgainstEdge(const ClipEdge bdry,
                                         const bool closed,
                                         const std::vector<coord_t>& vin,
                                         std::vector<coord_t>& vout) const {
  // Clear the output vector
  vout.clear();

  // Special case for the 1st vertex. For polygons (closed) connect
  // last vertex to first vertex. For polylines repeat the first vertex
  uint32_t n = vin.size();
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
coord_t AABB2<coord_t>::ClipIntersection(const ClipEdge bdry,
                                         const coord_t& insidept,
                                         const coord_t& outsidept) const {
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

// Explicit instantiation
template class AABB2<Point2>;
template class AABB2<PointLL>;

} // namespace midgard
} // namespace valhalla
