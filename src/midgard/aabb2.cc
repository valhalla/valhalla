#include "valhalla/midgard/aabb2.h"
#include "valhalla/midgard/linesegment2.h"

namespace valhalla {
namespace midgard {

// Default constructor
template <class coord_t>
AABB2<coord_t>::AABB2()
    : minx_(0.0f),
      miny_(0.0f),
      maxx_(0.0f),
      maxy_(0.0f) {
}

// Construct an AABB given a minimum and maximum point.
template <class coord_t>
AABB2<coord_t>::AABB2(const coord_t& minpt, const coord_t& maxpt) {
  minx_ = minpt.x();
  miny_ = minpt.y();
  maxx_ = maxpt.x();
  maxy_ = maxpt.y();
}

// Constructor with specified bounds.
template <class coord_t>
AABB2<coord_t>::AABB2(const float minx, const float miny,
                      const float maxx, const float maxy) {
  minx_ = minx;
  miny_ = miny;
  maxx_ = maxx;
  maxy_ = maxy;
}

// Construct an AABB given a list of points.
template <class coord_t>
AABB2<coord_t>::AABB2(const std::vector<coord_t>& pts) {
  Create(pts);
}

// Equality operator.
template <class coord_t>
bool AABB2<coord_t>::operator ==(const AABB2<coord_t>& r2) const {
  return (minx_ == r2.minx() && maxx_ == r2.maxx() &&
          miny_ == r2.miny() && maxy_ == r2.maxy());
}

// Get the minimum x.
template <class coord_t>
float AABB2<coord_t>::minx() const {
  return minx_;
}

// Get the maximum x.
template <class coord_t>
float AABB2<coord_t>::maxx() const {
  return maxx_;
}

// Get the minimum y.
template <class coord_t>
float AABB2<coord_t>::miny() const {
  return miny_;
}

// Get the maximum y.
template <class coord_t>
float AABB2<coord_t>::maxy() const {
  return maxy_;
}

// Get the point at the minimum x,y.
template <class coord_t>
coord_t AABB2<coord_t>::minpt() const {
  return coord_t(minx_, miny_);
}

// Get the point at the maximum x,y.
template <class coord_t>
coord_t AABB2<coord_t>::maxpt() const {
  return coord_t(maxx_, maxy_);
}

// Creates an AABB given a list of points.
template <class coord_t>
void AABB2<coord_t>::Create(const std::vector<coord_t>& pts) {
  auto p = pts.begin();
  minx_ = p->x();
  maxx_ = minx_;
  miny_ = p->y();
  maxy_ = miny_;
  p++;
  for ( ; p < pts.end(); p++) {
    float x = p->x();
    if (x < minx_)
      minx_ = x;
    else if (x > maxx_)
      maxx_ = x;
    float y = p->y();
    if (y < miny_)
      miny_ = y;
    else if (y > maxy_)
      maxy_ = y;
  }
}

// Gets the center of the bounding box.
template <class coord_t>
coord_t AABB2<coord_t>::Center() const {
  return coord_t((minx_ + maxx_) * 0.5f, (miny_ + maxy_) * 0.5f);
}

// Tests if a specified point is within the bounding box.
template <class coord_t>
bool AABB2<coord_t>::Contains(const coord_t& pt) const {
  return (pt.x() >= minx_ && pt.y() >= miny_ &&
          pt.x() <  maxx_ && pt.y() <  maxy_);
}

// Checks to determine if another bounding box is completely inside
// this bounding box.
template <class coord_t>
bool AABB2<coord_t>::Contains(const AABB2<coord_t>& r2) const {
  return (Contains(r2.minpt()) && Contains(r2.maxpt()));
}

// Test if this bounding box intersects another bounding box.
template <class coord_t>
bool AABB2<coord_t>::Intersects(const AABB2<coord_t>& r2) const {
  // The bounding boxes do NOT intersect if the other bounding box (r2) is
  // entirely LEFT, BELOW, RIGHT, or ABOVE this bounding box.
  if ((r2.minx() < minx_ && r2.maxx() < minx_) ||
      (r2.miny() < miny_ && r2.maxy() < miny_) ||
      (r2.minx() > maxx_ && r2.maxx() > maxx_) ||
      (r2.miny() > maxy_ && r2.maxy() > maxy_))
    return false;

  return true;
}

// Tests whether the segment intersects the bounding box.
template <class coord_t>
bool AABB2<coord_t>::Intersect(const LineSegment2<coord_t>& seg) const {
  return Intersect(seg.a(), seg.b());
}

// Tests whether the segment intersects the bounding box.
template <class coord_t>
bool AABB2<coord_t>::Intersect(const coord_t& a, const coord_t& b) const {
  // Trivial case - either point within the bounding box
  if (Contains(a) || Contains(b))
    return true;

  // Trivial rejection - both points outside any one bounding edge
  if ((a.x() < minx_ && b.x() < minx_) ||    // Both left
      (a.y() < miny_ && b.y() < miny_) ||    // Both below
      (a.x() > maxx_ && b.x() > maxx_) ||    // Both right
      (a.y() > maxy_ && b.y() > maxy_))      // Both above
    return false;

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

// Gets the width of the bounding box.
template <class coord_t>
float AABB2<coord_t>::Width() const {
  return maxx_ - minx_;
}

// Gets the height of the bounding box.
template <class coord_t>
float AABB2<coord_t>::Height() const {
  return maxy_ - miny_;
}

// Expands (if necessary) the bounding box to include the specified
// bounding box.
template <class coord_t>
void AABB2<coord_t>::Expand(const AABB2<coord_t>& r2) {
  if (r2.minx() < minx_)
    minx_ = r2.minx();
  if (r2.miny() < miny_)
    miny_ = r2.miny();
  if (r2.maxx() > maxx_)
    maxx_ = r2.maxx();
  if (r2.maxy() > maxy_)
    maxy_ = r2.maxy();
}

// Explicit instantiation
template class AABB2<Point2>;
template class AABB2<PointLL>;

}
}
