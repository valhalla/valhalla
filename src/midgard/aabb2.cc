#include "valhalla/midgard/aabb2.h"
#include "valhalla/midgard/linesegment2.h"

namespace valhalla {
namespace midgard {

AABB2::AABB2()
    : minx_(0.0f),
      miny_(0.0f),
      maxx_(0.0f),
      maxy_(0.0f) {
}

AABB2::AABB2(const Point2& minpt, const Point2& maxpt) {
  minx_ = minpt.x();
  miny_ = minpt.y();
  maxx_ = maxpt.x();
  maxy_ = maxpt.y();
}

AABB2::AABB2(const float minx, const float miny, const float maxx,
             const float maxy) {
  minx_ = minx;
  miny_ = miny;
  maxx_ = maxx;
  maxy_ = maxy;
}

AABB2::AABB2(std::vector<Point2>& pts) {
  Create(pts);
}

bool AABB2::operator ==(const AABB2& r2) const {
  return (minx_ == r2.minx() && maxx_ == r2.maxx() &&
          miny_ == r2.miny() && maxy_ == r2.maxy());
}

float AABB2::minx() const {
  return minx_;
}

float AABB2::maxx() const {
  return maxx_;
}

float AABB2::miny() const {
  return miny_;
}

float AABB2::maxy() const {
  return maxy_;
}

Point2 AABB2::minpt() const {
  return Point2(minx_, miny_);
}

Point2 AABB2::maxpt() const {
  return Point2(maxx_, maxy_);
}

void AABB2::Create(std::vector<Point2>& pts) {
  float x, y;
  const Point2* p = &pts[0];
  minx_ = p->x();
  maxx_ = minx_;
  miny_ = p->y();
  maxy_ = miny_;
  p++;
  for (unsigned int i = 1, n = pts.size(); i < n; i++, p++) {
    x = p->x();
    if (x < minx_)
      minx_ = x;
    else if (x > maxx_)
      maxx_ = x;
    y = p->y();
    if (y < miny_)
      miny_ = y;
    else if (y > maxy_)
      maxy_ = y;
  }
}

Point2 AABB2::Center() const {
  return Point2((minx_ + maxx_) * 0.5f, (miny_ + maxy_) * 0.5f);
}

bool AABB2::Contains(const Point2& pt) const {
  return (pt.x() >= minx_ && pt.y() >= miny_ &&
          pt.x() <  maxx_ && pt.y() <  maxy_);
}

bool AABB2::Contains(const AABB2& r2) const {
  return (Contains(r2.minpt()) && Contains(r2.maxpt()));
}

bool AABB2::Intersects(const AABB2& r2) const {
  // The bounding boxes do NOT intersect if the other bounding box (r2) is
  // entirely LEFT, BELOW, RIGHT, or ABOVE this bounding box.
  if ((r2.minx() < minx_ && r2.maxx() < minx_) ||
      (r2.miny() < miny_ && r2.maxy() < miny_) ||
      (r2.minx() > maxx_ && r2.maxx() > maxx_) ||
      (r2.miny() > maxy_ && r2.maxy() > maxy_))
    return false;

  return true;
}

bool AABB2::Intersect(const Point2& a, const Point2& b) const {
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
  LineSegment2 s(a, b);
  float s1 = s.IsLeft(Point2(minx_, miny_));
  return ((s1 * s.IsLeft(Point2(minx_, maxy_)) <= 0.0f) ||
          (s1 * s.IsLeft(Point2(maxx_, maxy_)) <= 0.0f) ||
          (s1 * s.IsLeft(Point2(maxx_, miny_)) <= 0.0f));
}

float AABB2::Width() const {
  return maxx_ - minx_;
}

float AABB2::Height() const {
  return maxy_ - miny_;
}

float AABB2::Area() const {
  return Width() * Height();
}

void AABB2::Expand(const AABB2& r2) {
  if (r2.minx() < minx_)
    minx_ = r2.minx();
  if (r2.miny() < miny_)
    miny_ = r2.miny();
  if (r2.maxx() > maxx_)
    maxx_ = r2.maxx();
  if (r2.maxy() > maxy_)
    maxy_ = r2.maxy();
}

}
}
