#include "midgard/aabb2.h"

namespace valhalla{
namespace midgard{
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

  AABB2::AABB2(const float minx, const float miny,
        const float maxx, const float maxy) {
    minx_ = minx;
    miny_ = miny;
    maxx_ = maxx;
    maxy_ = maxy;
  }

  AABB2::AABB2(std::vector<Point2>& pts) {
    Create(pts);
  }

  bool AABB2::operator ==(const AABB2& r2) const {
    return (minx_ == r2.minx() && maxx_ == r2.maxx() && miny_ == r2.miny()
        && maxy_ == r2.maxy());
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
    return Point2((minx_ + maxx_) * 0.5, (miny_ + maxy_) * 0.5);
  }

  bool AABB2::Contains(const Point2& pt) const {
    return (pt.x() >= minx_ && pt.y() >= miny_ && pt.x() < maxx_
        && pt.y() < maxy_);
  }

  bool AABB2::Contains(const AABB2& r2) const {
    return (Contains(r2.minpt()) && Contains(r2.maxpt()));
  }

  bool AABB2::Intersects(const AABB2& r2) const {
    // The bounding boxes do NOT intersect if the other bounding box (r2) is
    // entirely LEFT, BELOW, RIGHT, or ABOVE this bounding box.
    if ((r2.minx() < minx_ && r2.maxx() < minx_)
        || (r2.miny() < miny_ && r2.maxy() < miny_)
        || (r2.minx() > maxx_ && r2.maxx() > maxx_)
        || (r2.miny() > maxy_ && r2.maxy() > maxy_))
      return false;

    return true;
  }

  bool AABB2::Intersect(const Point2& a, const Point2& b) const {
    // Trivial case - either point within with the bounding box
    if (Contains(a) || Contains(b))
      return true;

    // Trivial rejection
    if ((a.x() < minx_ && b.x() < minx_) ||    // Both left
        (a.y() < miny_ && b.y() < miny_) ||    // Both below
        (a.x() > maxx_ && b.x() > maxx_) ||    // Both right
        (a.y() > maxy_ && b.y() > maxy_))      // Both above
      return false;

    // Vertical - return true if y coordinates are on opposite sides of
    // either the top or bottom edge. Trivial reject case ensures y is
    // within the bounding box
    if (b.x() == a.x()) {
      if (((a.y() < miny_) != (b.y() < miny_)) ||
          ((a.y() > maxy_) != (b.y() > maxy_)))
        return true;
    }

    // Horizontal - return true if x coordinates are on opposite sides of
    // either the left or right edge. Trivial reject case ensures x is
    // within the bounding box
    if (b.y() == a.y()) {
      if (((a.x() < minx_) != (b.x() < minx_)) ||
          ((a.x() > maxx_) != (b.x() > maxx_)))
        return true;
    }

    // Find slope and intercept for the line equation and check which
    // edge(s) the segment crosses. Note that it is possible to cross
    // multiple edges. If any one crossing is within the bounding box range
    // we return true.
    float m = (b.y() - a.y()) / (b.x() - a.x());
    float s =  b.y() - (m * b.x());
    if ((a.x() < minx_) != (b.x() < minx_)) {   // Crosses left edge
      float y = m * minx_ + s;
      if (miny_ <= y && y <= maxy_)
        return true;
    }
    if ((a.x() > maxx_) != (b.x() > maxx_))  {  // Crosses right edge
      float y = m * maxx_ + s;
      if (miny_ <= y && y <= maxy_)
        return true;
    }
    if ((a.y() < miny_) != (b.y() < miny_))  {  // Crosses bottom edge
      float x = (miny_ - s) / m;
      if (minx_ <= x && x <= maxx_)
        return true;
    }
    if ((a.y() > maxy_) != (b.y() > maxy_))  {  // Crosses top edge
      float x = (maxy_ - s) / m;
      if (minx_ <= x && x <= maxx_)
        return true;
    }
    return false;
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
