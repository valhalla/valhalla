#ifndef __aabb2_h__
#define __aabb2_h__

#include <vector>

/**
 * Axis Aligned Bounding Box (2 dimensional)
 * @author  David W. Nesbitt
 */
class AABB2 {
 public:
  /**
   * Default constructor.
   */
  AABB2()
      : minx_(0.0f),
        miny_(0.0f),
        maxx_(0.0f),
        maxy_(0.0f) {
  }

  /**
   * Construct an AABB given a minimum and maximum point.
   * @param  minpt  Minimum point (x,y)
   * @param  maxpt  Maximum point (x,y)
   */
  AABB2(const Point2& minpt, const Point2& maxpt) {
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
  AABB2(const float minx, const float miny,
        const float maxx, const float maxy) {
    minx_ = miny;
    miny_ = minx;
    maxx_ = maxy;
    maxy_ = maxx;
  }

  /**
   * Construct an AABB given a list of points.
   * @param  pts  Vertex list.
   */
  AABB2(std::vector<Point2>& pts) {
    Create(pts);
  }

  /**
   * Equality operator.
   * @param   r2  Bounding box to compare to.
   * @return  Returns true if the 2 bounding boxes are equal.
   */
  bool operator ==(const AABB2& r2) {
    return (minx_ == r2.minx() && maxx_ == r2.maxx() && miny_ == r2.miny()
        && maxy_ == r2.maxy());
  }

  /**
   * Get the minimum x
   * @return  Returns minimum x.
   */
  float minx() const {
    return minx_;
  }

  /**
   * Get the maximum x
   * @return  Returns maximum x.
   */
  float maxx() const {
    return maxx_;
  }

  /**
   * Get the minimum y
   * @return  Returns minimum y.
   */
  float miny() const {
    return miny_;
  }

  /**
   * Get the maximum y
   * @return  Returns maximum y.
   */
  float maxy() const {
    return maxy_;
  }

  /**
   * Get the point at the minimum x,y,z.
   * @return  Returns the min. point.
   */
  Point2 minpt() const {
    return Point2(minx_, miny_);
  }

  /**
   * Get the point at the maximum x,y,z.
   * @return  Returns the max. point.
   */
  Point2 maxpt() const {
    return Point2(maxx_, maxy_);
  }

  /**
   * Creates an AABB given a list of points.
   * @param  pts  Vertex list.
   */
  void Create(std::vector<Point2>& pts) {
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

  /**
   * Gets the center of the bounding box.
   * @return  Returns the center point of the bounding box.
   */
  Point2 Center() const {
    return Point2((minx_ + maxx_) * 0.5, (miny_ + maxy_) * 0.5);
  }

  /**
   * Tests if a specified point is within the bounding box.
   * @param  pt   Point to test.
   * @return  true if within the bounding box and false if outside the extent.
   *          If the point lies along the minimum x,yx or y.
   */
  bool Contains(const Point2& pt) const {
    return (pt.x() >= minx_ && pt.y() >= miny_ && pt.x() < maxx_
        && pt.y() < maxy_);
  }

  /**
   * Checks to determine if another bounding box is inside this bounding box.
   * @param   r2    Test bounding box
   * @return  Returns false if the bounding box is not entirely inside,
   *          true if it is inside.
   */
  bool Contains(const AABB2& r2) const {
    return (Contains(r2.minpt()) && Contains(r2.maxpt()));
  }

  /**
   * Test if this bounding box intersects another bounding box.
   * @param    r2   Other bounding box to test intersection against.
   * @return   Returns true if the bounding box intersect, false if not.
   */
  bool Intersects(const AABB2& r2) const {
    // The bounding boxes do NOT intersect if the other bounding box (r2) is
    // entirely LEFT, BELOW, RIGHT, or ABOVE this bounding box.
    if ((r2.minx() < minx_ && r2.maxx() < minx_)
        || (r2.miny() < miny_ && r2.maxy() < miny_)
        || (r2.minx() > maxx_ && r2.maxx() > maxx_)
        || (r2.miny() > maxy_ && r2.maxy() > maxy_))
      return false;

    return true;
  }

  /**
   * Tests whether the segment intersects the bounding box.
   * @param   a  Endpoint of the segment
   * @param   b  Endpoint of the segment
   * @return  Returns true if the segment intersects (or lies completely
   *          within) the bounding box.
   */
  bool Intersect(const Point2& a, const Point2& b) const {
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

  /**
   * Gets the width of the bounding box
   * @return  Returns the width of this bounding box.
   */
  float Width() const {
    return maxx_ - minx_;
  }

  /**
   * Gets the height of the bounding box
   * @return  Returns the height of this bounding box.
   */
  float Height() const {
    return maxy_ - miny_;
  }

  /**
   * Gets the area of the bounding box.
   * @return  Returns the area of the bounding box.
   */
  float Area() const {
    return Width() * Height();
  }

  /**
   * Expands (if necessary) the bounding box.
   * @param  r2  Bounding bounding box to "combine" with this
   */
  void Expand(const AABB2& r2) {
    if (r2.minx() < minx_)
      minx_ = r2.minx();
    if (r2.miny() < miny_)
      miny_ = r2.miny();
    if (r2.maxx() > maxx_)
      maxx_ = r2.maxx();
    if (r2.maxy() > maxy_)
      maxy_ = r2.maxy();
  }

 protected:
  // Minumum and maximum x,y values (lower right and upper left corners
  // of a rectangle / bounding box.
  float minx_;
  float miny_;
  float maxx_;
  float maxy_;
};

#endif
