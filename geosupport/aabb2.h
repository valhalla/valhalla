#ifndef __aabb2__
#define __aabb2__

#include <vector>

/**
 * Axis Aligned Bounding Box (2 dimensional)
 * @author  David W. Nesbitt
 */
class AABB2 {
 protected:
  float minx_;
  float miny_;
  float maxx_;
  float maxy_;

 public:
  /**
   * Default constructor.
   */
  AABB2() {
    minx_ = 0.0f;
    miny_ = 0.0f;
    maxx_ = 0.0f;
    maxy_ = 0.0f;
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
  bool operator == (const AABB2& r2) {
    return (minx_ == r2.minx() && maxx_ == r2.maxx() &&
            miny_ == r2.miny() && maxy_ == r2.maxy());
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
  Point2 get_minpt() const {
    return Point2(minx_, miny_);
  }

  /**
   * Get the point at the maximum x,y,z.
   * @return  Returns the max. point.
   */
  Point2 get_maxpt() const {
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
  Point2 GetCenter() const {
    return Point2((minx_ + maxx_) * 0.5, (miny_ + maxy_) * 0.5);
  }

  /**
   * Tests if a specified point is within the bounding box.
   * @param  pt   Point to test.
   * @return  true if within the bounding box and false if outside the extent.
   *          If the point lies along the minimum x,yx or y.
   */
  bool Within(const Point2& pt) const {
    return (pt.x() >= minx_ && pt.y() >= miny_ &&
            pt.x() <  maxx_ && pt.y() <  maxy_);
  }

  /**
   * Checks to determine if another bounding box is inside this bounding box.
   * @param   r2    Test bounding box
   * @return  Returns false if the bounding box is not entirely inside,
   *          true if it is inside.
   */
  bool Contains(const AABB2& r2) const {
    return (Within(r2.get_minpt()) && Within(r2.get_maxpt()));
  }

  /**
   * Test if this bounding box intersects another bounding box.
   * @param    r2   Other bounding box to test intersection against.
   * @return   Returns true if the bounding box intersect, false if not.
   */
  bool Intersects(const AABB2& r2) const {
    // The bounding boxes do NOT intersect if the other bounding box (r2) is
    // entirely LEFT, BELOW, RIGHT, or ABOVE this bounding box.
    if ((r2.minx() < minx_ && r2.maxx() < minx_) ||
        (r2.miny() < miny_ && r2.maxy() < miny_) ||
        (r2.minx() > maxx_ && r2.maxx() > maxx_) ||
        (r2.miny() > maxy_ && r2.maxy() > maxy_))
      return false;

    return true;
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
};

#endif
