#ifndef __polyline2_h_
#define __polyline2_h_

#include "point2.h"
#include "aabb2.h"
#include "clipper2.h"
#include "linesegment2.h"

namespace valhalla{
namespace geo{

/**
 * 2-D polyline
 */
class Polyline2 {
 public:
  Polyline2() { }

  /**
   * Constructor given a list of points.
   * @param  pts  List of points.
   */
  Polyline2(std::vector<Point2>& pts) {
    pts_ = pts;
  }

  // TODO - do we need copy constructor and = operator?

  /**
   * Gets the list of points.
   * @return  Returns the list of points.
   */
  std::vector<Point2>& pts() {
    return pts_;
  }

  /**
   * Add a point to the polyline. Checks to see if the input point is
   * equal to the current endpoint of the polyline and does not add the
   * point if it is equal.
   */
  void Add(const Point2& p) {
    unsigned int n = pts_.size();
    if (n == 0 || !(p == pts_[n-1]))
      pts_.push_back(p);
  }

  /**
  * Finds the length of the polyline by accumulating the length of all
  * segments.
  * @return    Returns the length of the polyline.
  */
  float Length() const {
    float length = 0;
    for (unsigned int i = 0, n = pts_.size(); i < n-1; i++) {
      length += pts_[i].Distance(pts_[i+1]);
    }
    return length;
  }

  /**
   * Gets the closest point to this polyline from the specified point.
   * @param  pt      Point to find closest point to.
   * @param  closest (OUT) Closest point along the polyline
   * @param  idx     (OUT) Index of the segment of the polyline which contains
   *                       the closest point.
   * @return   Returns the distance squared of the closest point.
   */
  float ClosestPoint(const Point2& pt, Point2& nearest, int& idx) const {
    return pt.ClosestPoint(pts_, nearest, idx);
  }

  /**
   * Generalize this polyline.
   */
  unsigned int Generalize(const float t) {
    // Create a vector for the output shape. Recursively call Douglass-Peucker
    // method to generalize the polyline. Square the error tolerance to avoid
    // sqrts.
    std::vector<Point2> genpts;
    DouglasPeucker(0, pts_.size() - 1, t*t, genpts);
    pts_= genpts;
    return pts_.size();
  }

  /**
   * Get a generalized polyline from this polyline. This polyline remains
   * unchanged.
   * @param  t       Generalization tolerance.
   * @param  genpts  Points within a generalized polyline he generalized polyline.
   */
  Polyline2 GeneralizedPolyline(const float t) {
    // Recursively call Douglass-Peucker method to generalize the polyline.
    // Square the error tolerance to avoid sqrts.
    std::vector<Point2> genpts;
    DouglasPeucker(0, pts_.size() - 1, t * t, genpts);
    return Polyline2(genpts);
  }

  /**
   * Clip this polyline to the specified bounding box.
   * @param box  Bounding box to clip this polyline to.
   */
  unsigned int Clip(const AABB2& box) {
    Clipper2 clipper;
    return clipper.Clip(box, pts_, false);
  }

  /**
   * Gets a polyline clipped to the supplied bounding box. This polyline
   * remains unchanged.
   */
   Polyline2 ClippedPolyline(const AABB2& box) {
    // Copy the polyline points to a temporary vector
    Clipper2 clipper;
    std::vector<Point2> pts = pts_;
    clipper.Clip(box, pts, false);
    return Polyline2(pts);
  }

  // TODO - add Google polyline encoding

protected:
  // Polyline points
  std::vector<Point2> pts_;

  void DouglasPeucker(const unsigned int i, const unsigned int j,
                      const float t2, std::vector<Point2>& genpts) {
    // Find the vertex farthest from the line segment ViVj
    unsigned int index = 0;
    float maxdist = 0.0f;
    float d2;
    Point2 tmp;
    LineSegment2 v(pts_[i], pts_[j]);
    for (unsigned int k = i+1; k < j; k++) {
      // Get the squared distance from the intermediate point to the segment
      d2 = v.DistanceSquared(pts_[k], tmp);
      if (d2 > maxdist) {
        maxdist = d2;
        index   = k;
      }
    }

    // If the maximum distance is greater than the error tolerance,
    // divide the sub-polyline into two at the furthest point and
    // recursively call this method
    if (maxdist > t2) {
      DouglasPeucker(i, index, t2, genpts);
      DouglasPeucker(index, j, t2, genpts);
    }
    else {
      // Output segment ViVj to the generalized shape
      unsigned int n = genpts.size();
      if (n == 0 || !(pts_[i] == genpts[n-1])) {
        genpts.push_back(pts_[i]);
        n++;
      }
      if (!(pts_[j] == genpts[n-1])) {
        genpts.push_back(pts_[j]);
      }
    }
  }
};

}
}

#endif
