#include "valhalla/midgard/polyline2.h"

namespace valhalla{
namespace midgard{

Polyline2::Polyline2() { }

// Constructor given a list of points.
Polyline2::Polyline2(std::vector<Point2>& pts) {
  pts_ = pts;
}

// Get the list of points.
std::vector<Point2>& Polyline2::pts() {
  return pts_;
}

// Add a point to the polyline. Checks to see if the input point is
// equal to the current endpoint of the polyline and does not add the
// point if it is equal.
void Polyline2::Add(const Point2& p) {
  uint32_t n = pts_.size();
  if (n == 0 || !(p == pts_[n-1]))
    pts_.push_back(p);
}

// Finds the length of the polyline by accumulating the length of all
// segments.
float Polyline2::Length() const {
  float length = 0;
  for (uint32_t i = 0; i < pts_.size() - 1; i++) {
    length += pts_[i].Distance(pts_[i+1]);
  }
  return length;
}

// Find the length of the supplied polyline.
float Polyline2::Length(const std::vector<Point2>& pts) const {
  float length = 0;
  for (uint32_t i = 0; i < pts_.size() - 1; i++) {
    length += pts[i].Distance(pts[i+1]);
  }
  return length;
}

// Finds the closest point to the supplied polyline as well as the distance
// squared to that point and the index of the segment where the closest point lies.
std::tuple<Point2, float, int> Polyline2::ClosestPoint(const Point2& pt) const {
  return pt.ClosestPoint(pts_);
}

// Generalize this polyline.
uint32_t Polyline2::Generalize(const float t) {
  // Create a vector for the output shape. Recursively call Douglass-Peucker
  // method to generalize the polyline. Square the error tolerance to avoid
  // sqrts.
  std::vector<Point2> genpts;
  DouglasPeucker(0, pts_.size() - 1, t*t, genpts);
  pts_= genpts;
  return pts_.size();
}

// Get a generalized polyline from this polyline. This polyline remains
// unchanged.
Polyline2 Polyline2::GeneralizedPolyline(const float t) {
  // Recursively call Douglass-Peucker method to generalize the polyline.
  // Square the error tolerance to avoid sqrts.
  std::vector<Point2> genpts;
  DouglasPeucker(0, pts_.size() - 1, t * t, genpts);
  return Polyline2(genpts);
}

// Clip this polyline to the specified bounding box.
uint32_t Polyline2::Clip(const AABB2& box) {
  Clipper2 clipper;
  return clipper.Clip(box, pts_, false);
}

// Gets a polyline clipped to the supplied bounding box. This polyline
// remains unchanged.
Polyline2 Polyline2::ClippedPolyline(const AABB2& box) {
  // Copy the polyline points to a temporary vector
  Clipper2 clipper;
  std::vector<Point2> pts = pts_;
  clipper.Clip(box, pts, false);
  return Polyline2(pts);
}

void Polyline2::DouglasPeucker(const uint32_t i, const uint32_t j,
                    const float t2, std::vector<Point2>& genpts) {
  // Find the vertex farthest from the line segment ViVj
  uint32_t index = 0;
  float maxdist = 0.0f;
  float d2;
  Point2 tmp;
  LineSegment2 v(pts_[i], pts_[j]);
  for (uint32_t k = i+1; k < j; k++) {
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
    uint32_t n = genpts.size();
    if (n == 0 || !(pts_[i] == genpts[n-1])) {
      genpts.push_back(pts_[i]);
      n++;
    }
    if (!(pts_[j] == genpts[n-1])) {
      genpts.push_back(pts_[j]);
    }
  }
}

}
}
