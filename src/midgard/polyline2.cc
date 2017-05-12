#include "valhalla/midgard/polyline2.h"

#include <vector>
#include <list>
#include <functional>

namespace valhalla {
namespace midgard {

template <class coord_t>
Polyline2<coord_t>::Polyline2() { }

// Constructor given a list of points.
template <class coord_t>
Polyline2<coord_t>::Polyline2(std::vector<coord_t>& pts) {
  pts_ = pts;
}

// Get the list of points.
template <class coord_t>
std::vector<coord_t>& Polyline2<coord_t>::pts() {
  return pts_;
}

// Add a point to the polyline. Checks to see if the input point is
// equal to the current endpoint of the polyline and does not add the
// point if it is equal.
template <class coord_t>
void Polyline2<coord_t>::Add(const coord_t& p) {
  uint32_t n = pts_.size();
  if (n == 0 || !(p == pts_[n-1]))
    pts_.push_back(p);
}

// Finds the length of the polyline by accumulating the length of all
// segments.
template <class coord_t>
float Polyline2<coord_t>::Length() const {
  float length = 0;
  if (pts_.size() < 2)
    return length;
  for (auto p = std::next(pts_.cbegin()); p != pts_.cend(); ++p)
    length += std::prev(p)->Distance(*p);
  return length;
}

// Find the length of the supplied polyline.
template <class coord_t>
template <class container_t>
float Polyline2<coord_t>::Length(const container_t& pts) {
  float length = 0;
  if (pts.size() < 2)
    return length;
  for (auto p = std::next(pts.cbegin()); p != pts.cend(); ++p)
    length += std::prev(p)->Distance(*p);
  return length;
}

// Finds the closest point to the supplied polyline as well as the distance
// to that point and the index of the segment where the closest point lies.
template <class coord_t>
std::tuple<coord_t, float, int> Polyline2<coord_t>::ClosestPoint(const coord_t& pt) const {
  return pt.ClosestPoint(pts_);
}

// Generalize this polyline.
template <class coord_t>
uint32_t Polyline2<coord_t>::Generalize(const float t) {
  // Create a vector for the output shape. Recursively call Douglass-Peucker
  // method to generalize the polyline. Square the error tolerance to avoid
  // sqrts.
  Generalize(pts_, t);
  return pts_.size();
}

// Get a generalized polyline from this polyline. This polyline remains
// unchanged.
template <class coord_t>
Polyline2<coord_t> Polyline2<coord_t>::GeneralizedPolyline(const float t) {
  // Recursively call Douglass-Peucker method to generalize the polyline.
  // Square the error tolerance to avoid sqrts.
  Polyline2 generalized(pts_);
  generalized.Generalize(t);
  return generalized;
}

template <class coord_t>
template <class container_t>
void Polyline2<coord_t>::Generalize(container_t& polyline, float epsilon) {

  //the recursive bit
  epsilon *= epsilon;
  std::function<void (typename container_t::iterator, typename container_t::iterator)> peucker;
  peucker = [&peucker, &polyline, epsilon](typename container_t::iterator start, typename container_t::iterator end) {
    //find the point furthest from the line
    float dmax = 0.f;
    typename container_t::iterator itr;
    LineSegment2<coord_t> l{*start, *end};
    coord_t tmp;
    for(auto i = std::next(start); i != end; ++i) {
      auto d = l.DistanceSquared(*i, tmp);
      if(d > dmax) {
        itr = i;
        dmax = d;
      }
    }

    //there are some high frequency details between start and end
    //so we need to look for flatter sections between them
    if(dmax >= epsilon) {
        peucker(start, itr);
        peucker(itr, end);
    }//nothing sticks out between start and end so simplify it away
    else
      polyline.erase(std::next(start), end);
  };

  //recurse!
  peucker(polyline.begin(), std::prev(polyline.end()));
}

// Clip this polyline to the specified bounding box.
template <class coord_t>
uint32_t Polyline2<coord_t>::Clip(const AABB2<coord_t>& box) {
  return box.Clip(pts_, false);
}

// Gets a polyline clipped to the supplied bounding box. This polyline
// remains unchanged.
template <class coord_t>
Polyline2<coord_t> Polyline2<coord_t>::ClippedPolyline(const AABB2<coord_t>& box) {
  // Copy the polyline points to a temporary vector
  std::vector<coord_t> pts = pts_;
  box.Clip(pts, false);
  return Polyline2(pts);
}

// Explicit instantiation
template class Polyline2<Point2>;
template class Polyline2<PointLL>;

template float Polyline2<PointLL>::Length(const std::vector<PointLL>&);
template float Polyline2<Point2>::Length(const std::vector<Point2>&);
template float Polyline2<PointLL>::Length(const std::list<PointLL>&);
template float Polyline2<Point2>::Length(const std::list<Point2>&);

template void Polyline2<PointLL>::Generalize(std::vector<PointLL>&, float);
template void Polyline2<Point2>::Generalize(std::vector<Point2>&, float);
template void Polyline2<PointLL>::Generalize(std::list<PointLL>&, float);
template void Polyline2<Point2>::Generalize(std::list<Point2>&, float);

}
}
