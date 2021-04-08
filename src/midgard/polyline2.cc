#include "midgard/polyline2.h"
#include "midgard/point2.h"
#include "midgard/pointll.h"

#include <list>

namespace valhalla {
namespace midgard {

/**
 * Finds the length of the polyline by accumulating the length of all
 * segments.
 * @return    Returns the length of the polyline.
 */
template <typename coord_t> typename coord_t::value_type Polyline2<coord_t>::Length() const {
  typename coord_t::value_type length = 0;
  if (pts_.size() < 2) {
    return length;
  }
  for (auto p = std::next(pts_.cbegin()); p != pts_.cend(); ++p) {
    length += std::prev(p)->Distance(*p);
  }
  return length;
}

/**
 * Compute the length of the specified polyline.
 * @param   pts  Polyline vertices.
 * @return  Returns the length of the polyline.
 */
template <typename coord_t>
template <class container_t>
typename coord_t::value_type Polyline2<coord_t>::Length(const container_t& pts) {
  typename coord_t::value_type length = 0;
  if (pts.size() < 2) {
    return length;
  }
  for (auto p = std::next(pts.cbegin()); p != pts.cend(); ++p) {
    length += std::prev(p)->Distance(*p);
  }
  return length;
}

/**
 * Generalize the given list of points
 *
 * @param polyline    the list of points
 * @param epsilon     the tolerance used in removing points
 * @param  indices    list of indices of points not to generalize
 */
template <typename coord_t>
template <class container_t>
void Polyline2<coord_t>::Generalize(container_t& polyline,
                                    typename coord_t::value_type epsilon,
                                    const std::unordered_set<size_t>& indices) {
  // any epsilon this low will have no effect on the input nor will any super short input
  if (epsilon <= 0 || polyline.size() < 3)
    return;

  // the recursive bit
  epsilon *= epsilon;
  std::function<void(typename container_t::iterator, size_t, typename container_t::iterator, size_t)>
      peucker;
  peucker = [&peucker, &polyline, epsilon, &indices](typename container_t::iterator start, size_t s,
                                                     typename container_t::iterator end, size_t e) {
    // find the point furthest from the line
    typename coord_t::value_type dmax = std::numeric_limits<typename coord_t::value_type>::lowest();
    typename container_t::iterator itr;
    LineSegment2<coord_t> l{*start, *end};
    size_t j = e - 1, k;
    coord_t tmp;
    for (auto i = std::prev(end); i != start; --i, --j) {
      // special points we dont want to generalize no matter what take precidence
      if (indices.find(j) != indices.end()) {
        itr = i;
        dmax = epsilon;
        k = j;
        break;
      }

      // if this is the highest frequency detail so far
      auto d = l.DistanceSquared(*i, tmp);
      if (d > dmax) {
        itr = i;
        dmax = d;
        k = j;
      }
    }

    // there are some high frequency details between start and end
    // so we need to look for flatter sections between them
    if (dmax >= epsilon) {
      // we recurse from right to left for two reasons:
      // 1. we want to preserve iterator validity in the vector version
      // 2. its the only way to preserve the indices in the keep set
      if (e - k > 1)
        peucker(itr, k, end, e);
      if (k - s > 1)
        peucker(start, s, itr, k);
    } // nothing sticks out between start and end so simplify everything between away
    else
      polyline.erase(std::next(start), end);
  };

  // recurse!
  peucker(polyline.begin(), 0, std::prev(polyline.end()), polyline.size() - 1);
}

// Explicit instantiation
template class Polyline2<PointXY<float>>;
template class Polyline2<PointXY<double>>;
template class Polyline2<GeoPoint<float>>;
template class Polyline2<GeoPoint<double>>;

template float Polyline2<PointXY<float>>::Length(const std::vector<PointXY<float>>&);
template double Polyline2<PointXY<double>>::Length(const std::vector<PointXY<double>>&);
template float Polyline2<PointXY<float>>::Length(const std::list<PointXY<float>>&);
template double Polyline2<PointXY<double>>::Length(const std::list<PointXY<double>>&);
template float Polyline2<GeoPoint<float>>::Length(const std::vector<GeoPoint<float>>&);
template double Polyline2<GeoPoint<double>>::Length(const std::vector<GeoPoint<double>>&);
template float Polyline2<GeoPoint<float>>::Length(const std::list<GeoPoint<float>>&);
template double Polyline2<GeoPoint<double>>::Length(const std::list<GeoPoint<double>>&);

template void Polyline2<PointXY<float>>::Generalize(std::vector<PointXY<float>>&,
                                                    float,
                                                    const std::unordered_set<size_t>&);
template void Polyline2<PointXY<double>>::Generalize(std::vector<PointXY<double>>&,
                                                     double,
                                                     const std::unordered_set<size_t>&);
template void Polyline2<PointXY<float>>::Generalize(std::list<PointXY<float>>&,
                                                    float,
                                                    const std::unordered_set<size_t>&);
template void Polyline2<PointXY<double>>::Generalize(std::list<PointXY<double>>&,
                                                     double,
                                                     const std::unordered_set<size_t>&);
template void Polyline2<GeoPoint<float>>::Generalize(std::vector<GeoPoint<float>>&,
                                                     float,
                                                     const std::unordered_set<size_t>&);
template void Polyline2<GeoPoint<double>>::Generalize(std::vector<GeoPoint<double>>&,
                                                      double,
                                                      const std::unordered_set<size_t>&);
template void Polyline2<GeoPoint<float>>::Generalize(std::list<GeoPoint<float>>&,
                                                     float,
                                                     const std::unordered_set<size_t>&);
template void Polyline2<GeoPoint<double>>::Generalize(std::list<GeoPoint<double>>&,
                                                      double,
                                                      const std::unordered_set<size_t>&);

} // namespace midgard
} // namespace valhalla
