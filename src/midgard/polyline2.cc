#include "midgard/distanceapproximator.h"
#include "midgard/polyline2.h"
#include "midgard/point2.h"
#include "midgard/pointll.h"
#include "midgard/util.h"

#include <list>
#include <iostream>
#include <unistd.h>


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

struct TileId {
  unsigned int x, y;
  friend bool operator==(const TileId& L, const TileId& R) {
    return L.x == R.x && L.y == R.y;
  }
};

struct TileId_hash_functor {
  size_t operator()(const TileId& tid) const {
#if 0
    // In my tests this wasn't quite as fast as the simple impl below.
    size_t seed = 0;
    valhalla::midgard::hash_combine(seed, tid.x);
    valhalla::midgard::hash_combine(seed, tid.y);
    return seed;
#endif
    // I'll concede this could be an awful way to hash in mock
    // scenarios. However, for real world data, its darn simple/fast.
    return tid.x;
  }
};

// Use the barycentric technique to test if the point p
// is inside the triangle formed by (a, b, c).
bool triangle_contains(const PointLL& a,
                       const PointLL& b,
                       const PointLL& c,
                       const PointLL& p) {
  double v0x = c.x() - a.x();
  double v0y = c.y() - a.y();
  double v1x = b.x() - a.x();
  double v1y = b.y() - a.y();
  double v2x = p.x() - a.x();
  double v2y = p.y() - a.y();

  double dot00 = v0x*v0x + v0y*v0y;
  double dot01 = v0x*v1x + v0y*v1y;
  double dot02 = v0x*v2x + v0y*v2y;
  double dot11 = v1x*v1x + v1y*v1y;
  double dot12 = v1x*v2x + v1y*v2y;

  double denom = dot00*dot11 - dot01*dot01;

  // Triangle with very small area, e.g., nearly a line.
  if (std::fabs(denom) < 1e-24)
    return false;

  double u = (dot11 * dot02 - dot01 * dot12) / denom;
  double v = (dot00 * dot12 - dot01 * dot02) / denom;

  // Check if point is in triangle
  return (u >= 0) && (v >= 0) && (u + v < 1);
}

#if 0
bool triangle_contains_any_of(const PointLL& a,
                              const PointLL& b,
                              const PointLL& c,
                              const PointLL** ap,
                              size_t nump) {
  double v0x = c.x() - a.x();
  double v0y = c.y() - a.y();
  double v1x = b.x() - a.x();
  double v1y = b.y() - a.y();
  double dot00 = v0x * v0x + v0y * v0y;
  double dot01 = v0x * v1x + v0y * v1y;
  double dot11 = v1x * v1x + v1y * v1y;

  for (size_t i = 0; i < nump; i++) {
    const PointLL& p = *ap[i];
    double v2x = p.x() - a.x();
    double v2y = p.y() - a.y();

    double dot02 = v0x * v2x + v0y * v2y;
    double dot12 = v1x * v2x + v1y * v2y;

    double denom = dot00 * dot11 - dot01 * dot01;

    // Triangle with very small area, e.g., nearly a line.
    if (denom < 1e-20)
      return false;

    double u = (dot11 * dot02 - dot01 * dot12) / denom;
    double v = (dot00 * dot12 - dot01 * dot02) / denom;

    // Check if point is in triangle
    bool contains = (u >= 0) && (v >= 0) && (u + v < 1);
    if (contains)
      return true;
  }

  return false;
}

bool box_contains(const PointLL& sw,
                  const PointLL& ne,
                  const PointLL& p) {
  return (p.x() >= sw.x()) && (p.x() < ne.x()) && (p.y() >= sw.y()) && (p.y() < ne.y());
}

bool intersects(const PointLL& a,
                const PointLL& b,
                const PointLL& c,
                const PointLL& d) {
  double xa = a.lng(), xb = b.lng(), xc = c.lng(), xd = d.lng();
  double ya = a.lat(), yb = b.lat(), yc = c.lat(), yd = d.lat();

  double denom = (xd-xc)*(yb-ya) - (yd-yc)*(xb-xa);

  constexpr double tol = 1e-20;

  // parallel check
  if (fabs(denom) < tol)
    return false;

  double t0 = (yd-yc)*(xa-xc) - (xd-xc)*(ya-yc);
  t0 = t0 / denom;

  if ((t0 < tol) || (t0 > (1-tol)))
    return false;

  double t1 = (xb-xa)*(yc-ya) - (yb-ya)*(xc-xa);
  t1 = t1 / denom;

  if ((t1 < tol) || (t1 > (1-tol)))
    return false;

  return true;
}
#endif


class PointTiler {
  // key: TileId
  // value: vector of indices into points array
  std::unordered_map<TileId, std::unordered_set<size_t>, TileId_hash_functor> tiled_space;

  // How many sections the world's longitude (x) and latitude (y) are divided
  // to satisfy the given tile_width_degrees.
  unsigned int num_x_subdivisions;
  unsigned int num_y_subdivisions;

  inline TileId get_tile_id(const PointLL& pt) {
    TileId tid;
    tid.x = std::lround(num_x_subdivisions * ((pt.lng() + 180.0) / 360.0));
    tid.y = std::lround(num_y_subdivisions * ((pt.lat() + 90.0) / 180.0));
    return tid;
  }

#if 0
  void get_tile_corners(const TileId& tid, PointLL& sw, PointLL& nw, PointLL& se, PointLL& ne) {
    double minx = ((360.0*(double)tid.x) / num_x_subdivisions) - 180.0;
    double maxx = ((360.0*(double)(tid.x+1)) / num_x_subdivisions) - 180.0;
    double miny = ((180.0*(double)tid.y) / num_y_subdivisions) - 90.0;
    double maxy = ((180.0*(double)(tid.y+1)) / num_y_subdivisions) - 90.0;

    sw = {minx, miny};
    nw = {minx, maxy};
    se = {maxx, miny};
    ne = {maxx, maxy};
  }
#endif

public:
  // when points are deleted from our polyline they become this "special" value
  static const PointLL deleted_point;

  // need random access to every point
  std::vector<PointLL> polyline;

  PointTiler(double tile_width_degrees);

  template <class container_t> void tile(container_t& polyline);

  void get_points_near(const PointLL& pt, std::unordered_set<size_t>& near_pts);

  void get_points_in_and_around_triangle(const PointLL& pta,
                                         const PointLL& ptb,
                                         const PointLL& ptc,
                                         std::unordered_set<size_t>& near_pts);

#if 0
  void get_points_along_line(const PointLL& pta,
                             const PointLL& ptb,
                             std::unordered_set<size_t>& near_pts);
#endif

  void remove_point(const size_t& idx);

  void simplify_polyline(const size_t& sidx, const size_t& eidx) {
    for (size_t i = sidx + 1; i < eidx; i++) {
      remove_point(i);
    }
  }
};

// A "special" value that means "this point is deleted".
const PointLL PointTiler::deleted_point = {1000.0, 1000.0};

PointTiler::PointTiler(double tile_width_degrees) {
  assert(tile_width_degrees <= 180.0);
  assert(tile_width_degrees > 0.0);
  int level = 1;
  num_x_subdivisions = 1;
  double degrees_at_level = 180.0;
  constexpr double max_level = 32;
  while ((degrees_at_level > tile_width_degrees) && (level <= max_level)) {
    level++;
    degrees_at_level /= 2.0;
    num_x_subdivisions = num_x_subdivisions << 1;
  }

  // y's space is half that of x
  num_y_subdivisions = num_x_subdivisions >> 1;
}

template <class container_t>
void PointTiler::tile(container_t& polyline) {
  this->polyline.reserve(polyline.size());
  tiled_space.reserve(polyline.size());
  size_t index = 0;
  for (auto iter = polyline.begin(); iter != polyline.end(); iter++, index++) {
    this->polyline.emplace_back(*iter);
    TileId pid = get_tile_id(*iter);
    auto map_iter = tiled_space.find(pid);
    if (map_iter == tiled_space.end())
      tiled_space.insert(std::pair<TileId, std::unordered_set<size_t>>{pid, {index}});
    else
      map_iter->second.insert(index);
  }
}

void PointTiler::get_points_near(const PointLL& pt, std::unordered_set<size_t>& near_pts) {
  TileId pid = get_tile_id(pt);
  // TODO: handle under/overflow
  for (unsigned int x = pid.x-1; x < pid.x+1; x++) {
    for (unsigned int y = pid.y-1; y < pid.y+1; y++) {
      auto iter = tiled_space.find(TileId{x, y});
      if (iter != tiled_space.end()) {
        near_pts.insert(iter->second.begin(), iter->second.end());
      }
    }
  }
}

void PointTiler::get_points_in_and_around_triangle(const PointLL& pta,
                                                   const PointLL& ptb,
                                                   const PointLL& ptc,
                                                   std::unordered_set<size_t>& near_pts) {
  TileId pida = get_tile_id(pta);
  TileId pidb = get_tile_id(ptb);
  TileId pidc = get_tile_id(ptc);
  unsigned int minx = std::min(std::min(pida.x, pidb.x), pidc.x);
  unsigned int maxx = std::max(std::max(pida.x, pidb.x), pidc.x);
  unsigned int miny = std::min(std::min(pida.y, pidb.y), pidc.y);
  unsigned int maxy = std::max(std::max(pida.y, pidb.y), pidc.y);
  // TODO: handle tile-id under/overflow
  // TODO: follow triangle edges instead of just using a box
  for (unsigned int x = minx-1; x < maxx+1; x++) {
    for (unsigned int y = miny-1; y < maxy+1; y++) {
      TileId tid{x, y};
      auto iter = tiled_space.find(tid);
      if (iter != tiled_space.end()) {
        near_pts.insert(iter->second.begin(), iter->second.end());
      }
    }
  }
}

#if 0
void PointTiler::get_points_along_line(const PointLL& pta,
                                       const PointLL& ptb,
                                       std::unordered_set<size_t>& near_pts) {
  TileId pida = get_tile_id(pta);
  TileId pidb = get_tile_id(ptb);

  // vertical (from the tile perspective)
  if (pidb.x - pida.x == 0) {
    // make pida have the smaller y
    if (pida.y > pidb.y)
      std::swap(pida, pidb);

    for (unsigned int y = pida.y-1; y <= pidb.y+1; y++) {
      for (unsigned int x = pida.x-1; x <= pida.x+1; x++) {
        TileId tid = {x, y};
        auto iter = tiled_space.find(tid);
        if (iter != tiled_space.end()) {
          near_pts.insert(iter->second.begin(), iter->second.end());
        }
      }
    }
  }
  // horizontal (from the tile perspective)
  else if (pidb.y - pida.y == 0) {
    // make pida have the smaller x
    if (pida.x > pidb.x)
      std::swap(pida, pidb);

    for (unsigned int x = pida.x-1; x <= pidb.x+1; x++) {
      for (unsigned int y = pida.y-1; y <= pida.y+1; y++) {
        TileId tid = {x, y};
        auto iter = tiled_space.find(tid);
        if (iter != tiled_space.end()) {
          near_pts.insert(iter->second.begin(), iter->second.end());
        }
      }
    }
  }
  // diagonal (from the tile perspective)
  else {
    double dy = (double)pidb.y - (double)pida.y;
    double dx = (double)pidb.x - (double)pida.x;
    double dydx_slope = dy/dx;
    // more vertical than horizontal; step y from miny-2 to maxy+2; solve for x at y and vary
    // x width by 4 tiles
    if (std::fabs(dydx_slope) >= 1.0) {
      // make pida have the smaller y
      if (pida.y > pidb.y)
        std::swap(pida, pidb);

      double dxdy_slope = 1.0 / dydx_slope;
      double xint = pida.x - dxdy_slope * pida.y;

      for (unsigned int y = pida.y - 2; y <= pidb.y + 2; y++) {
        double xaty = dxdy_slope * (double)y + xint;
        auto xn = (unsigned int)xaty;
        for (unsigned int x = xn - 2; x <= xn + 2; x++) {
          TileId tid = {x, y};
          auto iter = tiled_space.find(tid);
          if (iter != tiled_space.end()) {
            near_pts.insert(iter->second.begin(), iter->second.end());
          }
        }
      }
    }
    // more horizontal than vertical; step x from minx-2 to maxx+2; solve for y at x and vary
    // y width by 4 tiles
    else {
      // make pida have the smaller x
      if (pida.x > pidb.x)
        std::swap(pida, pidb);

      double yint = pida.y - dydx_slope * pida.x;
      for (unsigned int x = pida.x - 2; x <= pidb.x + 2; x++) {
        double yatx = dydx_slope * (double)x + yint;
        auto yn = std::lround(yatx);
        for (unsigned int y = yn - 2; y <= yn + 2; y++) {
          TileId tid{x, y};
          auto iter = tiled_space.find(tid);
          if (iter != tiled_space.end()) {
            near_pts.insert(iter->second.begin(), iter->second.end());
          }
        }
      }
    }
  }
}
#endif

void PointTiler::remove_point(const size_t& idx) {
  // delete this entry from its tile
  auto iter = tiled_space.find(get_tile_id(polyline[idx]));
  if (iter != tiled_space.end()) {
    std::unordered_set<size_t>& pixel_points = iter->second;
    pixel_points.erase(idx);
  }

  // don't actually delete from the vector, just mark as deleted
  polyline[idx] = PointTiler::deleted_point;
}


template <class container_t>
void peucker_avoid_self_intersections(PointTiler& tiled_points,
                                      const double& epsilon_sq,
                                      const std::unordered_set<size_t>& exclusions,
                                      size_t sidx,
                                      size_t eidx ) {

  while ((exclusions.find(sidx) != exclusions.end()) && (sidx < eidx)) {
    sidx++;
  }
  while ((exclusions.find(eidx) != exclusions.end()) && (eidx > sidx)) {
    eidx--;
  }
  if (sidx >= eidx)
    return;

  const PointLL& start = tiled_points.polyline[sidx];
  const PointLL& end = tiled_points.polyline[eidx];

  double dmax = std::numeric_limits<double>::lowest();
  LineSegment2<PointLL> line_segment{start, end};

  // hfidx represents the index of the highest freq detail (the dividing point)
  size_t hfidx = sidx;

  // find the point furthest from the line-segment formed by {start, end}
  PointLL tmp;
  for (size_t idx = sidx+1; idx < eidx; idx++ ) {
    // special points we dont want to generalize no matter what take precedence
    if (exclusions.find(idx) != exclusions.end()) {
      dmax = epsilon_sq;
      hfidx = idx;
      break;
    }

    const PointLL& c = tiled_points.polyline[idx];

    // test if this is the highest frequency detail so far
    auto d = line_segment.DistanceSquared(c, tmp);
    if (d > dmax) {
      dmax = d;
      hfidx = idx;
    }
  }

  // if (dmax < epsilon_sq) then we have a relatively straight line between (start,end).
  // However, it can occur that if we were to blindly simplify this polyline we might create
  // a self-intersection. To test for that possibility, picture N triangles formed by
  // start, end and every polyline point between. Then test if any points reside in each
  // triangle. It is reasonable that some of the points along (start,end) will be inside
  // the triangle, ignore those. If any points remain, we cannot simplify this line or we'd
  // otherwise create a self-intersection.
  if (dmax < epsilon_sq) {
    bool can_simplify = true;
    std::unordered_set<size_t> exact_points_inside_triangle;
    for (size_t cidx = sidx + 1; (cidx < eidx) && can_simplify; cidx++) {
      const PointLL& c = tiled_points.polyline[cidx];

      // Get set of pixel-id's that overlap the triangle formed by (start, c, end).
      std::unordered_set<size_t> rough_points;
      tiled_points.get_points_in_and_around_triangle(start, c, end, rough_points);

      // Using a tiled space for our search space is kinda coarse. Now
      // we tighten things up by checking if any points are exactly inside
      // the triangle (start,c,end).
      exact_points_inside_triangle.clear();
      for (const size_t& point_idx : rough_points) {
        if (triangle_contains(start, c, end, tiled_points.polyline[point_idx])) {
          exact_points_inside_triangle.insert(point_idx);
        }
      }

      // Ignore triangle points that are along the polyline [start,end]
      for (size_t i = sidx; i <= eidx && !exact_points_inside_triangle.empty(); i++) {
        exact_points_inside_triangle.erase(i);
      }

      // If no points remain within the triangle, we can simplify this line
      can_simplify = exact_points_inside_triangle.empty();
    }

    if (can_simplify) {
      // Simplify the polyline
      tiled_points.simplify_polyline(sidx+1, eidx);  // [sidx, eidx)
    }
    else {
      // Simplifying this polyline would result in a self-intersection, so
      // we cannot. Force recursion around hfidx.
      dmax = epsilon_sq;
    }
  }

  // there are some high frequency details between start and end
  // so we need to look for flatter sections between them
  if (dmax >= epsilon_sq) {
    // we recurse from right to left for two reasons:
    // 1. we want to preserve iterator validity in the vector version
    // 2. its the only way to preserve the indices in the keep set
    if (eidx - hfidx > 1)
      peucker_avoid_self_intersections<container_t>(tiled_points, epsilon_sq, exclusions, hfidx, eidx);
    if (hfidx - sidx > 1)
      peucker_avoid_self_intersections<container_t>(tiled_points, epsilon_sq, exclusions, sidx, hfidx);
  }
}

template <class container_t>
void Generalize_orig(container_t& polyline,
                     double epsilon,
                     const std::unordered_set<size_t>& exclusions) {
  // any epsilon this low will have no effect on the input nor will any super short input
  if (epsilon <= 0 || polyline.size() < 3)
    return;

  // the recursive bit
  epsilon *= epsilon;
  std::function<void(typename container_t::iterator, size_t, typename container_t::iterator, size_t)>
      peucker;
  peucker = [&peucker, &polyline, epsilon, &exclusions](typename container_t::iterator start, size_t s,
                                                        typename container_t::iterator end, size_t e) {
    // find the point furthest from the line
    double dmax = std::numeric_limits<double>::lowest();
    typename container_t::iterator itr;
    LineSegment2<PointLL> l{*start, *end};
    size_t j = e - 1, k;
    PointLL tmp;
    for (auto i = std::prev(end); i != start; --i, --j) {
      // special points we dont want to generalize no matter what take precidence
      if (exclusions.find(j) != exclusions.end()) {
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


/**
 * Generalize the given list of points
 *
 * @param polyline    the list of points
 * @param epsilon     the tolerance used in removing points
 * @param exclusions  list of indices of points not to generalize
 */
template <typename coord_t>
template <class container_t>
std::tuple<long, long> Polyline2<coord_t>::Generalize(container_t& polyline,
                                    typename coord_t::value_type epsilon,
                                    const std::unordered_set<size_t>& exclusions) {
  long orig_time = 0.0;
  long new_time = 0.0;

  // any epsilon this low will have no effect on the input nor will any super short input
  if (epsilon <= 0 || polyline.size() < 3)
    return std::make_tuple(orig_time, new_time);

  {
    container_t polyline_copy(polyline);

    auto start_time = std::chrono::high_resolution_clock::now();
    Generalize_orig(polyline_copy, epsilon, exclusions);
    auto end_time = std::chrono::high_resolution_clock::now();
    uint32_t total_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    orig_time = total_time;
  }

  auto start_time = std::chrono::high_resolution_clock::now();


  // Create a tiled-space spatial-index which allows us to conduct a peucker style
  // line simplification while also avoiding creating self-intersections within the
  // polyline/polygon.
  const PointLL& first_point = *polyline.begin();
  double meters_per_deg = DistanceApproximator<PointLL>::MetersPerLngDegree(first_point.lat());
  double epsilon_in_deg = epsilon / meters_per_deg;
  PointTiler tiled_points(epsilon_in_deg);
  tiled_points.tile<container_t>(polyline);

  peucker_avoid_self_intersections<container_t>(tiled_points, epsilon*epsilon, exclusions, 0, polyline.size()-1);

  // copy the simplified polyline into 'polyline'
  polyline.clear();
  for (const auto& pt : tiled_points.polyline) {
    if (pt != PointTiler::deleted_point) {
      polyline.push_back(pt);
    }
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  uint32_t total_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
  new_time = total_time;

  return std::make_tuple(orig_time, new_time);
}


#if 0
  auto iter = polyline.begin();
  PointLL p0{ iter->x(), iter->y() };
  iter++;
  PointLL p1{ iter->x(), iter->y() };
  iter++;
  size_t ii = 2;
  for (; iter != polyline.end(); ii++, iter++) {
    PointLL p2{ iter->x(), iter->y() };
    double curvature = p0.Curvature(p1, p2);
    if (curvature < epsilon) {
      exclude.insert(ii-1);
    }
    p0 = p1;
    p1 = p2;
  }
#endif


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

template std::tuple<long, long> Polyline2<PointXY<float>>::Generalize(std::vector<PointXY<float>>&,
                                                    float,
                                                    const std::unordered_set<size_t>&);
template std::tuple<long, long> Polyline2<PointXY<double>>::Generalize(std::vector<PointXY<double>>&,
                                                     double,
                                                     const std::unordered_set<size_t>&);
template std::tuple<long, long> Polyline2<PointXY<float>>::Generalize(std::list<PointXY<float>>&,
                                                    float,
                                                    const std::unordered_set<size_t>&);
template std::tuple<long, long> Polyline2<PointXY<double>>::Generalize(std::list<PointXY<double>>&,
                                                     double,
                                                     const std::unordered_set<size_t>&);
template std::tuple<long, long> Polyline2<GeoPoint<float>>::Generalize(std::vector<GeoPoint<float>>&,
                                                     float,
                                                     const std::unordered_set<size_t>&);
template std::tuple<long, long> Polyline2<GeoPoint<double>>::Generalize(std::vector<GeoPoint<double>>&,
                                                      double,
                                                      const std::unordered_set<size_t>&);
template std::tuple<long, long> Polyline2<GeoPoint<float>>::Generalize(std::list<GeoPoint<float>>&,
                                                     float,
                                                     const std::unordered_set<size_t>&);
template std::tuple<long, long> Polyline2<GeoPoint<double>>::Generalize(std::list<GeoPoint<double>>&,
                                                      double,
                                                      const std::unordered_set<size_t>&);

} // namespace midgard
} // namespace valhalla
