#include <cmath>
#include "valhalla/midgard/util.h"
#include "valhalla/midgard/constants.h"
#include "valhalla/midgard/point2.h"
#include "valhalla/midgard/distanceapproximator.h"

#include <cstdint>
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <vector>
#include <list>
#include <sys/stat.h>

namespace {

constexpr double RAD_PER_METER  = 1.0 / 6378160.187;
constexpr double RAD_PER_DEG = valhalla::midgard::kPiDouble / 180.0;
constexpr double DEG_PER_RAD = 180.0 / valhalla::midgard::kPiDouble;

}

namespace valhalla {
namespace midgard {

// Trim the front of a polyline (represented as a list or vector of Point2).
// Returns the trimmed portion of the polyline. The supplied polyline is
// altered (the trimmed part is removed).
template <class container_t>
container_t trim_front(container_t& pts, const float dist) {
  // Return if less than 2 points
  if (pts.size() < 2) {
    return {};
  }

  // Walk the polyline and accumulate length until it exceeds dist
  container_t result;
  result.push_back(pts.front());
  double d = 0.0f;
  for (auto p1 = pts.begin(), p2 = std::next(pts.begin()); p2 != pts.end(); ++p1, ++p2) {
    double segdist = p1->Distance(*p2);
    if ((d + segdist) > dist) {
      double frac = (dist - d) / segdist;
      auto midpoint = p1->AffineCombination((1.0-frac), frac, *p2);
      result.push_back(midpoint);

      // Remove used part of polyline
      pts.erase(pts.begin(), p1);
      pts.front() = midpoint;
      return result;
    } else {
      d += segdist;
      result.push_back(*p2);
    }
  }

  // Used all of the polyline without exceeding dist
  pts.clear();
  return result;
}

// Explicit instantiations
template std::vector<PointLL> trim_front<std::vector<PointLL> >(std::vector<PointLL>&, const float);
template std::vector<Point2> trim_front<std::vector<Point2> >(std::vector<Point2>&, const float);
template std::list<PointLL> trim_front<std::list<PointLL> >(std::list<PointLL>&, const float);
template std::list<Point2> trim_front<std::list<Point2> >(std::list<Point2>&, const float);

memory_status::memory_status(const std::unordered_set<std::string> interest){
  //grab the vm stats from the file
  std::ifstream file("/proc/self/status");
  std::string line;
  while(std::getline(file, line)){
    //did we find a memory metric
    if(line.find_first_of("Vm") == 0){
      //grab the name of it and see if we care about it
      std::string name = line.substr(0, line.find_first_of(':'));
      if(interest.size() > 0 && interest.find(name) == interest.end())
        continue;
      //try to get the number of bytes
      line.erase(std::remove_if(line.begin(), line.end(), [](const char c) {return !std::isdigit(c);}), line.end());
      if(line.size() == 0)
        continue;
      auto bytes = std::stod(line) * 1024.0;
      //get the units and scale
      std::pair<double, std::string> metric = std::make_pair(bytes, "b");
      for(auto unit : { "B", "KB", "MB", "GB" }){
        metric.second = unit;
        if (metric.first > 1024.0)
          metric.first /= 1024.0;
        else
          break;
      }
      metrics.emplace(std::piecewise_construct, std::forward_as_tuple(name), std::forward_as_tuple(metric));
    }
    line.clear();
  }
}

bool memory_status::supported() {
  struct stat s;
  return stat("/proc/self/status", &s) == 0;
}

std::ostream& operator<<(std::ostream& stream, const memory_status& s){
  for(const auto& metric : s.metrics)
    stream << metric.first << ": " << metric.second.first << metric.second.second << std::endl;
  return stream;
}

/* This method makes use of several computations explained and demonstrated at:
 * http://williams.best.vwh.net/avform.htm *
 * We humbly bow to you sir!
 */
template <class container_t>
container_t resample_spherical_polyline(const container_t& polyline, double resolution, bool preserve) {
  if(polyline.size() == 0)
    return {};

  //for each point
  container_t resampled = {polyline.front()};
  resolution *= RAD_PER_METER;
  double remaining = resolution;
  PointLL last = resampled.back();
  for(auto p = std::next(polyline.cbegin()); p != polyline.cend(); ++p) {
    //radians
    auto lon2 = p->first * -RAD_PER_DEG;
    auto lat2 = p->second * RAD_PER_DEG;
    //how much do we have left on this segment from where we are (in great arc radians)
    //double d = 2.0 * asin(sqrt(pow(sin((resampled.back().second * RAD_PER_DEG - lat2) / 2.0), 2.0) + cos(resampled.back().second * RAD_PER_DEG) * cos(lat2) *pow(sin((resampled.back().first * -RAD_PER_DEG - lon2) / 2.0), 2.0)));
    auto d = acos(
      sin(last.second * RAD_PER_DEG) * sin(lat2) +
      cos(last.second * RAD_PER_DEG) * cos(lat2) *
      cos(last.first * -RAD_PER_DEG - lon2)
    );
    //keep placing points while we can fit them
    while(d > remaining) {
      //some precomputed stuff
      auto lon1 = last.first * -RAD_PER_DEG;
      auto lat1 = last.second * RAD_PER_DEG;
      auto sd = sin(d);
      auto a = sin(d - remaining) / sd;
      auto acs1 = a * cos(lat1);
      auto b = sin(remaining) / sd;
      auto bcs2 = b * cos(lat2);
      //find the interpolated point along the arc
      auto x = acs1 * cos(lon1) + bcs2 * cos(lon2);
      auto y = acs1 * sin(lon1) + bcs2 * sin(lon2);
      auto z = a * sin(lat1) + b * sin(lat2);
      last.first = atan2(y, x) * -DEG_PER_RAD;
      last.second = atan2(z, sqrt(x * x + y * y)) * DEG_PER_RAD;
      resampled.push_back(last);
      //we just consumed a bit
      d -= remaining;
      //we need another bit
      remaining = resolution;
    }
    //we're going to the next point so consume whatever's left
    remaining -= d;
    last = *p;
    if(preserve)
      resampled.push_back(last);
  }

  //TODO: do we want to let them know remaining?

  //hand it back
  return resampled;
}

//explicit instantiations
template std::vector<PointLL> resample_spherical_polyline<std::vector<PointLL> >(const std::vector<PointLL>&, double, bool);
template std::vector<Point2> resample_spherical_polyline<std::vector<Point2> >(const std::vector<Point2>&, double, bool);
template std::list<PointLL> resample_spherical_polyline<std::list<PointLL> >(const std::list<PointLL>&, double, bool);
template std::list<Point2> resample_spherical_polyline<std::list<Point2> >(const std::list<Point2>&, double, bool);

//Return the intersection of two infinite lines if any
template <class coord_t>
bool intersect(const coord_t& u, const coord_t& v, const coord_t& a, const coord_t& b, coord_t& i) {
  auto uv_xd = u.first - v.first;
  auto uv_yd = u.second - v.second;
  auto ab_xd = a.first - b.first;
  auto ab_yd = a.second - b.second;
  auto d_cross = uv_xd*ab_yd - ab_xd*uv_yd;
  //parallel or very close to it
  if(std::abs(d_cross) < 1e-5)
    return false;
  auto uv_cross = u.first*v.second - u.second*v.first;
  auto ab_cross = a.first*b.second - a.second*b.first;
  i.first  = (uv_cross*ab_xd - uv_xd*ab_cross) / d_cross;
  i.second = (uv_cross*ab_yd - uv_yd*ab_cross) / d_cross;
  return true;
}
template bool intersect<PointLL>(const PointLL& u, const PointLL& v, const PointLL& a, const PointLL& b, PointLL& i);
template bool intersect<Point2>(const Point2& u, const Point2& v, const Point2& a, const Point2& b, Point2& i);

//Return the intercept of the line passing through uv with the horizontal line defined by y
template <class coord_t>
typename coord_t::first_type y_intercept(const coord_t& u, const coord_t& v, const typename coord_t::second_type y) {
  if(std::abs(u.first - v.first) < 1e-5)
    return u.first;
  if(std::abs(u.second - u.second) < 1e-5)
    return NAN;
  auto m = (v.second - u.second) / (v.first - u.first);
  auto b = u.second - (u.first * m);
  return (y - b) / m;
}
template PointLL::first_type y_intercept<PointLL>(const PointLL& u, const PointLL& v, const PointLL::first_type y);
template Point2::first_type y_intercept<Point2>(const Point2& u, const Point2& v, const Point2::first_type y);

//Return the intercept of the line passing through uv with the vertical line defined by x
template <class coord_t>
typename coord_t::first_type x_intercept(const coord_t& u, const coord_t& v, const typename coord_t::second_type x) {
  if(std::abs(u.second - v.second) < 1e-5)
    return u.second;
  if(std::abs(u.first - v.first) < 1e-5)
    return NAN;
  auto m = (v.second - u.second) / (v.first - u.first);
  auto b = u.second - (u.first * m);
  return x * m + b;
}
template PointLL::second_type x_intercept<PointLL>(const PointLL& u, const PointLL& v, const PointLL::second_type x);
template Point2::second_type x_intercept<Point2>(const Point2& u, const Point2& v, const Point2::second_type x);

template <class container_t>
float polygon_area(const container_t& polygon) {
  typename container_t::value_type::first_type area = polygon.back() == polygon.front() ? 0.f :
    (polygon.back().first + polygon.front().first)*(polygon.back().second + polygon.front().second);
  for(auto p1 = polygon.cbegin(), p2 = std::next(polygon.cbegin()); p2 != polygon.cend(); ++p1, ++p2)
    area += (p1->first + p2->first)*(p1->second + p2->second);
  return area*.5;
}

template PointLL::first_type polygon_area(const std::list<PointLL>&);
template PointLL::first_type polygon_area(const std::vector<PointLL>&);
template Point2::first_type polygon_area(const std::list<Point2>&);
template Point2::first_type polygon_area(const std::vector<Point2>&);

}
}
