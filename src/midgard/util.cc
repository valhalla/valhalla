#include "valhalla/midgard/util.h"
#include "valhalla/midgard/constants.h"
#include "valhalla/midgard/distanceapproximator.h"
#include "valhalla/midgard/point2.h"
#include "valhalla/midgard/polyline2.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <list>
#include <sstream>
#include <stdlib.h>
#include <sys/stat.h>
#include <vector>

namespace {

constexpr double RAD_PER_METER = 1.0 / 6378160.187;
constexpr double RAD_PER_DEG = valhalla::midgard::kPiDouble / 180.0;
constexpr double DEG_PER_RAD = 180.0 / valhalla::midgard::kPiDouble;

std::vector<valhalla::midgard::PointLL>
resample_at_1hz(const std::vector<valhalla::midgard::gps_segment_t>& segments) {
  std::vector<valhalla::midgard::PointLL> resampled;
  float time_remainder = 0.0;
  for (const auto& segment : segments) {
    // get the speed of this edge
    auto meters = valhalla::midgard::Polyline2<valhalla::midgard::PointLL>::Length(segment.shape);
    // trim the shape to account of the portion of the previous second that bled onto this edge
    auto to_trim = segment.speed * time_remainder;
    auto trimmed = valhalla::midgard::trim_polyline(segment.shape.cbegin(), segment.shape.cend(),
                                                    to_trim / meters, 1.f);
    // resample it at 1 second intervals
    auto second_interval =
        valhalla::midgard::resample_spherical_polyline(trimmed, segment.speed, false);
    resampled.insert(resampled.end(), second_interval.begin(), second_interval.end());
    // figure out how much of the last second will bleed into the next edge
    double intpart;
    time_remainder = std::modf((meters - to_trim) / segment.speed, &intpart);
  }
  return resampled;
}

} // namespace

namespace valhalla {
namespace midgard {

// scalar * vector operator.
Vector2 operator*(float s, const Vector2& v) {
  return Vector2(v.x() * s, v.y() * s);
}

// Trim the front of a polyline (represented as a list or vector of Point2).
// Returns the trimmed portion of the polyline. The supplied polyline is
// altered (the trimmed part is removed).
template <class container_t> container_t trim_front(container_t& pts, const float dist) {
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
      auto midpoint = p1->AffineCombination((1.0 - frac), frac, *p2);
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

float tangent_angle(size_t index,
                    const PointLL& point,
                    const std::vector<PointLL>& shape,
                    const float sample_distance,
                    bool forward) {
  // depending on if we are going forward or backward we choose a different increment
  auto increment = forward ? -1 : 1;
  auto first_end = forward ? shape.cbegin() : shape.cend() - 1;
  auto second_end = forward ? shape.cend() - 1 : shape.cbegin();

  // u and v will be points we move along the shape until we have enough distance between them or
  // run out of points

  // move backwards until we have enough or run out
  float remaining = sample_distance;
  auto u = point;
  auto i = shape.cbegin() + index + forward;
  while (remaining > 0 && i != first_end) {
    // move along and see how much distance that added
    i += increment;
    auto d = u.Distance(*i);
    // are we done yet?
    if (remaining <= d) {
      auto coef = remaining / d;
      u = u.AffineCombination(1 - coef, coef, *i);
      return u.Heading(point);
    }
    // next one
    u = *i;
    remaining -= d;
  }

  // move forwards until we have enough or run out
  auto v = point;
  i = shape.cbegin() + index + !forward;
  while (remaining > 0 && i != second_end) {
    // move along and see how much distance that added
    i -= increment;
    auto d = v.Distance(*i);
    // are we done yet?
    if (remaining <= d) {
      auto coef = remaining / d;
      v = v.AffineCombination(1 - coef, coef, *i);
      return u.Heading(v);
    }
    // next one
    v = *i;
    remaining -= d;
  }
  return u.Heading(v);
}

// Explicit instantiations
template std::vector<PointLL> trim_front<std::vector<PointLL>>(std::vector<PointLL>&, const float);
template std::vector<Point2> trim_front<std::vector<Point2>>(std::vector<Point2>&, const float);
template std::list<PointLL> trim_front<std::list<PointLL>>(std::list<PointLL>&, const float);
template std::list<Point2> trim_front<std::list<Point2>>(std::list<Point2>&, const float);

memory_status::memory_status(const std::unordered_set<std::string>& interest) {
  // grab the vm stats from the file
  std::ifstream file("/proc/self/status");
  std::string line;
  while (std::getline(file, line)) {
    // did we find a memory metric
    if (line.find_first_of("Vm") == 0) {
      // grab the name of it and see if we care about it
      std::string name = line.substr(0, line.find_first_of(':'));
      if (interest.size() > 0 && interest.find(name) == interest.end()) {
        continue;
      }
      // try to get the number of bytes
      line.erase(std::remove_if(line.begin(), line.end(),
                                [](const char c) { return !std::isdigit(c); }),
                 line.end());
      if (line.size() == 0) {
        continue;
      }
      auto bytes = std::stod(line) * 1024.0;
      // get the units and scale
      std::pair<double, std::string> metric = std::make_pair(bytes, "b");
      for (auto unit : {"B", "KB", "MB", "GB"}) {
        metric.second = unit;
        if (metric.first > 1024.0) {
          metric.first /= 1024.0;
        } else {
          break;
        }
      }
      metrics.emplace(std::piecewise_construct, std::forward_as_tuple(name),
                      std::forward_as_tuple(metric));
    }
    line.clear();
  }
}

bool memory_status::supported() {
  struct stat s;
  return stat("/proc/self/status", &s) == 0;
}

std::ostream& operator<<(std::ostream& stream, const memory_status& s) {
  for (const auto& metric : s.metrics) {
    stream << metric.first << ": " << metric.second.first << metric.second.second << std::endl;
  }
  return stream;
}

/* This method makes use of several computations explained and demonstrated at:
 * http://williams.best.vwh.net/avform.htm *
 * We humbly bow to you sir!
 */
template <class container_t>
container_t
resample_spherical_polyline(const container_t& polyline, double resolution, bool preserve) {
  if (polyline.size() == 0) {
    return {};
  };

  // for each point
  container_t resampled = {polyline.front()};
  resolution *= RAD_PER_METER;
  double remaining = resolution;
  PointLL last = resampled.back();
  for (auto p = std::next(polyline.cbegin()); p != polyline.cend(); ++p) {
    // radians
    auto lon2 = p->first * -RAD_PER_DEG;
    auto lat2 = p->second * RAD_PER_DEG;
    // how much do we have left on this segment from where we are (in great arc radians)
    // double d = 2.0 * asin(sqrt(pow(sin((resampled.back().second * RAD_PER_DEG - lat2) /
    // 2.0), 2.0) + cos(resampled.back().second * RAD_PER_DEG) * cos(lat2)
    // *pow(sin((resampled.back().first * -RAD_PER_DEG - lon2) / 2.0), 2.0)));
    auto d = acos(sin(last.second * RAD_PER_DEG) * sin(lat2) +
                  cos(last.second * RAD_PER_DEG) * cos(lat2) * cos(last.first * -RAD_PER_DEG - lon2));
    // keep placing points while we can fit them
    while (d > remaining) {
      // some precomputed stuff
      auto lon1 = last.first * -RAD_PER_DEG;
      auto lat1 = last.second * RAD_PER_DEG;
      auto sd = sin(d);
      auto a = sin(d - remaining) / sd;
      auto acs1 = a * cos(lat1);
      auto b = sin(remaining) / sd;
      auto bcs2 = b * cos(lat2);
      // find the interpolated point along the arc
      auto x = acs1 * cos(lon1) + bcs2 * cos(lon2);
      auto y = acs1 * sin(lon1) + bcs2 * sin(lon2);
      auto z = a * sin(lat1) + b * sin(lat2);
      last.first = atan2(y, x) * -DEG_PER_RAD;
      last.second = atan2(z, sqrt(x * x + y * y)) * DEG_PER_RAD;
      resampled.push_back(last);
      // we just consumed a bit
      d -= remaining;
      // we need another bit
      remaining = resolution;
    }
    // we're going to the next point so consume whatever's left
    remaining -= d;
    last = *p;
    if (preserve) {
      resampled.push_back(last);
    }
  }

  // TODO: do we want to let them know remaining?

  // hand it back
  return resampled;
}

// explicit instantiations
template std::vector<PointLL>
resample_spherical_polyline<std::vector<PointLL>>(const std::vector<PointLL>&, double, bool);
template std::vector<Point2>
resample_spherical_polyline<std::vector<Point2>>(const std::vector<Point2>&, double, bool);
template std::list<PointLL>
resample_spherical_polyline<std::list<PointLL>>(const std::list<PointLL>&, double, bool);
template std::list<Point2>
resample_spherical_polyline<std::list<Point2>>(const std::list<Point2>&, double, bool);

// Resample the polyline to the specified resolution. This is a faster and less precise
// method than resample_spherical_polyline.
std::vector<PointLL>
resample_polyline(const std::vector<PointLL>& polyline, const float length, const float resolution) {
  if (polyline.size() == 0) {
    return {};
  }

  // Add the first point
  std::vector<PointLL> resampled = {polyline.front()};

  // Compute sample distance that is near the resolution but splits the polyline equally
  size_t n = std::round(length / resolution);
  float sample_distance = length / n;

  // Iterate through line segments of the polyline
  float accumulated_d = 0.0f;
  auto p0 = polyline.cbegin();
  for (auto p1 = std::next(polyline.cbegin()); p1 != polyline.cend(); ++p0, ++p1) {
    // break if we have sampled enough
    if (resampled.size() == n) {
      break;
    }

    // Find distance (meters) between the 2 points of the input polyline.
    float d = p0->Distance(*p1);

    // Interpolate between the prior polyline point if we exceed the resolution
    // (including distance accumulated so far)
    if (d + accumulated_d > sample_distance) {
      float dlon = p1->first - p0->first;
      float dlat = p1->second - p0->second;

      // Form the first interpolated point
      float p = (sample_distance - accumulated_d) / d;
      resampled.emplace_back(p0->first + p * dlon, p0->second + p * dlat);

      // Continue to interpolate along the segment while accumulated distance is less than resolution
      float dp = sample_distance / d;
      while (p + dp < 1.0f && resampled.size() < n) {
        p += dp;
        resampled.emplace_back(p0->first + p * dlon, p0->second + p * dlat);
      }

      // Set the accumulated distance to the distance remaining on this segment
      accumulated_d = d * (1.0f - p);

    } else {
      // Have not accumulated enough distance. Add d to the accumulated distance
      accumulated_d += d;
    }
  }

  // Append the last polyline point
  resampled.push_back(std::move(polyline.back()));

  return resampled;
}

// Return the intersection of two infinite lines if any
template <class coord_t>
bool intersect(const coord_t& u, const coord_t& v, const coord_t& a, const coord_t& b, coord_t& i) {
  auto uv_xd = u.first - v.first;
  auto uv_yd = u.second - v.second;
  auto ab_xd = a.first - b.first;
  auto ab_yd = a.second - b.second;
  auto d_cross = uv_xd * ab_yd - ab_xd * uv_yd;
  // parallel or very close to it
  if (std::abs(d_cross) < 1e-5) {
    return false;
  }
  auto uv_cross = u.first * v.second - u.second * v.first;
  auto ab_cross = a.first * b.second - a.second * b.first;
  i.first = (uv_cross * ab_xd - uv_xd * ab_cross) / d_cross;
  i.second = (uv_cross * ab_yd - uv_yd * ab_cross) / d_cross;
  return true;
}
template bool intersect<PointLL>(const PointLL& u,
                                 const PointLL& v,
                                 const PointLL& a,
                                 const PointLL& b,
                                 PointLL& i);
template bool
intersect<Point2>(const Point2& u, const Point2& v, const Point2& a, const Point2& b, Point2& i);

// Return the intercept of the line passing through uv with the horizontal line defined by y
template <class coord_t>
typename coord_t::first_type
y_intercept(const coord_t& u, const coord_t& v, const typename coord_t::second_type y) {
  if (std::abs(u.first - v.first) < 1e-5) {
    return u.first;
  }
  if (std::abs(u.second - u.second) < 1e-5) {
    return NAN;
  }
  auto m = (v.second - u.second) / (v.first - u.first);
  auto b = u.second - (u.first * m);
  return (y - b) / m;
}
template PointLL::first_type
y_intercept<PointLL>(const PointLL& u, const PointLL& v, const PointLL::first_type y);
template Point2::first_type
y_intercept<Point2>(const Point2& u, const Point2& v, const Point2::first_type y);

// Return the intercept of the line passing through uv with the vertical line defined by x
template <class coord_t>
typename coord_t::first_type
x_intercept(const coord_t& u, const coord_t& v, const typename coord_t::second_type x) {
  if (std::abs(u.second - v.second) < 1e-5) {
    return u.second;
  }
  if (std::abs(u.first - v.first) < 1e-5) {
    return NAN;
  }
  auto m = (v.second - u.second) / (v.first - u.first);
  auto b = u.second - (u.first * m);
  return x * m + b;
}
template PointLL::second_type
x_intercept<PointLL>(const PointLL& u, const PointLL& v, const PointLL::second_type x);
template Point2::second_type
x_intercept<Point2>(const Point2& u, const Point2& v, const Point2::second_type x);

template <class container_t> float polygon_area(const container_t& polygon) {
  typename container_t::value_type::first_type area =
      polygon.back() == polygon.front() ? 0.f
                                        : (polygon.back().first + polygon.front().first) *
                                              (polygon.back().second + polygon.front().second);
  for (auto p1 = polygon.cbegin(), p2 = std::next(polygon.cbegin()); p2 != polygon.cend();
       ++p1, ++p2) {
    area += (p1->first + p2->first) * (p1->second + p2->second);
  }
  return area * .5;
}

template PointLL::first_type polygon_area(const std::list<PointLL>&);
template PointLL::first_type polygon_area(const std::vector<PointLL>&);
template Point2::first_type polygon_area(const std::list<Point2>&);
template Point2::first_type polygon_area(const std::vector<Point2>&);

std::vector<midgard::PointLL> simulate_gps(const std::vector<gps_segment_t>& segments,
                                           std::vector<float>& accuracies,
                                           float smoothing,
                                           float accuracy,
                                           size_t sample_rate,
                                           unsigned seed) {
  // resample the coords along a given edge at one second intervals
  auto resampled = resample_at_1hz(segments);

  // a way to get noise but only allow for slow change
  std::mt19937 generator(seed);
  std::uniform_real_distribution<float> distribution(-1, 1);
  ring_queue_t<std::pair<float, float>> noises(smoothing);
  auto get_noise = [&]() {
    // we generate a vector whose magnitude is no more than accuracy
    auto lon_adj = distribution(generator);
    auto lat_adj = distribution(generator);
    auto len = std::sqrt((lon_adj * lon_adj) + (lat_adj * lat_adj));
    lon_adj /= len;
    lat_adj /= len; // norm
    auto scale = (distribution(generator) + 1.f) / 2.f;
    lon_adj *= scale * accuracy;
    lat_adj *= scale * accuracy; // random scale <= accuracy
    noises.emplace_back(std::make_pair(lon_adj, lat_adj));
    // average over last n to smooth
    std::pair<float, float> noise{0, 0};
    std::for_each(noises.begin(), noises.end(), [&noise](const std::pair<float, float>& n) {
      noise.first += n.first;
      noise.second += n.second;
    });
    noise.first /= noises.size();
    noise.second /= noises.size();
    return noise;
  };
  // fill up the noise queue so the first points arent unsmoothed
  while (!noises.full()) {
    get_noise();
  }

  // for each point of the 1hz shape
  std::vector<midgard::PointLL> simulated;
  for (size_t i = 0; i < resampled.size(); ++i) {
    const auto& p = resampled[i];
    // is this a harmonic of the desired sampling rate
    if (i % sample_rate == 0) {
      // meters of noise with extremely low likelihood its larger than accuracy
      auto noise = get_noise();
      // use the number of meters per degree in both axis to offset the point by the noise
      auto metersPerDegreeLon = DistanceApproximator::MetersPerLngDegree(p.second);
      simulated.emplace_back(midgard::PointLL(p.first + noise.first / metersPerDegreeLon,
                                              p.second + noise.second / kMetersPerDegreeLat));
      // keep the distance to use for accuracy
      accuracies.emplace_back(simulated.back().Distance(p));
    }
  }
  return simulated;
}

polygon_t to_boundary(const std::unordered_set<uint32_t>& region, const Tiles<PointLL>& tiles) {
  // do we have this tile in this region
  auto member = [&region](int32_t tile) { return region.find(tile) != region.cend(); };
  // get the neighbor tile giving -1 if no neighbor
  auto neighbor = [&tiles](int32_t tile, int side) -> int32_t {
    if (tile == -1) {
      return -1;
    }
    auto rc = tiles.GetRowColumn(tile);
    switch (side) {
      default:
      case 0:
        return rc.second == 0 ? -1 : tile - 1;
      case 1:
        return rc.first == 0 ? -1 : tile - tiles.ncolumns();
      case 2:
        return rc.second == tiles.ncolumns() - 1 ? -1 : tile + 1;
      case 3:
        return rc.first == tiles.nrows() - 1 ? -1 : tile + tiles.ncolumns();
    }
  };
  // get the beginning coord of the counter clockwise winding given edge of the given tile
  auto coord = [&tiles](uint32_t tile, int side) -> PointLL {
    auto box = tiles.TileBounds(tile);
    switch (side) {
      default:
      case 0:
        return PointLL(box.minx(), box.maxy());
      case 1:
        return box.minpt();
      case 2:
        return PointLL(box.maxx(), box.miny());
      case 3:
        return box.maxpt();
    }
  };
  // trace a ring of the polygon
  polygon_t polygon;
  std::array<std::unordered_set<uint32_t>, 4> used;
  auto trace = [&member, &neighbor, &coord, &polygon, &used](uint32_t start_tile, int start_side,
                                                             bool ccw) {
    auto tile = start_tile;
    auto side = start_side;
    polygon.emplace_back();
    auto& ring = polygon.back();
    // walk until you see the starting edge again
    do {
      // add this edges geometry
      if (ccw) {
        ring.push_back(coord(tile, side));
      } else {
        ring.push_front(coord(tile, side));
      }
      auto inserted = used[side].insert(tile);
      if (!inserted.second) {
        throw std::logic_error("Any tile edge can only be used once as part of the geometry");
      }
      // we need to go to the first existing neighbor tile following our winding
      // starting with the one on the other side of the current side
      auto adjc = neighbor(tile, (side + 1) % 4);
      auto diag = neighbor(adjc, side);
      if (member(diag)) {
        tile = diag;
        side = (side + 3) % 4;
      } // next one keep following winding
      else if (member(adjc)) {
        tile = adjc;
      } // if neither of those were there we stay on this tile and go to the next side
      else {
        side = (side + 1) % 4;
      }
    } while (tile != start_tile || side != start_side);
  };

  // the smallest numbered tile has a left edge on the outer ring of the polygon
  auto start_tile = *region.cbegin();
  int start_side = 0;
  for (auto tile : region) {
    if (tile < start_tile) {
      start_tile = tile;
    }
  }

  // trace the outer
  trace(start_tile, start_side, true);

  // trace the inners
  for (auto start_tile : region) {
    // if the neighbor isnt a member and we didnt already use the side between them
    for (start_side = 0; start_side < 4; ++start_side) {
      if (!member(neighbor(start_tile, start_side)) &&
          used[start_side].find(start_tile) == used[start_side].cend()) {
        // build the inner ring
        if (start_side != -1) {
          trace(start_tile, start_side, false);
        }
      }
    }
  }

  // close all the rings
  for (auto& ring : polygon) {
    ring.push_back(ring.front());
  }

  // give it back
  return polygon;
}

} // namespace midgard
} // namespace valhalla
