// -*- mode: c++ -*-
#ifndef MMP_GEOMETRY_HELPERS_H_
#define MMP_GEOMETRY_HELPERS_H_
#include <cmath>

#include <vector>
#include <algorithm>

#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/distanceapproximator.h>
#include <valhalla/midgard/pointll.h>

namespace valhalla{
namespace meili {
namespace helpers {

// snapped point, sqaured distance, segment index, offset
template <typename coord_t>
std::tuple<coord_t, float, typename std::vector<coord_t>::size_type, float>
Project(const coord_t& p,
        const typename std::vector<coord_t>& shape,
        const midgard::DistanceApproximator& approximator,
        float snap_distance = 0.f)
{
  if (shape.empty()) {
    throw std::invalid_argument("got empty shape");
  }

  coord_t closest_point(shape.front());
  float closest_distance = approximator.DistanceSquared(closest_point);
  decltype(shape.size()) closest_segment = 0;
  float closest_partial_length = 0.f;
  float total_length = 0.f;
  float lon_scale = cosf(p.lat() * midgard::kRadPerDeg);

  //for each segment
  for(decltype(shape.size()) i = 0; i < shape.size() - 1; ++i) {
    //project a onto b where b is the origin vector representing this segment
    //and a is the origin vector to the point we are projecting, (a.b/b.b)*b
    const auto& u = shape[i];
    const auto& v = shape[i + 1];
    auto bx = v.first - u.first;
    auto by = v.second - u.second;
    auto bx2 = bx * lon_scale;
    auto sq = bx2*bx2 + by*by;
    const auto scale = sq > 0? (((p.first - u.first)*lon_scale*bx2 + (p.second - u.second)*by) / sq) : 0.f;
    //projects along the ray before u
    if (scale <= 0.f) {
      bx = u.first;
      by = u.second;
    }//projects along the ray after v
    else if (scale >= 1.f) {
      bx = v.first;
      by = v.second;
    }//projects along the ray between u and v
    else {
      bx = bx*scale + u.first;
      by = by*scale + u.second;
    }
    //check if this point is better
    coord_t point(bx, by);
    const auto distance = approximator.DistanceSquared(point);
    if (distance < closest_distance) {
      closest_point = std::move(point);
      closest_distance = distance;
      closest_segment = i;
      closest_partial_length = total_length;
    }

    //total edge length
    total_length += u.Distance(v);
  }

  // Offset is a float between 0 and 1 representing the location of
  // the closest point on LineString to the given Point, as a fraction
  // of total 2d line length.
  closest_partial_length += shape[closest_segment].Distance(closest_point);
  float offset = total_length > 0.f? static_cast<float>(closest_partial_length / total_length) : 0.f;
  offset = std::max(0.f, std::min(offset, 1.f));

  // Snapp to vertexes if it's close
  if (total_length * offset <= snap_distance) {
    closest_point = shape.front();
    closest_distance = approximator.DistanceSquared(closest_point);
    closest_segment = 0;
    offset = 0.f;
  } else if (total_length * (1.f - offset) <= snap_distance) {
    closest_point = shape.back();
    closest_distance = approximator.DistanceSquared(closest_point);
    closest_segment = shape.size() - 1;
    offset = 1.f;
  }

  return std::make_tuple(std::move(closest_point), closest_distance, closest_segment, offset);
}

}
}
}
#endif // MMP_GEOMETRY_HELPERS_H_
