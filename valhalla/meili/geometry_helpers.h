// -*- mode: c++ -*-
#pragma once
#include <cmath>

#include <algorithm>
#include <vector>

#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/distanceapproximator.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/util.h>

namespace valhalla {
namespace meili {
namespace helpers {

// snapped point, sqaured distance, segment index, offset
inline std::tuple<midgard::PointLL, float, typename std::vector<midgard::PointLL>::size_type, float>
Project(const midgard::projector_t& p,
        const typename std::vector<midgard::PointLL>& shape,
        float snap_distance = 0.f) {
  if (shape.empty()) {
    throw std::invalid_argument("got empty shape");
  }

  midgard::PointLL closest_point(shape.front());
  float closest_distance = p.approx.DistanceSquared(closest_point);
  decltype(shape.size()) closest_segment = 0;
  float closest_partial_length = 0.f;
  float total_length = 0.f;

  // for each segment
  for (decltype(shape.size()) i = 0; i < shape.size() - 1; ++i) {
    // project a onto b where b is the origin vector representing this segment
    // and a is the origin vector to the point we are projecting, (a.b/b.b)*b
    const auto& u = shape[i];
    const auto& v = shape[i + 1];
    auto point = p(u, v);

    // check if this point is better
    const auto distance = p.approx.DistanceSquared(point);
    if (distance < closest_distance) {
      closest_point = std::move(point);
      closest_distance = distance;
      closest_segment = i;
      closest_partial_length = total_length;
    }

    // total edge length
    total_length += u.Distance(v);
  }

  // Offset is a float between 0 and 1 representing the location of
  // the closest point on LineString to the given Point, as a fraction
  // of total 2d line length.
  closest_partial_length += shape[closest_segment].Distance(closest_point);
  float offset = total_length > 0.f ? static_cast<float>(closest_partial_length / total_length) : 0.f;
  offset = std::max(0.f, std::min(offset, 1.f));

  // Snap to vertices if it's close
  if (total_length * offset <= snap_distance) {
    closest_point = shape.front();
    closest_distance = p.approx.DistanceSquared(closest_point);
    closest_segment = 0;
    offset = 0.f;
  } else if (total_length * (1.f - offset) <= snap_distance) {
    closest_point = shape.back();
    closest_distance = p.approx.DistanceSquared(closest_point);
    closest_segment = shape.size() - 1;
    offset = 1.f;
  }

  return std::make_tuple(std::move(closest_point), closest_distance, closest_segment, offset);
}

} // namespace helpers
} // namespace meili
} // namespace valhalla
