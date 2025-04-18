// -*- mode: c++ -*-
#pragma once
#include <cmath>
#include <vector>

namespace valhalla {
namespace midgard {
template <class T> class GeoPoint;
using PointLL = GeoPoint<double>;
template <class T> class Shape7Decoder;
class projector_t;
} // namespace midgard
namespace meili {
namespace helpers {

// snapped point, squared distance, segment index, offset
std::tuple<midgard::PointLL, double, typename std::vector<midgard::PointLL>::size_type, double>
Project(const midgard::projector_t& p,
        midgard::Shape7Decoder<midgard::PointLL>& shape,
        double snap_distance = 0.0);

} // namespace helpers
} // namespace meili
} // namespace valhalla
