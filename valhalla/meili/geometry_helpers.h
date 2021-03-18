// -*- mode: c++ -*-
#pragma once
#include <cmath>

#include <vector>

#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/distanceapproximator.h>
#include <valhalla/midgard/encoded.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/util.h>

namespace valhalla {
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
