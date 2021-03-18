#include "meili/geometry_helpers.h"

#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/distanceapproximator.h>
#include <valhalla/midgard/encoded.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/util.h>

using namespace valhalla;
using namespace valhalla::meili;
using namespace valhalla::midgard;

namespace valhalla {
namespace meili {
namespace helpers {

// snapped point, squared distance, segment index, offset
std::tuple<PointLL, double, typename std::vector<PointLL>::size_type, double>
Project(const projector_t& p, Shape7Decoder<midgard::PointLL>& shape, double snap_distance) {
  PointLL first_point(shape.pop());
  auto closest_point = first_point;
  auto closest_segment_point = first_point;
  double closest_distance = std::numeric_limits<double>::max();
  size_t closest_segment = 0;
  double closest_partial_length = 0.0;
  double total_length = 0.0;

  // for each segment
  auto u = first_point;
  size_t i = 0;
  for (; !shape.empty(); ++i) {
    // project a onto b where b is the origin vector representing this segment
    // and a is the origin vector to the point we are projecting, (a.b/b.b)*b
    auto v = shape.pop();

    auto projection = p(u, v);

    // check if this point is better
    const auto distance = p.approx.DistanceSquared(projection);
    if (distance < closest_distance) {
      closest_point = std::move(projection);
      closest_distance = distance;
      closest_segment = i;
      closest_partial_length = total_length;
      closest_segment_point = u;
    }

    // total edge length
    total_length += u.Distance(v);
    u = v;
  }

  // percent_along is a double between 0 and 1 representing the location of
  // the closest point on LineString to the given Point, as a fraction
  // of total 2d line length.
  closest_partial_length += closest_segment_point.Distance(closest_point);
  double percent_along =
      total_length > 0.0 ? static_cast<double>(closest_partial_length / total_length) : 0.0;

  // Not so much "snapping" as recognizing that floating-point has limited precision.
  // For example:
  // double k = 1e-18;         // valid
  // double r = 1.0 - k;       // sure why not
  // Result: r == 1.0          // exactly 1.0?! yep.
  //
  // Downstream logic looks at percentages along the edge and the opp-edge. We want to be
  // sure that values very close to 0.0 snap to 0.0 since downstream math may flip this
  // percentage_along to (1.0-percentage_along) if the the edge is not forward, making it
  // exactly 1.0. This causes issues because the percentage_along the opp_edge is not
  // exactly 0.0, which is not expected.
  //
  // double has ~16 decimal digits of precision. Hence, we will consider 1e-15 as our
  // representative small figure, below which we snap to 0.0 and within that distance
  // of 1.0 will snap to 1.0.
  constexpr double double_precision_at_one = 1e-15;
  constexpr double very_close_to_one = 1.0 - double_precision_at_one;
  if (percent_along < double_precision_at_one) {
    percent_along = 0.0;
  } else if (percent_along > very_close_to_one) {
    percent_along = 1.0;
  }

  // Snap to nearest node using snap_distance
  if (total_length * percent_along <= snap_distance) {
    closest_point = first_point;
    closest_distance = p.approx.DistanceSquared(closest_point);
    closest_segment = 0;
    percent_along = 0.f;
  } else if (total_length * (1.f - percent_along) <= snap_distance) {
    closest_point = u;
    closest_distance = p.approx.DistanceSquared(closest_point);
    closest_segment = i - 1;
    percent_along = 1.f;
  }

  return std::make_tuple(std::move(closest_point), closest_distance, closest_segment, percent_along);
}

} // namespace helpers
} // namespace meili
} // namespace valhalla
