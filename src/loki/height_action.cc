#include "loki/worker.h"
#include "midgard/encoded.h"
#include "midgard/logging.h"
#include "proto_conversions.h"
#include "tyr/serializers.h"

using namespace valhalla;
using namespace valhalla::tyr;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::skadi;

namespace valhalla {
namespace loki {

std::vector<PointLL> loki_worker_t::init_height(Api& request) {
  auto& options = *request.mutable_options();
  // not enough shape
  if (options.shape_size() < 1) {
    throw valhalla_exception_t{312};
  };

  // convert back to native pointll :(
  std::vector<PointLL> shape;
  for (const auto& l : options.shape()) {
    shape.emplace_back(to_ll(l.ll()));
  }

  // resample the shape
  bool resampled = false;
  if (options.has_resample_distance()) {
    if (options.resample_distance() < min_resample) {
      throw valhalla_exception_t{313, " " + std::to_string(min_resample) + " meters"};
    };
    if (options.shape_size() > 1) {
      // resample the shape but make sure to keep the first and last shapepoint
      auto last = shape.back();
      shape = midgard::resample_spherical_polyline(shape, options.resample_distance());
      shape.emplace_back(std::move(last));
      // put it back
      options.clear_shape();
      for (const auto& p : shape) {
        from_ll(options.mutable_shape()->Add(), p);
      }
      // re-encode it for display if they sent it encoded
      if (options.has_encoded_polyline()) {
        // Default to 6 digit precision unless polyline5 is specified
        // NOTE: geojson is NOT support yet for height action
        int precision = options.shape_format() == polyline5 ? 1e5 : 1e6;
        options.set_encoded_polyline(midgard::encode(shape, precision));
      }
      resampled = true;
    }
  }

  // there are limits though
  if (static_cast<size_t>(options.shape_size()) > max_elevation_shape) {
    throw valhalla_exception_t{314, " (" + std::to_string(options.shape_size()) +
                                        (resampled ? " after resampling" : "") + "). The limit is " +
                                        std::to_string(max_elevation_shape)};
  }

  return shape;
}

/* example height with range response:
{
  "shape": [ {"lat": 40.712433, "lon": -76.504913}, {"lat": 40.712276, "lon": -76.605263} ],
  "range_height": [ [0,303], [8467,275], [25380,198] ]
}
*/
std::string loki_worker_t::height(Api& request) {
  // time this whole method and save that statistic
  auto _ = measure_scope_time(request);

  auto shape = init_height(request);
  // get the elevation of each posting
  std::vector<double> heights = sample.get_all(shape);

  // get the distances between the postings if desired
  std::vector<double> ranges;
  if (request.options().range()) {
    ranges.reserve(shape.size());
    ranges.emplace_back(0);
    for (auto point = std::next(shape.cbegin()); point != shape.cend(); ++point) {
      ranges.emplace_back(ranges.back() + point->Distance(*std::prev(point)));
    }
  }

  return tyr::serializeHeight(request, heights, ranges);
}
} // namespace loki
} // namespace valhalla
