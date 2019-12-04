#include "baldr/datetime.h"
#include "baldr/rapidjson_utils.h"
#include "loki/search.h"
#include "loki/worker.h"
#include "midgard/logging.h"

using namespace valhalla;
using namespace valhalla::baldr;

namespace {
midgard::PointLL to_ll(const valhalla::Location& l) {
  return midgard::PointLL{l.ll().lng(), l.ll().lat()};
}

void check_distance(const google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
                    float matrix_max_distance,
                    float& max_location_distance) {
  // see if any locations pairs are unreachable or too far apart
  for (auto source = locations.begin(); source != locations.end() - 1; ++source) {
    for (auto target = source + 1; target != locations.end(); ++target) {
      // check if distance between latlngs exceed max distance limit
      auto path_distance = to_ll(*source).Distance(to_ll(*target));

      if (path_distance >= max_location_distance) {
        max_location_distance = path_distance;
      }

      if (path_distance > matrix_max_distance) {
        throw valhalla_exception_t{154};
      };
    }
  }
}
} // namespace

namespace valhalla {
namespace loki {

void loki_worker_t::init_isochrones(Api& request) {
  auto& options = *request.mutable_options();

  // strip off unused information
  parse_locations(options.mutable_locations());
  if (options.locations_size() < 1) {
    throw valhalla_exception_t{120};
  };
  for (auto& l : *options.mutable_locations()) {
    l.clear_heading();
  }

  // check that the number of contours is ok
  if (options.contours_size() < 1) {
    throw valhalla_exception_t{113};
  } else if (options.contours_size() > max_contours) {
    throw valhalla_exception_t{152, std::to_string(max_contours)};
  }

  // validate the contour time by checking the last one
  const auto contour = options.contours().rbegin();
  if (contour->time() > max_time) {
    throw valhalla_exception_t{151, std::to_string(max_time)};
  }

  parse_costing(request);
}
void loki_worker_t::isochrones(Api& request) {
  init_isochrones(request);
  auto& options = *request.mutable_options();
  // check that location size does not exceed max
  if (options.locations_size() > max_locations.find("isochrone")->second) {
    throw valhalla_exception_t{150, std::to_string(max_locations.find("isochrone")->second)};
  };

  // check the distances
  auto max_location_distance = std::numeric_limits<float>::min();
  check_distance(options.locations(), max_distance.find("isochrone")->second, max_location_distance);
  if (!options.do_not_track()) {
    valhalla::midgard::logging::Log("max_location_distance::" +
                                        std::to_string(max_location_distance * midgard::kKmPerMeter) +
                                        "km",
                                    " [ANALYTICS] ");
  }

  try {
    // correlate the various locations to the underlying graph
    auto locations = PathLocation::fromPBF(options.locations());
    const auto projections = loki::Search(locations, *reader, costing.get());
    for (size_t i = 0; i < locations.size(); ++i) {
      const auto& projection = projections.at(locations[i]);
      PathLocation::toPBF(projection, options.mutable_locations(i), *reader);
    }
  } catch (const std::exception&) { throw valhalla_exception_t{171}; }
}

} // namespace loki
} // namespace valhalla
