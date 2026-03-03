#include "loki/search.h"
#include "loki/worker.h"
#include "tyr/serializers.h"

using namespace valhalla;
using namespace valhalla::baldr;

namespace valhalla {
namespace loki {

void loki_worker_t::init_locate(Api& request) {
  parse_locations(request.mutable_options()->mutable_locations(), request);
  if (request.options().locations_size() < 1)
    throw valhalla_exception_t{120};

  parse_costing(request, true);
}

std::string loki_worker_t::locate(Api& request) {
  // time this whole method and save that statistic
  auto _ = measure_scope_time(request);

  // correlate the various locations to the underlying graph
  init_locate(request);
  auto locations = PathLocation::fromPBF(request.options().locations());
  const auto& costing_name = Costing_Enum_Name(request.options().costing_type());
  std::unordered_map<baldr::Location, PathLocation> projections;
  if (costing_name == "auto_pedestrian") {
    if (locations.size() > 2) {
      throw valhalla_exception_t{150, "for auto_pedestrian: " + std::to_string(locations.size())};
    }
    std::vector<baldr::Location> start_loc(locations.begin(), locations.begin() + 1);
    auto start_projection =
        search_.search(start_loc, mode_costing[static_cast<size_t>(TravelMode::kDrive)]);
    for (const auto& [loc, path_loc] : start_projection) {
      projections.insert({loc, path_loc});
    }
    std::vector<baldr::Location> end_loc(locations.end() - 1, locations.end());

    auto end_projection =
        search_.search(end_loc, mode_costing[static_cast<size_t>(TravelMode::kPedestrian)]);
    for (const auto& [loc, path_loc] : end_projection) {
      projections.insert({loc, path_loc});
    }
  } else {
    projections = search_.search(locations, mode_costing[static_cast<size_t>(mode)]);
  }
  return tyr::serializeLocate(request, locations, projections, *reader);
}

} // namespace loki
} // namespace valhalla
