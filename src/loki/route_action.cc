#include "loki/search.h"
#include "loki/worker.h"

#include "baldr/datetime.h"
#include "baldr/rapidjson_utils.h"
#include "baldr/tilehierarchy.h"
#include "midgard/logging.h"

using namespace valhalla;
using namespace valhalla::baldr;

namespace {
midgard::PointLL to_ll(const valhalla::Location& l) {
  return midgard::PointLL{l.ll().lng(), l.ll().lat()};
}

void check_locations(const size_t location_count, const size_t max_locations) {
  // check that location size does not exceed max.
  if (location_count > max_locations) {
    throw valhalla_exception_t{150, std::to_string(max_locations)};
  };
}

void check_distance(const google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
                    float max_distance) {
  // test if total distance along a polyline formed by connecting locations exceeds the maximum
  float total_path_distance = 0.0f;
  for (auto location = ++locations.begin(); location != locations.end(); ++location) {
    // check if distance between latlngs exceed max distance limit for each mode of travel
    auto path_distance = to_ll(*std::prev(location)).Distance(to_ll(*location));
    max_distance -= path_distance;
    if (max_distance < 0) {
      throw valhalla_exception_t{154};
    }
    total_path_distance += path_distance;
  }
  valhalla::midgard::logging::Log("total_location_distance::" +
                                      std::to_string(total_path_distance * midgard::kKmPerMeter) +
                                      "km",
                                  " [ANALYTICS] ");
}

} // namespace

namespace valhalla {
namespace loki {

void loki_worker_t::init_route(Api& request) {
  parse_locations(request.mutable_options()->mutable_locations());
  // need to check location size here instead of in parse_locations because of locate action needing
  // a different size
  if (request.options().locations_size() < 2) {
    throw valhalla_exception_t{120};
  };
  parse_costing(request);
}

void loki_worker_t::route(Api& request) {
  init_route(request);
  auto& options = *request.mutable_options();
  auto costing_name = Costing_Enum_Name(options.costing());
  check_locations(options.locations_size(), max_locations.find(costing_name)->second);
  check_distance(options.locations(), max_distance.find(costing_name)->second);

  // Validate walking distances (make sure they are in the accepted range)
  if (costing_name == "multimodal" || costing_name == "transit") {
    auto* ped_opts = options.mutable_costing_options(static_cast<int>(pedestrian));
    if (!ped_opts->has_transit_start_end_max_distance())
      ped_opts->set_transit_start_end_max_distance(min_transit_walking_dis);
    auto transit_start_end_max_distance = ped_opts->transit_start_end_max_distance();

    if (!ped_opts->has_transit_transfer_max_distance())
      ped_opts->set_transit_transfer_max_distance(min_transit_walking_dis);
    auto transit_transfer_max_distance = ped_opts->transit_transfer_max_distance();

    if (transit_start_end_max_distance < min_transit_walking_dis ||
        transit_start_end_max_distance > max_transit_walking_dis) {
      throw valhalla_exception_t{155, " Min: " + std::to_string(min_transit_walking_dis) + " Max: " +
                                          std::to_string(max_transit_walking_dis) + " (Meters)"};
    }
    if (transit_transfer_max_distance < min_transit_walking_dis ||
        transit_transfer_max_distance > max_transit_walking_dis) {
      throw valhalla_exception_t{156, " Min: " + std::to_string(min_transit_walking_dis) + " Max: " +
                                          std::to_string(max_transit_walking_dis) + " (Meters)"};
    }
  }

  // correlate the various locations to the underlying graph
  std::unordered_map<size_t, size_t> color_counts;
  try {
    auto locations = PathLocation::fromPBF(options.locations(), true);
    const auto projections = loki::Search(locations, *reader, costing.get());
    for (size_t i = 0; i < locations.size(); ++i) {
      const auto& correlated = projections.at(locations[i]);
      PathLocation::toPBF(correlated, options.mutable_locations(i), *reader);
      // TODO: get transit level for transit costing
      // TODO: if transit send a non zero radius
      if (!connectivity_map) {
        continue;
      }
      auto colors =
          connectivity_map->get_colors(TileHierarchy::levels().rbegin()->first, correlated, 0);
      for (auto color : colors) {
        auto itr = color_counts.find(color);
        if (itr == color_counts.cend()) {
          color_counts[color] = 1;
        } else {
          ++itr->second;
        }
      }
    }
  } catch (const std::exception&) { throw valhalla_exception_t{171}; }

  // are all the locations in the same color regions
  if (!connectivity_map) {
    return;
  }
  bool connected = false;
  for (const auto& c : color_counts) {
    if (c.second == options.locations_size()) {
      connected = true;
      break;
    }
  }
  if (!connected) {
    throw valhalla_exception_t{170};
  };
}
} // namespace loki
} // namespace valhalla
