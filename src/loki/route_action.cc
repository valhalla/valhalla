#include "baldr/tilehierarchy.h"
#include "loki/search.h"
#include "loki/worker.h"

using namespace valhalla;
using namespace valhalla::baldr;

namespace {

void check_locations(const size_t location_count, const size_t max_locations) {
  // check that location size does not exceed max.
  if (location_count > max_locations) {
    throw valhalla_exception_t{150, std::to_string(max_locations)};
  };
}

void check_distance(const google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
                    float max_distance,
                    bool all_pairs) {
  // test if total distance along a polyline formed by connecting locations exceeds the maximum
  // or if all_pairs is specified test all pairs of locations to see if any are over the threshold
  float total_path_distance = 0.0f;
  for (int i = 0; i < locations.size(); ++i) {
    for (int j = i + 1; j < locations.size(); ++j) {
      auto dist = to_ll(locations.Get(i)).Distance(to_ll(locations.Get(j)));
      total_path_distance += i + 1 == j ? dist : 0;
      if ((!all_pairs && total_path_distance > max_distance) || (all_pairs && dist > max_distance))
        throw valhalla_exception_t{154,
                                   std::to_string(static_cast<size_t>(max_distance)) + " meters"};
      if (!all_pairs)
        break;
    }
  }
}

} // namespace

namespace valhalla {
namespace loki {

void loki_worker_t::init_route(Api& request) {
  parse_locations(request.mutable_options()->mutable_locations(), request);
  // need to check location size here instead of in parse_locations because of locate action needing
  // a different size
  if (request.options().locations_size() < 2) {
    throw valhalla_exception_t{120};
  };
  parse_costing(request);
}

void loki_worker_t::route(Api& request) {
  // time this whole method and save that statistic
  auto _ = measure_scope_time(request);

  init_route(request);
  auto& options = *request.mutable_options();
  const auto& costing_name = Costing_Enum_Name(options.costing_type());
  if (request.options().action() == Options::centroid) {
    check_locations(options.locations_size(), max_locations.find("centroid")->second);
    check_distance(options.locations(), max_distance.find("centroid")->second, true);
  } else {
    check_locations(options.locations_size(), max_locations.find(costing_name)->second);
    check_distance(options.locations(), max_distance.find(costing_name)->second, false);
  }

  // check distance for hierarchy pruning
  check_hierarchy_distance(request);

  auto connectivity_level = TileHierarchy::levels().back();
  uint32_t connectivity_radius = 0;
  // Validate walking distances (make sure they are in the accepted range)
  if (costing_name == "multimodal" || costing_name == "transit" || costing_name == "auto_walk") {
    auto& ped_opts = *options.mutable_costings()->find(Costing::pedestrian)->second.mutable_options();
    if (!ped_opts.has_transit_start_end_max_distance_case())
      ped_opts.set_transit_start_end_max_distance(min_transit_walking_dis);
    auto transit_start_end_max_distance = ped_opts.transit_start_end_max_distance();

    if (!ped_opts.has_transit_transfer_max_distance_case())
      ped_opts.set_transit_transfer_max_distance(min_transit_walking_dis);
    auto transit_transfer_max_distance = ped_opts.transit_transfer_max_distance();

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
    if (costing_name != "auto_walk") {
      connectivity_level = TileHierarchy::GetTransitLevel();
    }
    connectivity_radius = ped_opts.transit_start_end_max_distance();
  }

  // correlate the various locations to the underlying graph
  std::unordered_map<size_t, size_t> color_counts;
  try {
    auto locations = PathLocation::fromPBF(options.locations(), true);
    size_t locations_end = locations.size();

    // maybe squeeze in the first and last locations of each user specified feature for cost factor
    // lines as we'll need those for edge walking
    for (const auto& line : options.cost_factor_lines()) {
      locations.push_back(PathLocation::fromPBF(line.shape(0)));
      locations.back().min_inbound_reach_ = 0;
      locations.push_back(PathLocation::fromPBF(line.shape(line.shape_size() - 1)));
      locations.back().min_inbound_reach_ = 0;
    }

    std::unordered_map<baldr::Location, PathLocation> projections;

    // in case of auto_walk costing, we 1) only allow two locations
    // and 2) need two different costings for the start and end location.
    // Search::search does not allow for multiple costings per location so instead
    // we bite the bullet and call search twice, merging the results.
    // TODO(chris): right now we hash location based purely on ll, but what if the user wants to start
    // and end at the same location but with different costing? What does that even mean? Find the
    // nearest parking and walk back to where I am? Seems like a plausible use case...
    if (costing_name == "auto_walk") {
      std::vector<baldr::Location> start_loc(locations.begin(), locations.begin() + 1);
      auto start_projection = search_.search(start_loc, costing);
      for (const auto& [loc, path_loc] : start_projection) {
        projections.insert({loc, path_loc});
      }
      std::vector<baldr::Location> end_loc(locations.end() - 1, locations.end());
      auto end_projection = search_.search(end_loc, costing);
      for (const auto& [loc, path_loc] : end_projection) {
        projections.insert({loc, path_loc});
      }
    } else {
      projections = search_.search(locations, costing);
    }
    for (size_t i = 0; i < locations_end; ++i) {
      const auto& correlated = projections.at(locations[i]);
      PathLocation::toPBF(correlated, options.mutable_locations(i), *reader);
      if (!connectivity_map) {
        continue;
      }
      auto colors = connectivity_map->get_colors(connectivity_level, correlated, connectivity_radius);
      for (auto color : colors) {
        auto itr = color_counts.find(color);
        if (itr == color_counts.cend()) {
          color_counts[color] = 1;
        } else {
          ++itr->second;
        }
      }
    }

    // store the correlations for the cost factor lines
    size_t i = 0;
    for (auto& line : *options.mutable_cost_factor_lines()) {
      const auto& correlated_start = projections.at(locations[locations_end + 2 * i]);
      auto* start = line.mutable_locations()->Add();
      PathLocation::toPBF(correlated_start, start, *reader);
      const auto& correlated_end = projections.at(locations[locations_end + 2 * i + 1]);
      auto* end = line.mutable_locations()->Add();
      PathLocation::toPBF(correlated_end, end, *reader);
      ++i;
    }
  } catch (const valhalla_exception_t& e) { throw e; } catch (const std::exception&) {
    throw valhalla_exception_t{171};
  }

  // are all the locations in the same color regions
  if (!connectivity_map) {
    return;
  }
  bool connected = false;
  for (const auto& c : color_counts) {
    if (c.second == static_cast<size_t>(options.locations_size())) {
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
