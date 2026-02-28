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
  if (costing_name == "multimodal" || costing_name == "transit" ||
      costing_name == "auto_pedestrian") {
    auto& ped_opts = *options.mutable_costings()->find(Costing::pedestrian)->second.mutable_options();

    // "transit_start_end_max_distance" is deprecated
    // but we will still allow it for some time
    if (ped_opts.has_transit_transfer_max_distance_case()) {
      ped_opts.set_multimodal_start_end_max_distance(ped_opts.transit_start_end_max_distance());
    }

    // we have renamed this parameter
    if (!ped_opts.has_multimodal_start_end_max_distance_case())
      ped_opts.set_multimodal_start_end_max_distance(min_multimodal_walking_dist);
    auto multimodal_start_end_max_distance = ped_opts.multimodal_start_end_max_distance();

    if (!ped_opts.has_transit_transfer_max_distance_case())
      ped_opts.set_transit_transfer_max_distance(min_multimodal_walking_dist);
    auto transit_transfer_max_distance = ped_opts.transit_transfer_max_distance();

    if (multimodal_start_end_max_distance < min_multimodal_walking_dist ||
        multimodal_start_end_max_distance > max_multimodal_walking_dist) {
      throw valhalla_exception_t{155, " Min: " + std::to_string(min_multimodal_walking_dist) +
                                          " Max: " + std::to_string(max_multimodal_walking_dist) +
                                          " (Meters)"};
    }
    if (transit_transfer_max_distance < min_multimodal_walking_dist ||
        transit_transfer_max_distance > max_multimodal_walking_dist) {
      throw valhalla_exception_t{156, " Min: " + std::to_string(min_multimodal_walking_dist) +
                                          " Max: " + std::to_string(max_multimodal_walking_dist) +
                                          " (Meters)"};
    }
    if (costing_name != "auto_pedestrian") {
      connectivity_level = TileHierarchy::GetTransitLevel();
    }
    connectivity_radius = ped_opts.transit_start_end_max_distance();
  }

  // correlate the various locations to the underlying graph
  std::unordered_map<size_t, size_t> color_counts;
  try {
    auto* locations = options.mutable_locations();
    auto locations_end = locations->end();

    // maybe squeeze in the first and last locations of each user specified feature for cost factor
    // lines as we'll need those for edge walking
    for (const auto& line : options.cost_factor_lines()) {
      Location first_shape_loc;
      first_shape_loc.mutable_ll()->set_lat(line.shape().begin()->ll().lat());
      first_shape_loc.mutable_ll()->set_lng(line.shape().begin()->ll().lng());
      first_shape_loc.set_minimum_inbound_reachability(0);
      locations->Add(std::move(first_shape_loc));
      Location last_shape_loc;
      last_shape_loc.mutable_ll()->set_lat(line.shape().rbegin()->ll().lat());
      last_shape_loc.mutable_ll()->set_lng(line.shape().rbegin()->ll().lng());
      last_shape_loc.set_minimum_inbound_reachability(0);
      locations->Add(std::move(last_shape_loc));
    }

    // in case of auto_pedestrian costing, we 1) only allow two locations
    // and 2) need two different costings for the start and end location.
    // Search::search does not allow for multiple costings per location so instead
    // we bite the bullet and call search twice, merging the results.
    // TODO(chris): right now we hash location based purely on ll, but what if the user wants to start
    // and end at the same location but with different costing? What does that even mean? Find the
    // nearest parking and walk back to where I am? Seems like a plausible use case...
    if (costing_name == "auto_pedestrian") {
      if (locations->size() > 2) {
        throw valhalla_exception_t{150, "for auto_pedestrian: " + std::to_string(locations->size())};
      }
      google::protobuf::RepeatedPtrField<Location> start_loc(locations->begin(),
                                                             locations->begin() + 1);
      search_.search(start_loc, costing);
      google::protobuf::RepeatedPtrField<Location> end_loc(locations->end() - 1, locations->end());
      search_.search(end_loc, costing);
      // merge them again
      locations->Swap(&start_loc);
      locations->MergeFrom(end_loc);
    } else {
      search_.search(*locations, costing);
    }
    for (auto it = locations->begin(); it != locations_end; ++it) {
      if (!connectivity_map) {
        continue;
      }
      auto colors = connectivity_map->get_colors(connectivity_level, *it, connectivity_radius);
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
    // todo(chris): make sure this'll work with auto_pedestrian as well
    size_t i = 0;
    for (auto& line : *options.mutable_cost_factor_lines()) {
      const auto& correlated_start = locations_end + 2 * i;
      line.mutable_locations()->Add(std::move(*correlated_start));
      const auto& correlated_end = locations_end + 2 * i + 1;
      line.mutable_locations()->Add(std::move(*correlated_start));
      ++i;
    }
    // and remove the first and last cost factor lines from the locations again
    locations->erase(locations_end, locations->end());
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
