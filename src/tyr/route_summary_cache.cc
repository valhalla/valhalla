#include "proto/directions.pb.h"
#include "proto/options.pb.h"
#include "proto/trip.pb.h"
#include "proto_conversions.h"

#include "./route_summary_cache.h"

namespace valhalla {
namespace tyr {

NamedSegment& NamedSegment::operator=(const NamedSegment& ns) {
  name = ns.name;
  index = ns.index;
  distance = ns.distance;
  return *this;
}

NamedSegment& NamedSegment::operator=(NamedSegment&& ns) noexcept {
  name = std::move(ns.name);
  index = ns.index;
  distance = ns.distance;
  return *this;
}

route_summary_cache::route_summary_cache(
    const google::protobuf::RepeatedPtrField<DirectionsRoute>& routes) {
  // A route/leg/maneuver may go on/off the same named road many times.
  // Combine the distances for same named roads - store in a NamedSegment.
  route_leg_segs_by_dist.reserve(routes.size());
  for (const auto& route : routes) {
    std::vector<std::vector<NamedSegment>> leg_segs_by_dist;
    leg_segs_by_dist.reserve(route.legs_size());
    for (int j = 0; j < route.legs_size(); j++) {
      const DirectionsLeg& leg = route.legs(j);
      std::unordered_map<std::string, std::pair<uint32_t, float>> maneuver_summary_map;
      maneuver_summary_map.reserve(leg.maneuver_size());
      uint32_t maneuver_index = 0;
      for (const auto& maneuver : leg.maneuver()) {
        if (maneuver.street_name_size() > 0) {
          const std::string& name = maneuver.street_name(0).value();
          auto maneuver_summary = maneuver_summary_map.find(name);
          if (maneuver_summary == maneuver_summary_map.end()) {
            maneuver_summary_map[name] = std::make_pair(maneuver_index, maneuver.length());
          } else {
            maneuver_summary->second.second += maneuver.length();
          }
        }
        ++maneuver_index;
      }

      // Create a list of named segments (maneuver name, maneuver index, distance)
      // sorted by distance.
      std::vector<NamedSegment> segs_by_dist;
      segs_by_dist.reserve(maneuver_summary_map.size());
      for (const auto& map_item : maneuver_summary_map) {
        segs_by_dist.emplace_back(map_item.first, map_item.second.first, map_item.second.second);
      }

      // Sort list by descending maneuver distance
      std::sort(segs_by_dist.begin(), segs_by_dist.end(),
                [](const NamedSegment& a, const NamedSegment& b) { return b.distance < a.distance; });

      leg_segs_by_dist.emplace_back(std::move(segs_by_dist));
    }

    route_leg_segs_by_dist.emplace_back(std::move(leg_segs_by_dist));
  }

  // fill the cache with empty results so we can safely index it with the same
  // dimensionality as the route_leg_segs_by_dist object.
  cache.reserve(route_leg_segs_by_dist.size());
  for (size_t i = 0; i < route_leg_segs_by_dist.size(); i++) {
    cache.emplace_back();
    cache[i].reserve(route_leg_segs_by_dist[i].size());
    for (size_t j = 0; j < route_leg_segs_by_dist[i].size(); j++) {
      cache[i].emplace_back();
      cache[i][j].reserve(route_leg_segs_by_dist[i][j].size());
      for (size_t k = 0; k < route_leg_segs_by_dist[i][j].size(); k++) {
        cache[i][j].emplace_back();
      }
    }
  }
}

size_t route_summary_cache::num_named_segments_for_route_leg(size_t route_idx, size_t leg_idx) {
  if (route_idx >= route_leg_segs_by_dist.size()) {
    return 0;
  }

  if (leg_idx >= route_leg_segs_by_dist[route_idx].size()) {
    return 0;
  }

  return route_leg_segs_by_dist[route_idx][leg_idx].size();
}

// Return the n-part named-segment summary for the given route/leg.
// The summary returned is guaranteed to be comprised of as few named
// segments as possible, while also being unique among all route/same-leg
// summaries.
std::string
route_summary_cache::get_n_segment_summary(size_t route_idx, size_t leg_idx, size_t num_named_segs) {
  if (route_idx >= cache.size()) {
    return "";
  }

  if (leg_idx >= cache[route_idx].size()) {
    return "";
  }

  if (num_named_segs == 0) {
    return "";
  }

  // num_named_segs is the number of named segments you'd like the summary to
  // be comprised of. The smallest requestable value is 1 - so we store all
  // summaries offset by n = num_named_segs - 1.
  size_t n = num_named_segs - 1;
  if (n >= cache[route_idx][leg_idx].size()) {
    return "";
  }

  // empty summary means cache miss
  if (cache[route_idx][leg_idx][n].empty()) {
    // go figure out the summary
    std::vector<const NamedSegment*> segs_by_maneuver_index;
    segs_by_maneuver_index.reserve(num_named_segs);
    for (size_t i = 0; i < num_named_segs; i++) {
      segs_by_maneuver_index.emplace_back(&route_leg_segs_by_dist[route_idx][leg_idx][i]);
    }

    // sort by maneuver index
    std::sort(segs_by_maneuver_index.begin(), segs_by_maneuver_index.end(),
              [](const NamedSegment* a, const NamedSegment* b) { return a->index < b->index; });

    std::string summary;
    for (size_t i = 0; i < num_named_segs; i++) {
      summary += segs_by_maneuver_index[i]->name;
      if (i != num_named_segs - 1)
        summary += ", ";
    }

    misses++;

    cache[route_idx][leg_idx][n] = std::move(summary);
  } else {
    hits++;
  }

  return cache[route_idx][leg_idx][n];
}

} // namespace tyr
} // namespace valhalla
