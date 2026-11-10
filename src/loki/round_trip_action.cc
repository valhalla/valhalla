
#include "loki/search.h"
#include "loki/worker.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <format>
#include <random>
#include <unordered_set>

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::loki;

namespace {

std::uint64_t mix(std::uint64_t x) {
  x += 0x9e3779b97f4a7c15ULL;
  x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
  x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
  return x ^ (x >> 31);
}

std::uint64_t round_trip_seed(const PointLL& ll) {
  const auto lat = static_cast<std::int64_t>(std::llround(ll.lat() * 1e6));
  const auto lng = static_cast<std::int64_t>(std::llround(ll.lng() * 1e6));
  return mix(static_cast<std::uint64_t>(lat) ^ mix(static_cast<std::uint64_t>(lng)));
}

std::vector<std::vector<PointLL>> seed_points(const Location& start,
                                              double heading,
                                              double heading_tolerance,
                                              double radius_meters,
                                              double radius_tolerance_meters,
                                              size_t num_groups,
                                              size_t points_per_group) {

  assert(num_groups > 0);
  assert(points_per_group > 0);

  PointLL startll{start.ll().lng(), start.ll().lat()};
  midgard::AzimuthalEquidistant aed(startll);
  Point2d start2d = aed.project(startll);

  auto heading_lo = heading - heading_tolerance;
  auto heading_hi = heading + heading_tolerance;

  radius_tolerance_meters = std::clamp(radius_tolerance_meters, 0.0, radius_meters);
  auto radius_lo = radius_meters - radius_tolerance_meters;
  auto radius_hi = radius_meters + radius_tolerance_meters;

  std::mt19937_64 engine{round_trip_seed(startll)};
  std::uniform_real_distribution radius_distribution(midgard::sqr(radius_lo),
                                                     midgard::sqr(radius_hi));

  std::vector<std::vector<PointLL>> seeded_points;
  seeded_points.resize(num_groups);

  auto step = (heading_hi - heading_lo) / static_cast<double>(num_groups);
  for (size_t i = 0; i < num_groups; ++i) {
    auto lo = heading_lo + i * step;
    auto hi = lo + step;
    std::uniform_real_distribution heading_distribution(lo, hi);
    seeded_points[i].resize(points_per_group);
    for (size_t j = 0; j < points_per_group; ++j) {
      // generate a random point in annular sector
      seeded_points[i][j] =
          aed.project_inverse(start2d.PointAlongHeading(heading_distribution(engine),
                                                        std::sqrt(radius_distribution(engine))));
    }
  }

  return seeded_points;
}

void parse_round_trip_options(Api& request, double max_distance) {
  auto* options = request.mutable_options()->mutable_round_trip_options();
  request.mutable_options()->mutable_locations(0)->set_round_trip_group_index(
      0); // overwrite if the user tried to set it

  // nothing or negative value set, throw
  if (options->distance() <= 0.0) {
    throw valhalla_exception_t{178};
  }

  // clamp distance
  if (options->distance() > max_distance) {
    options->set_distance(max_distance);
    add_warning(request, 304, std::format("{:.1f}", max_distance));
  }

  if (!options->has_heading()) {
    options->set_heading(0.0); // todo: randomly using seed
  } else if (options->heading() >= 360. || options->heading() < 0.0) {
    auto wrapped = std::fmod(options->heading(), 360.0);
    options->set_heading(wrapped < 0.0 ? wrapped + 360.0 : wrapped);
  }

  constexpr double kMinHeadingTolerance = 10.;
  constexpr double kMaxHeadingTolerance = 180.;

  if (!options->has_heading_tolerance()) {
    options->set_heading_tolerance(45.0);
  } else if (options->heading_tolerance() < kMinHeadingTolerance ||
             options->heading_tolerance() > kMaxHeadingTolerance) {
    options->set_heading_tolerance(
        std::clamp(options->heading_tolerance(), kMinHeadingTolerance, kMaxHeadingTolerance));
    add_warning(request, 305,
                std::format("[{:.1f},{:.1f}]", kMinHeadingTolerance, kMaxHeadingTolerance));
  }
}

/**
 * Estimate the radius to use in seeding intermediate round trip points.
 *
 * @param distance             desired round trip distance in meters
 * @param heading_tolerance    heading tolerance in degrees
 * @param num_groups           number of seed point groups
 * @param road_network_factor  factor to account for difference between beeline and network distance
 */
double estimate_radius(double distance,
                       double heading_tolerance,
                       size_t num_groups,
                       double road_network_factor) {

  return distance /
         (road_network_factor *
          (2 + 2 * (num_groups - 1) * std::sin((heading_tolerance / num_groups) * kRadPerDegD)));
}
} // namespace

namespace valhalla {
namespace loki {
void loki_worker_t::round_trip(Api& request) {

  // time this whole method and save that statistic
  auto _ = measure_scope_time(request);

  parse_costing(request);

  // only allow a single location for now
  if (request.options().locations_size() == 0) {
    throw valhalla_exception_t{120};
  } else if (request.options().locations_size() > 1) {
    throw valhalla_exception_t{150, "1"};
  }

  auto& options = *request.mutable_options();
  const auto& costing_name = Costing_Enum_Name(options.costing_type());
  auto it = max_roundtrip_distances.find(costing_name);
  if (it == max_roundtrip_distances.end()) {
    throw valhalla_exception_t{125};
  }
  try {
    parse_round_trip_options(request, max_roundtrip_distances.at(costing_name));
  } catch (const valhalla_exception_t&) { throw; } catch (const std::exception& e) {
    throw valhalla_exception_t{100, e.what()};
  }

  auto& round_trip_options = options.round_trip_options();
  size_t num_groups = round_trip_options.heading_tolerance() < 15   ? 1
                      : round_trip_options.heading_tolerance() < 45 ? 2
                                                                    : 3;

  constexpr size_t kLocationsPerGroup = 3;
  constexpr double kRoadNetworkFactor = 1.4;
  auto radius = estimate_radius(round_trip_options.distance(), round_trip_options.heading_tolerance(),
                                num_groups, kRoadNetworkFactor);

  auto radius_tolerance = radius * 0.15; // TODO: better estimate

  auto seeded_points = seed_points(options.locations(0), round_trip_options.heading(),
                                   round_trip_options.heading_tolerance(), radius, radius_tolerance,
                                   num_groups, kLocationsPerGroup);

  auto* locations = options.mutable_locations();
  for (size_t i = 0; i < seeded_points.size(); ++i) {
    const auto& seed_group = seeded_points[i];
    for (const auto& pt : seed_group) {
      auto l = locations->Add();
      l->mutable_ll()->set_lng(pt.lng());
      l->mutable_ll()->set_lat(pt.lat());
      l->set_round_trip_group_index(i + 1);
    }
  }
  parse_locations(request.mutable_options()->mutable_locations(), request, valhalla_exception_t{112});

  try {
    search_.search(*options.mutable_locations(), mode_costing[static_cast<size_t>(mode)]);
    // TODO: what do we do with uncorrelated seed locations?
    // the sensible thing would be to regenerate points and try again for those
    // for now we just drop them down below
  } catch (const valhalla_exception_t&) { throw; } catch (const std::exception& e) {
    LOG_ERROR(e.what());
    throw valhalla_exception_t{171};
  }

  // if the start location didn't have anything, throw
  if (locations->at(0).correlation().edges_size() == 0 &&
      locations->at(0).correlation().filtered_edges_size() == 0) {
    throw valhalla_exception_t{171};
  } else if (locations->at(0).correlation().edges_size() == 0) {
    // if it only had filtered edges, promote those
    locations->at(0).mutable_correlation()->mutable_edges()->Swap(
        locations->at(0).mutable_correlation()->mutable_filtered_edges());
  }

  // drop uncorrelated seed points
  for (auto it = locations->begin() + 1; it != locations->end();) {
    auto& l = *it;
    if (l.correlation().edges_size() == 0 && l.correlation().filtered_edges_size() == 0) {
      it = locations->erase(it);
    } else {
      // if we have nothing but filtered edges, promote them
      if (l.correlation().edges_size() == 0) {
        l.mutable_correlation()->mutable_edges()->Swap(
            l.mutable_correlation()->mutable_filtered_edges());
      }
      ++it;
    }
  }

  // do all the seeded locations share at least one colored region with the start location
  if (connectivity_map) {
    auto start_colors =
        connectivity_map->get_colors(TileHierarchy::levels().back(), options.locations(0), 0);

    // go through the seeded locations
    for (auto it = locations->begin() + 1; it != locations->end();) {
      const auto& l = *it;
      auto colors = connectivity_map->get_colors(TileHierarchy::levels().back(), l, 0);
      bool found_common_color = false;
      for (auto& color : colors) {
        auto itr = start_colors.find(color);
        if (itr != start_colors.cend()) {
          found_common_color = true;
          break;
        }
      }

      if (!found_common_color) {
        it = locations->erase(it);
      } else {
        ++it;
      }
    }
  }

  // now that we have pre-filtered locations with empty correlations, we can see what's left

  // if we're back to 1, throw
  if (locations->size() == 1) {
    LOG_DEBUG("All round trip seed points were removed");
    throw valhalla_exception_t{171};
  }

  // or if a seed group came out empty
  std::unordered_set<size_t> groups_present;
  for (auto it = locations->begin() + 1; it != locations->end(); ++it) {
    groups_present.insert(it->round_trip_group_index());
  }

  for (size_t i = 1; i <= num_groups; ++i) {
    if (!groups_present.contains(i)) {
      LOG_DEBUG("All round trip seed points from group {} were removed", i);
      throw valhalla_exception_t{171};
    }
  }

  // finally, move all locations into sources and targets
  // copy the start and keep it under locations
  options.mutable_sources()->Add()->CopyFrom(*locations->begin());
  options.mutable_targets()->Add()->CopyFrom(*locations->begin());
  for (auto it = locations->begin() + 1; it != locations->end(); ++it) {
    options.mutable_sources()->Add()->CopyFrom(*it);
    options.mutable_targets()->Add(std::move(*it));
  }
  locations->DeleteSubrange(1, locations->size() - 1);
}
} // namespace loki
} // namespace valhalla
