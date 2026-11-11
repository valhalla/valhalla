
#include "loki/search.h"
#include "loki/worker.h"
#include "midgard/util.h"

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
                                              double radius_lo,
                                              double radius_hi,
                                              size_t num_groups,
                                              size_t points_per_group) {

  assert(num_groups > 0);
  assert(points_per_group > 0);
  assert(0. <= radius_lo && radius_lo <= radius_hi);

  PointLL startll{start.ll().lng(), start.ll().lat()};
  midgard::AzimuthalEquidistant aed(startll);
  Point2d start2d = aed.project(startll);

  auto heading_lo = heading - heading_tolerance;
  auto heading_hi = heading + heading_tolerance;

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

  constexpr double kDefaultOverlapFactor = 1.;
  constexpr double kMaxOverlapFactor = 100.;

  if (!options->has_overlap_factor()) {
    options->set_overlap_factor(kDefaultOverlapFactor);
  } else if (options->overlap_factor() < 0. || options->overlap_factor() > kMaxOverlapFactor) {
    options->set_overlap_factor(std::clamp(options->overlap_factor(), 0., kMaxOverlapFactor));
    add_warning(request, 306, std::format("[{:.1f},{:.1f}]", 0., kMaxOverlapFactor));
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

/**
 * Decide how many points are seeded for each group. More points lead to better results but also mean
 * more matrix pairs need to be calculated. On longer distances, this immensely slows down the matrix
 * calculations, so lower the point count at higher distances.
 *
 * @param  distance the desired distance in meters provided in the request options
 * @return the number of points per group to randomly seed
 */
size_t points_per_group(double distance) {
  constexpr double kShortRoundTrip = 4000.;
  constexpr double kLongRoundTrip = 15000.;
  constexpr size_t kShortPoints = 40;
  constexpr size_t kLongPoints = 16;

  // 1 at or below kShortRoundTrip, 0 at or above kLongRoundTrip, linear in between
  const double t =
      std::clamp((kLongRoundTrip - distance) / (kLongRoundTrip - kShortRoundTrip), 0., 1.);
  return kLongPoints +
         static_cast<size_t>(std::lround(t * static_cast<double>(kShortPoints - kLongPoints)));
}
} // namespace

namespace valhalla {
namespace loki {
void loki_worker_t::round_trip(Api& request) {

  // time this whole method and save that statistic
  auto _ = measure_scope_time(request);

  parse_costing(request);

  // only allow a single location
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
    LOG_ERROR("{}", e.what());
    throw valhalla_exception_t{100};
  }

  auto& round_trip_options = options.round_trip_options();

  // choose the number of seed groups based on how wide the round trip should be
  size_t num_groups = round_trip_options.heading_tolerance() < 20 ? 2 : 3;

  constexpr double kMinRoadNetworkFactor = 1.2;
  constexpr double kMaxRoadNetworkFactor = 2.0;
  const auto radius_lo =
      estimate_radius(round_trip_options.distance(), round_trip_options.heading_tolerance(),
                      num_groups, kMaxRoadNetworkFactor);
  const auto radius_hi =
      estimate_radius(round_trip_options.distance(), round_trip_options.heading_tolerance(),
                      num_groups, kMinRoadNetworkFactor);

  auto seeded_points = seed_points(options.locations(0), round_trip_options.heading(),
                                   round_trip_options.heading_tolerance(), radius_lo, radius_hi,
                                   num_groups, points_per_group(round_trip_options.distance()));

  // the search computes reach up to the highest minimum reachability of all locations it's given, so
  // the seeds are searched separately from the start
  google::protobuf::RepeatedPtrField<Location> seeds;
  for (size_t i = 0; i < seeded_points.size(); ++i) {
    const auto& seed_group = seeded_points[i];
    for (const auto& pt : seed_group) {
      auto l = seeds.Add();
      l->mutable_ll()->set_lng(pt.lng());
      l->mutable_ll()->set_lat(pt.lat());
      // downstream needs to know what group a seed location belongs to
      // 0 means it's the start location
      l->set_round_trip_group_index(i + 1);
    }
  }

  // a seed snapping to a road far outside the annular sector creates routes that are too long, so
  // only search near the ring
  constexpr double kMinSeedSearchCutoff = 200.;
  const auto seed_search_cutoff =
      static_cast<uint32_t>(std::max(kMinSeedSearchCutoff, (radius_hi - radius_lo) / 2.));
  // seeds are driven through, so keep them off roads that lead nowhere or that nobody wants a round
  // trip to pass through
  constexpr uint64_t kSeedExcludeUses = [] {
    uint64_t mask = 0;
    for (auto use :
         {Use::kRamp, Use::kTurnChannel, Use::kDriveway, Use::kAlley, Use::kParkingAisle,
          Use::kEmergencyAccess, Use::kDriveThru, Use::kCuldesac, Use::kServiceRoad, Use::kRestArea,
          Use::kServiceArea, Use::kSteps, Use::kElevator, Use::kEscalator, Use::kLadder,
          Use::kPlatform, Use::kFerry, Use::kRailFerry, Use::kConstruction}) {
      mask |= uint64_t{1} << static_cast<uint8_t>(use);
    }
    return mask;
  }();
  // motor vehicle round trips should run along proper roads, and a seed on a motorway can't be passed
  // through without a long detour to the next exit. Unclassified roads are allowed but only used
  // after all other seeds, see below
  const bool is_motor_vehicle = mode == sif::TravelMode::kDrive;
  for (auto it = seeds.begin(); it != seeds.end(); ++it) {
    auto* filter = it->mutable_search_filter();
    filter->set_exclude_uses(kSeedExcludeUses);
    filter->set_exclude_dead_ends(true);
    filter->set_exclude_not_thru(true);
    if (is_motor_vehicle) {
      filter->set_min_road_class(valhalla::RoadClass::kUnclassified);
      filter->set_max_road_class(valhalla::RoadClass::kTrunk);
    }
    it->set_node_snap_tolerance(0);
    it->set_minimum_reachability(
        0); // reach is expensive, so we don't calculate it for the seeded locations and instead rely
            // on the scoring of permutations in thor to filter out unfound matrix pairs
    it->set_search_cutoff(seed_search_cutoff);
    it->set_radius(80);
  }
  auto* locations = options.mutable_locations();
  parse_locations(locations, request, valhalla_exception_t{112});
  parse_locations(&seeds, request);

  try {
    search_.search(*locations, mode_costing[static_cast<size_t>(mode)]);
    search_.clear();
    search_.search(seeds, mode_costing[static_cast<size_t>(mode)]);
    for (auto& seed : seeds) {
      locations->Add(std::move(seed));
    }
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

  const PointLL start_ll(locations->at(0).ll().lng(), locations->at(0).ll().lat());
  for (auto it = locations->begin() + 1; it != locations->end();) {
    auto* correlation = it->mutable_correlation();
    // if we have nothing but filtered edges, promote them
    if (correlation->edges_size() == 0) {
      correlation->mutable_edges()->Swap(correlation->mutable_filtered_edges());
    }
    // the search filter only looks at the edge itself, but a seed edge is also arrived on, so its
    // opposing edge mustn't lead into a dead end or a region without exit either. Links aren't all
    // marked with a ramp or turn channel use, so drop them by their link flag as well. Internal
    // intersection edges and roundabouts are junctions rather than roads to pass through
    auto* candidate_edges = correlation->mutable_edges();
    for (int i = candidate_edges->size() - 1; i >= 0; --i) {
      const GraphId id(candidate_edges->Get(i).graph_id());
      graph_tile_ptr tile;
      const auto* edge = reader->directededge(id, tile);
      graph_tile_ptr opp_tile;
      const auto* opp = reader->GetOpposingEdge(id, opp_tile);
      if (!edge || edge->link() || edge->internal() || edge->roundabout() || !opp || opp->deadend() ||
          opp->not_thru()) {
        candidate_edges->DeleteSubrange(i, 1);
      }
    }
    // drop seeds without usable edges
    if (correlation->edges_size() == 0) {
      it = locations->erase(it);
      continue;
    }
    // keep a single directed edge, so the leg arriving at a seed and the leg leaving it drive that
    // edge in the same direction. Otherwise the loop may turn around at the seed, which the stitched
    // shape can't express. The groups go clockwise around the start, so at a seed the loop travels
    // perpendicular to the start, clockwise. Keep the edge closest to that heading
    const double loop_heading = start_ll.Heading(PointLL(it->ll().lng(), it->ll().lat())) + 90.;
    const auto heading_diff = [loop_heading](const PathEdge& edge) {
      const double d = std::fmod(std::abs(edge.heading() - loop_heading), 360.);
      return d > 180. ? 360. - d : d;
    };
    auto* edges = correlation->mutable_edges();
    auto best =
        std::min_element(edges->begin(), edges->end(), [&](const PathEdge& a, const PathEdge& b) {
          return heading_diff(a) < heading_diff(b);
        });
    edges->SwapElements(0, static_cast<int>(best - edges->begin()));
    edges->DeleteSubrange(1, edges->size() - 1);

    // with a search radius most kept edges are farther ones, where the seed projects onto either end
    // of the edge and acts like a node snap. Pass every seed through the middle of its edge instead,
    // and since the seed is only a rough position anyway, don't let thor charge for the distance to
    // it
    auto& seed_edge = *edges->Mutable(0);
    graph_tile_ptr tile;
    const auto* edge = reader->directededge(GraphId(seed_edge.graph_id()), tile);
    // edgeinfo() returns by value and shape() refers into it, so keep the EdgeInfo alive
    const auto edge_info = tile->edgeinfo(edge);
    const auto& shape = edge_info.shape();
    const auto middle = trim_polyline(shape.begin(), shape.end(), 0.5, 0.5);
    if (!middle.empty()) {
      seed_edge.set_percent_along(0.5);
      seed_edge.mutable_ll()->set_lng(middle.front().lng());
      seed_edge.mutable_ll()->set_lat(middle.front().lat());
    }
    seed_edge.set_distance(0);
    ++it;
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
  // thor takes the seeds of each group in order, so seeds on unclassified roads go last and are only
  // used if the others don't make a good enough round trip
  if (is_motor_vehicle) {
    std::stable_partition(locations->pointer_begin() + 1, locations->pointer_end(),
                          [&](const Location* l) {
                            graph_tile_ptr tile;
                            const auto* edge =
                                reader->directededge(GraphId(l->correlation().edges(0).graph_id()),
                                                     tile);
                            return edge && edge->classification() != baldr::RoadClass::kUnclassified;
                          });
  }
  // thor finds the start and the seeds in locations and sets up its matrices from there
}
} // namespace loki
} // namespace valhalla
