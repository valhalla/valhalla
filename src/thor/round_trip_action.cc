#include "baldr/graphreader.h"
#include "midgard/encoded.h"
#include "thor/costmatrix.h"
#include "thor/worker.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_set>
#include <vector>

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace {

// weight for the candidate score, distance deviation has an implicit weight of 1 and overlap is
// weighted by the requested round_trip_options.overlap_factor

// round trip candidates are weighted on distance deviation (implicit weight of 1), cost per distance,
// and overlap
constexpr double kCostPerDistanceWeight = 0.25;

// candidates retracing more than this fraction of their length are only used if nothing else is
// left
constexpr double kMaxOverlap = 0.5;

// the best candidate is taken once it is this close to the requested distance and overlaps this
// little
constexpr double kAcceptableDistanceDeviation = 0.1;
constexpr double kAcceptableOverlap = 0.2;

// seeds per group in the first round, each further round doubles it
constexpr size_t kInitialSeedsPerGroup = 3;

// a single leg candidate round trips are made up of
struct LegInfo {
  bool computed = false;

  // edges and their opposing edges, i.e. every road this leg touches in either direction
  // TODO: this uses a narrow definition of "every edge along a stretch of road" because it leaves out
  // edges from the opposing part of a doubly digitized way, as well as sidewalks and adjacent cycle
  // paths mapped separately
  std::vector<GraphId> roads;

  // edges excluding the first and last one, which are shared with the adjacent legs by design
  // and their length
  std::vector<std::pair<GraphId, float>> interior;
};

// matrix results between consecutive groups, row-major by source over all
// correlated locations. Connections that no round trip uses are not filled
struct connections {
  explicit connections(uint32_t num_locations)
      : num_locations(num_locations), distances(num_locations * num_locations),
        costs(num_locations * num_locations), shapes(num_locations * num_locations),
        paths(num_locations * num_locations) {
  }
  uint32_t index(uint32_t source, uint32_t target) const {
    return source * num_locations + target;
  }

  uint32_t num_locations;
  std::vector<float> distances;
  std::vector<float> costs;
  std::vector<std::string> shapes;
  std::vector<std::vector<GraphId>> paths;
};

// A complete round trip candidate
struct candidate {
  std::vector<uint32_t> connections; // matrix connection index per leg, in driving order
  double distance = 0.;
  double cost = 0.;
  double overlap = 0.; // fraction of the distance that is driven on a road used by an earlier leg
  double score = 0.;
};

const LegInfo& get_leg_info(std::vector<LegInfo>& cache,
                            const std::vector<std::vector<GraphId>>& paths,
                            uint32_t connection,
                            GraphReader& reader) {
  auto& info = cache[connection];
  if (info.computed) {
    return info;
  }
  info.computed = true;

  const auto& path = paths[connection];
  graph_tile_ptr tile;
  info.roads.reserve(path.size() * 2);
  for (size_t i = 0; i < path.size(); ++i) {
    const auto* de = reader.directededge(path[i], tile);
    if (!de) {
      continue;
    }
    info.roads.push_back(path[i]);
    auto opp = reader.GetOpposingEdgeId(path[i], tile);
    if (opp.is_valid()) {
      info.roads.push_back(opp);
    }
    if (i != 0 && i + 1 != path.size()) {
      info.interior.emplace_back(path[i], static_cast<float>(de->length()));
    }
  }
  return info;
}

// fraction of the candidate's interior length that runs on roads an earlier leg already used
double compute_overlap(const candidate& candidate,
                       std::vector<LegInfo>& cache,
                       const std::vector<std::vector<GraphId>>& paths,
                       GraphReader& reader) {
  std::unordered_set<GraphId> seen;
  double overlap = 0.;
  double total = 0.;
  for (auto connection : candidate.connections) {
    const auto& leg = get_leg_info(cache, paths, connection, reader);
    for (const auto& [edge, length] : leg.interior) {
      total += length;
      if (seen.contains(edge)) {
        overlap += length;
      }
    }
    // only insert after the leg is done, a leg crossing itself isn't what we're after here
    seen.insert(leg.roads.begin(), leg.roads.end());
  }
  return total > 0. ? overlap / total : 0.;
}

// every loop through one seed per group, scored and sorted best first. groups holds location
// indices, the start is location 0
std::vector<candidate> rank_candidates(const std::vector<std::vector<uint32_t>>& groups,
                                       const connections& connections,
                                       const valhalla::Options::RoundTripOptions& round_trip_options,
                                       GraphReader& reader) {
  // walk the cartesian product of the groups, one seed per group, like an odometer
  std::vector<candidate> candidates;
  std::vector<size_t> odometer(groups.size(), 0);
  while (true) {
    candidate candidate;
    uint32_t previous = 0;
    bool reachable = true;
    for (size_t g = 0; g <= groups.size() && reachable; ++g) {
      // the last leg goes back to the start
      const uint32_t next = g < groups.size() ? groups[g][odometer[g]] : 0;
      const auto connection = connections.index(previous, next);
      // unfound connections never get a path
      if (connections.paths[connection].empty()) {
        reachable = false;
        break;
      }
      candidate.connections.push_back(connection);
      candidate.distance += connections.distances[connection];
      candidate.cost += connections.costs[connection];
      previous = next;
    }
    if (reachable && candidate.distance > 0.) {
      candidates.push_back(std::move(candidate));
    }

    // advance the odometer
    size_t g = 0;
    for (; g < odometer.size(); ++g) {
      if (++odometer[g] < groups[g].size()) {
        break;
      }
      odometer[g] = 0;
    }
    if (g == odometer.size()) {
      break;
    }
  }

  // score the candidates
  std::vector<LegInfo> leg_cache(connections.paths.size());
  double min_cost_per_distance = std::numeric_limits<double>::max();
  for (auto& c : candidates) {
    c.overlap = compute_overlap(c, leg_cache, connections.paths, reader);
    min_cost_per_distance = std::min(min_cost_per_distance, c.cost / c.distance);
  }

  const double overlap_factor = round_trip_options.overlap_factor();

  // drop the heavy retracers unless that would leave nothing
  if (overlap_factor > 0. &&
      std::any_of(candidates.begin(), candidates.end(),
                  [](const candidate& c) { return c.overlap <= kMaxOverlap; })) {
    std::erase_if(candidates, [](const candidate& c) { return c.overlap > kMaxOverlap; });
  }

  const double desired_distance = round_trip_options.distance();
  for (auto& c : candidates) {
    const double distance_deviation = std::abs(c.distance - desired_distance) / desired_distance;
    const double cost_per_distance = (c.cost / c.distance) / min_cost_per_distance - 1.;
    c.score =
        distance_deviation + kCostPerDistanceWeight * cost_per_distance + overlap_factor * c.overlap;
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const candidate& a, const candidate& b) { return a.score < b.score; });
  return candidates;
}

// whether a candidate is good enough to stop looking at more seeds
bool meets_request(const candidate& candidate,
                   const valhalla::Options::RoundTripOptions& round_trip_options) {
  const double distance_deviation =
      std::abs(candidate.distance - round_trip_options.distance()) / round_trip_options.distance();
  return distance_deviation <= kAcceptableDistanceDeviation &&
         (round_trip_options.overlap_factor() == 0. || candidate.overlap <= kAcceptableOverlap);
}

std::vector<PointLL> stitch_shape(const candidate& candidate, const connections& connections) {
  std::vector<PointLL> shape;
  for (auto connection : candidate.connections) {
    auto leg = decode<std::vector<PointLL>>(connections.shapes[connection], 1.0 / 1e6);
    if (leg.empty()) {
      continue;
    }
    // legs meet at a seed's projected point partway along the seed edge. That point isn't a vertex
    // of the edge and the edge walk only accepts the edge's own vertices, so drop it entirely
    auto begin = leg.begin();
    if (!shape.empty() && shape.back().ApproximatelyEqual(*begin)) {
      shape.pop_back();
      ++begin;
    }
    shape.insert(shape.end(), begin, leg.end());
  }
  return shape;
}

} // namespace

namespace valhalla {
namespace thor {

void thor_worker_t::round_trip(Api& request) {
  // time this whole method and save that statistic
  auto _ = measure_scope_time(request);

  auto& options = *request.mutable_options();
  adjust_locations(request);
  auto costing = parse_costing(request);
  controller = AttributesController(options);

  // Save the original hierarchy limits for the final route calculation.
  const auto original_hierarchy_limits = mode_costing[int(mode)]->GetHierarchyLimits();

  check_hierarchy_limits(mode_costing[int(mode)]->GetHierarchyLimits(), mode_costing[int(mode)],
                         options.costings().find(options.costing_type())->second.options(),
                         hierarchy_limits_config_costmatrix, allow_hierarchy_limits_modifications,
                         mode_costing[int(mode)]->UseHierarchyLimits());

  // we stitch the final shapes from the matrix shapes, so set them on for the matrix
  const auto requested_shape_format = options.shape_format();
  options.set_shape_format(polyline6);

  // loki sends the start followed by the seeds, which were sampled randomly so any order is fine
  const auto all_locations = options.locations();
  std::vector<std::vector<uint32_t>> all_groups;
  for (int i = 1; i < all_locations.size(); ++i) {
    const auto group = all_locations[i].round_trip_group_index();
    if (all_groups.size() < group) {
      all_groups.resize(group);
    }
    all_groups[group - 1].push_back(i);
  }

  costmatrix_.set_interrupt(interrupt);
  costmatrix_.set_has_time(check_matrix_time(request, Matrix::CostMatrix));

  // a round trip only drives from one group to the next, so a small matrix per pair of consecutive
  // groups is enough. Each search then stops once it found that pair's few targets
  connections connections(all_locations.size());
  const auto compute_leg = [&](const std::vector<uint32_t>& sources,
                               const std::vector<uint32_t>& targets) {
    options.mutable_sources()->Clear();
    options.mutable_targets()->Clear();
    for (auto i : sources) {
      options.mutable_sources()->Add()->CopyFrom(all_locations[i]);
    }
    for (auto i : targets) {
      options.mutable_targets()->Add()->CopyFrom(all_locations[i]);
    }

    std::vector<std::vector<GraphId>> paths;
    request.clear_matrix();
    costmatrix_.Clear();
    costmatrix_.SourceToTarget(request, *reader, mode_costing, mode,
                               max_matrix_distance.find(costing)->second, &paths);

    const auto& matrix = request.matrix();
    for (size_t s = 0; s < sources.size(); ++s) {
      for (size_t t = 0; t < targets.size(); ++t) {
        const auto matrix_index = s * targets.size() + t;
        const auto connection = connections.index(sources[s], targets[t]);
        connections.distances[connection] = matrix.distances(matrix_index);
        connections.costs[connection] = matrix.costs(matrix_index);
        connections.shapes[connection] = matrix.shapes(matrix_index);
        connections.paths[connection] = std::move(paths[matrix_index]);
      }
    }
  };

  // start with a few seeds per group and double them until a candidate meets the request or all
  // seeds are used
  std::vector<candidate> candidates;
  bool extended_iteration = false;
  for (size_t seeds_per_group = kInitialSeedsPerGroup;; seeds_per_group *= 2) {

    if (extended_iteration) {
      LOG_DEBUG("Extended candidate search with seed locations per group = {}", seeds_per_group);
    }
    extended_iteration = true;
    std::vector<std::vector<uint32_t>> groups(all_groups.size());
    bool all_seeds_used = true;
    for (size_t g = 0; g < all_groups.size(); ++g) {
      const auto count = std::min(seeds_per_group, all_groups[g].size());
      all_seeds_used = all_seeds_used && count == all_groups[g].size();
      groups[g].assign(all_groups[g].begin(), all_groups[g].begin() + count);
    }

    // start -> first group -> ... -> last group -> start
    const std::vector<uint32_t> start{0};
    compute_leg(start, groups.front());
    for (size_t g = 0; g + 1 < groups.size(); ++g) {
      compute_leg(groups[g], groups[g + 1]);
    }
    compute_leg(groups.back(), start);

    candidates = rank_candidates(groups, connections, options.round_trip_options(), *reader);
    if (all_seeds_used ||
        (!candidates.empty() && meets_request(candidates.front(), options.round_trip_options()))) {
      break;
    }
  }

  options.mutable_sources()->Clear();
  options.mutable_targets()->Clear();
  request.clear_matrix();

  // set back to whatever the user asked for
  options.set_shape_format(requested_shape_format);

  if (candidates.empty()) {
    throw valhalla_exception_t{442};
  }

  const auto num_locations = connections.num_locations;
  const size_t num_results =
      std::min(candidates.size(), static_cast<size_t>(options.alternates()) + 1);

  // edge walk the stitched shapes, start and end of the walk are both the correlated start
  options.mutable_locations()->Clear();
  options.mutable_locations()->Add()->CopyFrom(all_locations[0]);
  options.mutable_locations()->Add()->CopyFrom(all_locations[0]);

  for (size_t i = 0; i < num_results; ++i) {
    const auto shape = stitch_shape(candidates[i], connections);
    if (shape.size() < 2) {
      LOG_DEBUG("Unusable shape for result {}", i);
      continue;
    }

    options.mutable_shape()->Clear();
    for (const auto& pt : shape) {
      auto* l = options.mutable_shape()->Add();
      l->mutable_ll()->set_lng(pt.lng());
      l->mutable_ll()->set_lat(pt.lat());
      l->set_type(Location::kVia);
    }
    options.mutable_shape(0)->set_type(Location::kBreak);
    options.mutable_shape(options.shape_size() - 1)->set_type(Location::kBreak);

    try {
      route_match(request);
      if (options.verbose()) {
        auto* route = request.mutable_trip()->mutable_routes(request.trip().routes_size() - 1);
        // every leg but the last one ends at a seed
        for (size_t leg = 0; leg + 1 < candidates[i].connections.size(); ++leg) {
          const auto& seed = all_locations[candidates[i].connections[leg] % num_locations];
          auto* l = route->mutable_round_trip_seeds()->Add();
          l->mutable_ll()->CopyFrom(seed.ll());
          l->set_round_trip_group_index(seed.round_trip_group_index());
        }
      }
    } catch (const std::exception& e) {
      // if the best candidate fails, the next one simply becomes the first route
      LOG_DEBUG("Round trip candidate {} failed to edge walk: {}", i, e.what());
    }
  }

  if (request.trip().routes_size() == 0) {
    throw valhalla_exception_t{443, "edge walk of the stitched round trip failed"};
  }
}

} // namespace thor
} // namespace valhalla
