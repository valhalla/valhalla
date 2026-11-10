#include "baldr/graphreader.h"
#include "midgard/encoded.h"
#include "thor/costmatrix.h"
#include "thor/worker.h"

#include <algorithm>
#include <cmath>
#include <limits>
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
constexpr double kCostPerDistanceWeight = 0.25;
// candidates retracing more than this fraction of their length are only used if nothing else is
// left, unless the request doesn't care about overlap at all
constexpr double kMaxOverlap = 0.5;

// what a single matrix connection contributes to a candidate, computed once per connection
struct LegInfo {
  bool computed = false;
  // edges and their opposing edges, i.e. every road this leg touches in either direction
  std::vector<GraphId> roads;
  // edges excluding the first and last one, which are shared with the adjacent legs by design
  std::vector<std::pair<GraphId, float>> interior;
};

struct Candidate {
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
double compute_overlap(const Candidate& candidate,
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

std::vector<PointLL> stitch_shape(const Candidate& candidate, const Matrix& matrix) {
  std::vector<PointLL> shape;
  for (auto connection : candidate.connections) {
    auto leg = decode<std::vector<PointLL>>(matrix.shapes(connection), 1.0 / 1e6);
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

  std::vector<std::vector<GraphId>> paths;
  costmatrix_.set_interrupt(interrupt);
  costmatrix_.set_has_time(check_matrix_time(request, Matrix::CostMatrix));
  costmatrix_.SourceToTarget(request, *reader, mode_costing, mode,
                             max_matrix_distance.find(costing)->second, &paths);

  // set back to whatever the user asked for
  options.set_shape_format(requested_shape_format);

  // sources and targets are both [start, seeds...], so source index i == target index i
  const auto num_locations = static_cast<uint32_t>(options.targets_size());
  const auto connection_idx = [num_locations](uint32_t source, uint32_t target) {
    return source * num_locations + target;
  };

  // location indices per group, group 0 is the start
  std::vector<std::vector<uint32_t>> groups;
  for (uint32_t i = 1; i < num_locations; ++i) {
    const auto group = options.targets(i).round_trip_group_index();
    if (groups.size() < group) {
      groups.resize(group);
    }
    groups[group - 1].push_back(i);
  }

  const auto& matrix = request.matrix();
  const double desired_distance = options.round_trip_options().distance();

  // walk the cartesian product of the groups, one seed per group, like an odometer
  std::vector<Candidate> candidates;
  std::vector<size_t> odometer(groups.size(), 0);
  while (true) {
    Candidate candidate;
    uint32_t previous = 0;
    bool reachable = true;
    for (size_t g = 0; g <= groups.size() && reachable; ++g) {
      // the last leg goes back to the start
      const uint32_t next = g < groups.size() ? groups[g][odometer[g]] : 0;
      const auto connection = connection_idx(previous, next);
      // unfound connections never get a path
      if (paths[connection].empty()) {
        reachable = false;
        break;
      }
      candidate.connections.push_back(connection);
      candidate.distance += matrix.distances(connection);
      candidate.cost += matrix.costs(connection);
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

  if (candidates.empty()) {
    throw valhalla_exception_t{442};
  }

  // score the candidates
  std::vector<LegInfo> leg_cache(paths.size());
  double min_cost_per_distance = std::numeric_limits<double>::max();
  for (auto& c : candidates) {
    c.overlap = compute_overlap(c, leg_cache, paths, *reader);
    min_cost_per_distance = std::min(min_cost_per_distance, c.cost / c.distance);
  }

  const double overlap_factor = options.round_trip_options().overlap_factor();

  // drop the heavy retracers unless that would leave nothing
  if (overlap_factor > 0. &&
      std::any_of(candidates.begin(), candidates.end(),
                  [](const Candidate& c) { return c.overlap <= kMaxOverlap; })) {
    std::erase_if(candidates, [](const Candidate& c) { return c.overlap > kMaxOverlap; });
  }

  for (auto& c : candidates) {
    const double distance_deviation = std::abs(c.distance - desired_distance) / desired_distance;
    const double cost_per_distance = (c.cost / c.distance) / min_cost_per_distance - 1.;
    c.score =
        distance_deviation + kCostPerDistanceWeight * cost_per_distance + overlap_factor * c.overlap;
  }

  const size_t num_results =
      std::min(candidates.size(), static_cast<size_t>(options.alternates()) + 1);
  std::partial_sort(candidates.begin(), candidates.begin() + num_results, candidates.end(),
                    [](const Candidate& a, const Candidate& b) { return a.score < b.score; });

  // edge walk the stitched shapes, start and end of the walk are both the correlated start
  const Location start = options.targets(0);
  options.mutable_locations()->Clear();
  options.mutable_locations()->Add()->CopyFrom(start);
  options.mutable_locations()->Add()->CopyFrom(start);

  for (size_t i = 0; i < num_results; ++i) {
    const auto shape = stitch_shape(candidates[i], matrix);
    if (shape.size() < 2) {
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
          const auto& seed = options.targets(candidates[i].connections[leg] % num_locations);
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

  request.clear_matrix();
}

} // namespace thor
} // namespace valhalla
