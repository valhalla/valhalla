#include "sif/recost.h"
#include "thor/looptree.h"
#include "thor/triplegbuilder.h"
#include "thor/worker.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace {

// the paths from the start to a turning point are between these fractions of the round trip
constexpr double kMinLegFraction = 0.2;
constexpr double kMaxLegFraction = 0.6;

// how many pairs of turning points get routed before giving up
constexpr size_t kMaxAttempts = 30;

// the search for alternatives of the middle leg, which offer more round trip lengths
// per pair of turning points. A stretch or sharing threshold of 0 uses the A*'s defaults.
constexpr uint32_t kMiddleAlternates = 8;
constexpr float kThresholdDelta = 420.f;
constexpr float kAlternativeCostExtend = 1.5f;
constexpr uint32_t kAlternativeIterationsDelta = 200000;
constexpr float kAtMostLonger = 0.f;
constexpr float kAtMostShared = 0.f;

// cost factor for the middle leg on the edges of the first and the last leg, in either direction
constexpr double kUsedEdgeFactor = 60.;

// near the start the last leg can't avoid the first one, so the first leg isn't penalized for this
// fraction of the round trip, within bounds
constexpr double kFreeStartFraction = 0.05;
constexpr double kMinFreeStartLength = 300.;
constexpr double kMaxFreeStartLength = 2000.;

// cost factor on the edges at the junctions of earlier legs, but only for this distance next to the
// junction, so that crossing an earlier leg costs about the same wherever it happens
constexpr double kCrossingEdgeFactor = 50.;
constexpr double kCrossingLength = 100.;

// meters added to a round trip's distance error per meter of road it drives more than once
constexpr double kOverlapWeight = 20.;

// a pair of turning points and how far the round trip through them is expected to miss the
// desired distance
struct CandidatePair {
  size_t out;
  size_t back;
  double expected_error;
};

// a location at the start or at the end of the candidate's edge only
Location edge_location(const IntermediateCandidate& candidate, bool at_end) {
  Location location;
  location.mutable_ll()->set_lng(candidate.ll.lng());
  location.mutable_ll()->set_lat(candidate.ll.lat());
  auto* edge = location.mutable_correlation()->mutable_edges()->Add();
  edge->set_graph_id(candidate.edge_id);
  edge->set_percent_along(at_end ? 1. : 0.);
  edge->mutable_ll()->CopyFrom(location.ll());
  edge->set_distance(0.);
  edge->set_begin_node(!at_end);
  edge->set_end_node(at_end);
  return location;
}

void add_used_edges(const std::vector<GraphId>& edges,
                    GraphReader& reader,
                    Costing_Options& penalties) {
  for (const auto& edge_id : edges) {
    graph_tile_ptr tile;
    for (const auto& id : {edge_id, reader.GetOpposingEdgeId(edge_id, tile)}) {
      if (!id.is_valid()) {
        continue;
      }
      auto* factor = penalties.mutable_cost_factor_edges()->Add();
      factor->set_id(id.value);
      factor->set_factor(kUsedEdgeFactor);
      factor->set_start(0.);
      factor->set_end(1.);
    }
  }
}

// Penalizes the edges at the junctions the used edges pass, next to the junction: the start of the
// edges leaving it and the end of the edges arriving at it. Left out are the used edges themselves
// and the junction at the end of the last used edge, where the new leg starts.
void add_crossing_edges(const std::vector<GraphId>& edges,
                        GraphReader& reader,
                        Costing_Options& costing_options) {
  std::unordered_set<GraphId> used;
  for (const auto& edge_id : edges) {
    graph_tile_ptr tile;
    used.insert(edge_id);
    used.insert(reader.GetOpposingEdgeId(edge_id, tile));
  }

  // per edge the fraction penalized from its start and the one penalized up to its end
  std::unordered_map<GraphId, std::pair<double, double>> ranges;
  const auto add_node = [&](const GraphId& node_id) {
    graph_tile_ptr tile;
    const auto* node = reader.nodeinfo(node_id, tile);
    if (!node) {
      return;
    }
    GraphId edge_id(node_id.tileid(), node_id.level(), node->edge_index());
    for (uint32_t i = 0; i < node->edge_count(); ++i, ++edge_id) {
      const auto* edge = tile->directededge(edge_id);
      if (edge->is_shortcut() || edge->length() == 0) {
        continue;
      }
      const double fraction = std::min(1., kCrossingLength / edge->length());
      if (!used.contains(edge_id)) {
        auto& range = ranges[edge_id];
        range.first = std::max(range.first, fraction);
      }
      graph_tile_ptr opp_tile = tile;
      const auto opp_id = reader.GetOpposingEdgeId(edge_id, opp_tile);
      if (opp_id.is_valid() && !used.contains(opp_id)) {
        auto& range = ranges[opp_id];
        range.second = std::max(range.second, fraction);
      }
    }
  };
  for (size_t i = 0; i + 1 < edges.size(); ++i) {
    graph_tile_ptr tile;
    const auto* edge = reader.directededge(edges[i], tile);
    if (!edge) {
      continue;
    }
    // a path may pass the junction on any level
    add_node(edge->endnode());
    const auto* node = reader.nodeinfo(edge->endnode(), tile);
    for (uint32_t t = 0; node && t < node->transition_count(); ++t) {
      add_node(tile->transition(node->transition_index() + t)->endnode());
    }
  }

  const auto add_range = [&costing_options](const GraphId& edge_id, double start, double end) {
    auto* factor = costing_options.mutable_cost_factor_edges()->Add();
    factor->set_id(edge_id.value);
    factor->set_factor(kCrossingEdgeFactor);
    factor->set_start(start);
    factor->set_end(end);
  };
  for (const auto& [edge_id, range] : ranges) {
    if (range.first + range.second >= 1.) {
      add_range(edge_id, 0., 1.);
      continue;
    }
    if (range.first > 0.) {
      add_range(edge_id, 0., range.first);
    }
    if (range.second > 0.) {
      add_range(edge_id, 1. - range.second, 1.);
    }
  }
}

// the length of the roads driven more than once in either direction, the first free edges near the
// start aside
double overlap_length(const std::vector<GraphId>& edges, size_t free_edges, GraphReader& reader) {
  std::unordered_set<GraphId> roads;
  double overlap = 0.;
  for (size_t i = free_edges; i < edges.size(); ++i) {
    graph_tile_ptr tile;
    const auto* edge = reader.directededge(edges[i], tile);
    if (!edge) {
      continue;
    }
    const double length = edge->length();
    const auto opp_id = reader.GetOpposingEdgeId(edges[i], tile);
    const auto road = opp_id.is_valid() && opp_id.value < edges[i].value ? opp_id : edges[i];
    if (!roads.insert(road).second) {
      overlap += length;
    }
  }
  return overlap;
}

float percent_along(const Location& location, const GraphId& edge_id) {
  for (const auto& edge : location.correlation().edges()) {
    if (edge.graph_id() == edge_id) {
      return edge.percent_along();
    }
  }
  throw std::logic_error("Round trip doesn't start or end on one of the start's edges");
}

} // namespace

namespace valhalla {
namespace thor {

void thor_worker_t::round_trip(Api& request) {
  // time this whole method and save that statistic
  auto _ = measure_scope_time(request);

  auto& options = *request.mutable_options();
  adjust_locations(request);
  parse_costing(request);
  controller = AttributesController(options);

  auto& costing = mode_costing[static_cast<uint32_t>(mode)];
  const auto& costing_options = options.costings().find(options.costing_type())->second.options();
  if (check_hierarchy_limits(costing->GetHierarchyLimits(), costing, costing_options,
                             hierarchy_limits_config_bidirectional_astar,
                             allow_hierarchy_limits_modifications, costing->UseHierarchyLimits())) {
    add_warning(request, allow_hierarchy_limits_modifications ? 210 : 209);
  }

  // some information we need
  auto* locations = options.mutable_locations();
  const Location start = locations->Get(0);
  const PointLL start_ll(start.ll().lng(), start.ll().lat());
  const auto& round_trip_options = options.round_trip_options();
  const double desired_distance = round_trip_options.distance();
  const double heading = round_trip_options.heading();
  const double heading_tolerance = round_trip_options.heading_tolerance();
  const double free_start_length =
      std::clamp(kFreeStartFraction * desired_distance, kMinFreeStartLength, kMaxFreeStartLength);

  // the first turning point lies in the counter clockwise half of the heading's sector and the
  // second one in the clockwise half, so the round trip runs clockwise
  out_tree_.Run(ExpansionType::forward, *locations, *reader, mode_costing, mode,
                kMinLegFraction * desired_distance, kMaxLegFraction * desired_distance,
                heading - heading_tolerance / 2., heading_tolerance / 2.);
  back_tree_.Run(ExpansionType::reverse, *locations, *reader, mode_costing, mode,
                 kMinLegFraction * desired_distance, kMaxLegFraction * desired_distance,
                 heading + heading_tolerance / 2., heading_tolerance / 2.);
  const auto out_candidates = out_tree_.Candidates();
  const auto back_candidates = back_tree_.Candidates();

  // the pairs expected to come closest to the desired distance go first
  std::vector<CandidatePair> pairs;
  pairs.reserve(out_candidates.size() * back_candidates.size());
  for (size_t i = 0; i < out_candidates.size(); ++i) {
    for (size_t j = 0; j < back_candidates.size(); ++j) {
      const auto& a = out_candidates[i];
      const auto& b = back_candidates[j];
      // the middle leg is expected to detour from its beeline as much as the paths to a and b do
      const double beelines = start_ll.Distance(a.ll) + start_ll.Distance(b.ll);
      const double detour = beelines > 0. ? (a.distance + b.distance) / beelines : 1.;
      const double expected = a.distance + detour * a.ll.Distance(b.ll) + b.distance;
      pairs.push_back({i, j, std::abs(expected - desired_distance)});
    }
  }
  std::sort(pairs.begin(), pairs.end(), [](const CandidatePair& x, const CandidatePair& y) {
    return x.expected_error < y.expected_error;
  });

  leg_astar_.SetAlternativeSettings(kThresholdDelta, kAlternativeCostExtend,
                                    kAlternativeIterationsDelta, kAtMostLonger, kAtMostShared);
  Options middle_options = options;
  middle_options.set_alternates(kMiddleAlternates);
  Options last_options = options;
  last_options.set_alternates(0);

  // TODO: check restrictions
  // routes a leg that avoids the roads of the legs before it
  const auto route_leg = [&](Location& origin, Location& destination,
                             const std::vector<GraphId>& used, const Options& leg_options) {
    Costing_Options penalties;
    penalties.mutable_cost_factor_edges()->CopyFrom(costing_options.cost_factor_edges());
    add_used_edges(used, *reader, penalties);
    add_crossing_edges(used, *reader, penalties);
    costing->SetCostFactorEdges(penalties);
    leg_astar_.Clear();
    auto paths =
        leg_astar_.GetBestPath(origin, destination, *reader, mode_costing, mode, leg_options);
    costing->SetCostFactorEdges(costing_options);
    return paths;
  };

  // the round trip that misses the desired distance least, with its overlap counted as a miss too,
  // the search stops once one is within tolerance
  const double tolerance = round_trip_options.distance_tolerance() * desired_distance;
  std::vector<GraphId> edges;
  double best_score = std::numeric_limits<double>::max();

  for (size_t i = 0; i < std::min(pairs.size(), kMaxAttempts) && best_score > tolerance; ++i) {
    const auto& a = out_candidates[pairs[i].out];
    const auto& b = back_candidates[pairs[i].back];
    auto at_a = edge_location(a, true);
    auto at_b = edge_location(b, false);

    // the middle leg starts on a's edge, which ends the first leg, and ends on b's edge, which
    // starts the last leg
    const auto first = out_tree_.Path(a.label);

    // the first leg is penalized only beyond its first edges near the start
    size_t free_edges = 0;
    double length = 0.;
    graph_tile_ptr tile;
    for (; free_edges < first.size(); ++free_edges) {
      const auto* edge = reader->directededge(first[free_edges], tile);
      length += edge ? edge->length() : 0.;
      if (length > free_start_length) {
        break;
      }
    }
    std::vector<GraphId> used(first.begin() + free_edges, first.end());

    // every alternative of the middle leg gets its own last leg, which avoids it
    const auto middles = route_leg(at_a, at_b, used, middle_options);
    for (const auto& middle : middles) {
      if (middle.size() < 2) {
        continue;
      }
      auto last_used = used;
      for (size_t k = 1; k + 1 < middle.size(); ++k) {
        last_used.push_back(middle[k].edgeid);
      }
      auto destination = start;
      const auto lasts = route_leg(at_b, destination, last_used, last_options);
      if (lasts.empty() || lasts.front().empty()) {
        continue;
      }
      const auto& last = lasts.front();

      auto round_trip = first;
      for (size_t k = 1; k + 1 < middle.size(); ++k) {
        round_trip.push_back(middle[k].edgeid);
      }
      for (const auto& info : last) {
        round_trip.push_back(info.edgeid);
      }

      const double distance = a.distance + middle.back().path_distance + last.back().path_distance;
      const double score = std::abs(distance - desired_distance) +
                           kOverlapWeight * overlap_length(round_trip, free_edges, *reader);
      if (score >= best_score) {
        continue;
      }
      best_score = score;
      edges = std::move(round_trip);
      if (best_score <= tolerance) {
        break;
      }
    }
  }
  if (edges.empty()) {
    throw valhalla_exception_t{442};
  }

  // the round trip starts and ends at the start
  locations->Clear();
  locations->Add()->CopyFrom(start);
  locations->Add()->CopyFrom(start);

  std::vector<PathInfo> path;
  path.reserve(edges.size());
  auto edge_itr = edges.begin();
  const auto edge_cb = [&edge_itr, &edges]() {
    return edge_itr == edges.end() ? GraphId{} : *edge_itr++;
  };
  const auto label_cb = [&path](const PathEdgeLabel& label) {
    path.emplace_back(label.mode(), label.cost(), label.edgeid(), 0, label.path_distance(),
                      label.restriction_idx(), label.transition_cost());
  };
  recost_forward(*reader, *costing, edge_cb, label_cb, percent_along(start, edges.front()),
                 percent_along(start, edges.back()), TimeInfo::invalid(), false, false);

  auto origin = options.locations(0);
  auto destination = options.locations(1);
  auto* route = request.mutable_trip()->mutable_routes()->Add();
  TripLegBuilder::Build(options, controller, *reader, mode_costing, path.begin(), path.end(), origin,
                        destination, *route->mutable_legs()->Add(), {"round_trip"}, interrupt);
}

} // namespace thor
} // namespace valhalla
