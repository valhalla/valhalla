#include "baldr/graphreader.h"
#include "midgard/logging.h"
#include "sif/recost.h"
#include "thor/costmatrix.h"
#include "thor/dijkstras.h"
#include "thor/triplegbuilder.h"
#include "thor/worker.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <unordered_set>
#include <vector>

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace {

// A round trip is a triangle: start -> a -> b -> start. The first leg comes from a forward expansion
// from the start, the last leg from a reverse expansion into the start, and only the middle leg
// a -> b needs a matrix.

// round trip candidates are weighted on distance deviation (implicit weight of 1), cost per distance,
// overlap and how well the triangle points into the requested heading
constexpr double kCostPerDistanceWeight = 0.25;
constexpr double kShapeWeight = 0.02;

// candidates retracing more than this fraction of their length are only used if nothing else is
// left
constexpr double kMaxOverlap = 0.3;

// each leg is about a third of the round trip. The expansions record turning points over a wider
// band of network distance, so that a second pass can pick shorter or longer legs if the first one
// misses the requested length
constexpr double kMinLegFraction = 0.1;
constexpr double kMaxLegFraction = 0.47;
constexpr double kTargetLegFraction = 1. / 3.;

// near the start the bearing of a node says little about where the path goes, so the expansions are
// only kept inside their cone beyond this distance, and with some slack so paths can bend around
// obstacles
constexpr double kConeStartDistance = 3000.;
constexpr double kConeMargin = 25.;

// turning points are recorded on a grid of distance and bearing, keeping the first junction in each
// cell. Each pass takes a few around its target distance from every bearing cell. The middle leg
// matrix is the square of that, so it has to stay small
constexpr int kDistanceBins = 10;
constexpr int kBearingBins = 4;
constexpr int kCoarseBearingBins = 3;

// the inner angle of an equilateral triangle, the ideal opening at the start
constexpr double kIdealOpening = 60.;

// the more the length may deviate, the fewer turning points a pass needs to find a fitting round
// trip. Without hierarchy limits every matrix search covers the whole local network, so those
// costings take one fewer
int turns_per_bearing(double distance_tolerance, bool coarse) {
  const int turns = distance_tolerance < 0.15 ? 3 : distance_tolerance < 0.3 ? 2 : 1;
  return std::max(1, coarse ? turns - 1 : turns);
}

double angle_diff(double a, double b) {
  const double d = std::fmod(std::abs(a - b), 360.);
  return d > 180. ? 360. - d : d;
}

// angle from `from` to `to` in (-180, 180], positive is clockwise
double signed_angle(double from, double to) {
  return std::fmod(to - from + 540., 360.) - 180.;
}

// places nobody wants a round trip to turn at, or that can't be passed through at all
constexpr uint64_t kTurnExcludeUses = [] {
  uint64_t mask = 0;
  for (auto use :
       {baldr::Use::kRamp, baldr::Use::kTurnChannel, baldr::Use::kDriveway, baldr::Use::kAlley,
        baldr::Use::kParkingAisle, baldr::Use::kEmergencyAccess, baldr::Use::kDriveThru,
        baldr::Use::kCuldesac, baldr::Use::kServiceRoad, baldr::Use::kRestArea,
        baldr::Use::kServiceArea, baldr::Use::kSteps, baldr::Use::kElevator, baldr::Use::kEscalator,
        baldr::Use::kLadder, baldr::Use::kPlatform, baldr::Use::kFerry, baldr::Use::kRailFerry,
        baldr::Use::kConstruction}) {
    mask |= uint64_t{1} << static_cast<uint8_t>(use);
  }
  return mask;
}();

// a node where the round trip can turn towards its next leg
struct Turn {
  uint32_t label;
  GraphId node;
  PointLL ll;
  double distance;
  double bearing;
  // whether more than one road meets here. Elsewhere the round trip can only continue straight on
  bool junction;
  int bearing_bin;
};

// An expansion from or into the start that only grows inside a cone around one corner of the
// triangle, and only on the higher road classes away from the start. It collects the turning points
// for that corner on the way.
class LoopTree : public Dijkstras {
public:
  struct Params {
    PointLL start;
    double min_distance;
    double max_distance;
    double cone_center;
    double cone_half_width;
    bool motor_vehicle;
    int distance_bins;
    int bearing_bins;
  };

  LoopTree(const boost::property_tree::ptree& config, const Params& params)
      : Dijkstras(config), params_(params) {
  }

  void Run(ExpansionType type,
           const Location& start,
           GraphReader& reader,
           const mode_costing_t& costings,
           travel_mode_t mode) {
    reverse_ = type == ExpansionType::reverse;
    use_hierarchy_limits_ = true;
    google::protobuf::RepeatedPtrField<Location> locations;
    locations.Add()->CopyFrom(start);
    if (reverse_) {
      Compute<ExpansionType::reverse>(locations, reader, costings, mode);
    } else {
      Compute<ExpansionType::forward>(locations, reader, costings, mode);
    }
  }

  std::vector<Turn> Turns() const {
    std::vector<Turn> turns;
    turns.reserve(turns_.size());
    for (const auto& [cell, turn] : turns_) {
      turns.push_back(turn);
    }
    return turns;
  }

  // the edges between the start and the turning point, in driving order
  std::vector<GraphId> Path(uint32_t label) const {
    std::vector<GraphId> edges;
    for (auto l = label; l != kInvalidLabel; l = bdedgelabels_[l].predecessor()) {
      edges.push_back(DrivingEdge(l));
    }
    if (!reverse_) {
      std::reverse(edges.begin(), edges.end());
    }
    return edges;
  }

  // reverse labels hold the edge in expansion direction, the driven one is its opposing edge
  GraphId DrivingEdge(uint32_t label) const {
    return reverse_ ? bdedgelabels_[label].opp_edgeid() : bdedgelabels_[label].edgeid();
  }

  const BDEdgeLabel& Label(uint32_t label) const {
    return bdedgelabels_[label];
  }

  size_t LabelCount() const {
    return bdedgelabels_.size();
  }

protected:
  void ExpandingNode(GraphReader&,
                     graph_tile_ptr,
                     const NodeInfo*,
                     const EdgeLabel&,
                     const EdgeLabel*) override {
  }

  void GetExpansionHints(uint32_t& bucket_count, uint32_t& edge_label_reservation) const override {
    bucket_count = 20000;
    edge_label_reservation = 200000;
  }

  ExpansionRecommendation
  ShouldExpand(GraphReader& reader, const EdgeLabel& pred, const ExpansionType) override {
    const double distance = pred.path_distance();
    if (distance > params_.max_distance) {
      return ExpansionRecommendation::prune_expansion;
    }

    graph_tile_ptr tile;
    const auto* node = reader.nodeinfo(pred.endnode(), tile);
    if (!node) {
      return ExpansionRecommendation::continue_expansion;
    }
    const PointLL ll = node->latlng(tile->header()->base_ll());
    const double bearing = params_.start.Heading(ll);
    const double off_center = angle_diff(bearing, params_.cone_center);
    if (distance > kConeStartDistance && off_center > params_.cone_half_width + kConeMargin) {
      return ExpansionRecommendation::prune_expansion;
    }

    if (distance >= params_.min_distance && off_center <= params_.cone_half_width &&
        node->edge_count() >= 2) {
      const uint32_t label = edgestatus_.Get(pred.edgeid(), pred.path_id()).index();
      graph_tile_ptr edge_tile;
      const auto* edge = reader.directededge(DrivingEdge(label), edge_tile);
      if (edge && Usable(edge)) {
        const int distance_bin =
            std::clamp(static_cast<int>((distance - params_.min_distance) /
                                        (params_.max_distance - params_.min_distance) *
                                        params_.distance_bins),
                       0, params_.distance_bins - 1);
        const int bearing_bin =
            std::clamp(static_cast<int>(
                           (signed_angle(params_.cone_center, bearing) + params_.cone_half_width) /
                           (2. * params_.cone_half_width) * params_.bearing_bins),
                       0, params_.bearing_bins - 1);
        const Turn turn{label,   pred.endnode(),          ll,         distance,
                        bearing, node->edge_count() >= 3, bearing_bin};
        auto [cell, inserted] =
            turns_.try_emplace(distance_bin * params_.bearing_bins + bearing_bin, turn);
        if (!inserted && !cell->second.junction && turn.junction) {
          cell->second = turn;
        }
      }
    }
    return ExpansionRecommendation::continue_expansion;
  }

private:
  // whether the edge a turning point is arrived on or left on is a proper road to pass through
  bool Usable(const DirectedEdge* edge) const {
    if (edge->link() || edge->internal() || edge->roundabout() || edge->deadend() ||
        edge->not_thru() || (kTurnExcludeUses & (uint64_t{1} << static_cast<uint8_t>(edge->use())))) {
      return false;
    }
    // away from the start the hierarchy limits keep motor vehicles on higher road classes anyway
    return !params_.motor_vehicle || edge->classification() <= baldr::RoadClass::kResidential;
  }

  Params params_;
  bool reverse_ = false;
  std::map<int, Turn> turns_;
};

// from every bearing cell turning points spread around the target distance that aren't used yet, so
// that the pairs cover a range of round trip lengths
std::vector<size_t> select_turns(const std::vector<Turn>& turns,
                                 int bearing_bins,
                                 int per_bearing,
                                 double target,
                                 std::vector<bool>& used) {
  constexpr double kSpread = 0.2;
  std::vector<size_t> selected;
  for (int bin = 0; bin < bearing_bins; ++bin) {
    for (int k = 0; k < per_bearing; ++k) {
      const double aim =
          per_bearing == 1
              ? target
              : target * (1. - kSpread + 2. * kSpread * k / static_cast<double>(per_bearing - 1));
      std::optional<size_t> best;
      for (size_t i = 0; i < turns.size(); ++i) {
        if (turns[i].bearing_bin == bin && !used[i] &&
            (!best || std::abs(turns[i].distance - aim) < std::abs(turns[*best].distance - aim))) {
          best = i;
        }
      }
      if (best) {
        used[*best] = true;
        selected.push_back(*best);
      }
    }
  }
  return selected;
}

// a leg of a round trip candidate and what it contributes to the overlap
struct Leg {
  std::vector<GraphId> edges;
  double distance = 0.;
  double cost = 0.;
  bool found = false;

  // edges and their opposing edges, i.e. every road this leg touches in either direction
  // TODO: this uses a narrow definition of "every edge along a stretch of road" because it leaves out
  // edges from the opposing part of a doubly digitized way, as well as sidewalks and adjacent cycle
  // paths mapped separately
  std::vector<GraphId> roads;
  // edges and their length, without the first and last edge which touch the turning points
  std::vector<std::pair<GraphId, float>> interior;
};

void index_leg(Leg& leg, GraphReader& reader) {
  graph_tile_ptr tile;
  leg.roads.reserve(leg.edges.size() * 2);
  for (size_t i = 0; i < leg.edges.size(); ++i) {
    const auto* edge = reader.directededge(leg.edges[i], tile);
    if (!edge) {
      continue;
    }
    leg.roads.push_back(leg.edges[i]);
    auto opp = reader.GetOpposingEdgeId(leg.edges[i], tile);
    if (opp.is_valid()) {
      leg.roads.push_back(opp);
    }
    if (i != 0 && i + 1 != leg.edges.size()) {
      leg.interior.emplace_back(leg.edges[i], static_cast<float>(edge->length()));
    }
  }
}

// A complete round trip candidate
struct candidate {
  size_t a = 0; // index into the turning points of the forward expansion
  size_t b = 0; // index into the turning points of the reverse expansion
  double distance = 0.;
  double cost = 0.;
  double overlap = 0.; // fraction of the distance that is driven on a road used by an earlier leg
  double score = 0.;
};

// fraction of the legs' interior length that runs on roads an earlier leg already used
double compute_overlap(const std::array<const Leg*, 3>& legs) {
  std::unordered_set<GraphId> seen;
  double overlap = 0.;
  double total = 0.;
  for (const auto* leg : legs) {
    for (const auto& [edge, length] : leg->interior) {
      total += length;
      if (seen.contains(edge)) {
        overlap += length;
      }
    }
    // only insert after the leg is done, a leg crossing itself isn't what we're after here
    seen.insert(leg->roads.begin(), leg->roads.end());
  }
  return total > 0. ? overlap / total : 0.;
}

// a location at a node: the edges leaving it for a source, the edges arriving at it for a target,
// except for the edge that would turn the round trip around at that node
Location node_location(const Turn& turn, GraphId uturn_edge, bool as_source, GraphReader& reader) {
  Location location;
  location.mutable_ll()->set_lng(turn.ll.lng());
  location.mutable_ll()->set_lat(turn.ll.lat());
  graph_tile_ptr tile;
  const auto* node = reader.nodeinfo(turn.node, tile);
  if (!node) {
    return location;
  }
  GraphId edge_id(turn.node.tileid(), turn.node.level(), node->edge_index());
  for (uint32_t i = 0; i < node->edge_count(); ++i, ++edge_id) {
    const auto* edge = tile->directededge(edge_id);
    if (edge->is_shortcut()) {
      continue;
    }
    GraphId id = edge_id;
    if (!as_source) {
      graph_tile_ptr opp_tile = tile;
      id = reader.GetOpposingEdgeId(edge_id, opp_tile);
      if (!id.is_valid()) {
        continue;
      }
    }
    if (id == uturn_edge) {
      continue;
    }
    auto* path_edge = location.mutable_correlation()->mutable_edges()->Add();
    path_edge->set_graph_id(id);
    path_edge->set_percent_along(as_source ? 0. : 1.);
    path_edge->mutable_ll()->CopyFrom(location.ll());
    path_edge->set_distance(0.);
    path_edge->set_begin_node(as_source);
    path_edge->set_end_node(!as_source);
  }
  return location;
}

float find_percent_along(const Location& location, GraphId edge_id) {
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
  auto costing = parse_costing(request);
  controller = AttributesController(options);

  if (check_hierarchy_limits(mode_costing[int(mode)]->GetHierarchyLimits(), mode_costing[int(mode)],
                             options.costings().find(options.costing_type())->second.options(),
                             hierarchy_limits_config_costmatrix, allow_hierarchy_limits_modifications,
                             mode_costing[int(mode)]->UseHierarchyLimits())) {
    add_warning(request, allow_hierarchy_limits_modifications ? 210 : 209);
  }

  const Location start = options.locations(0);
  const auto& round_trip_options = options.round_trip_options();
  const double desired_distance = round_trip_options.distance();
  const double heading = round_trip_options.heading();
  const double tolerance = round_trip_options.heading_tolerance();
  const bool motor_vehicle = mode == travel_mode_t::kDrive;
  const bool coarse = !mode_costing[int(mode)]->UseHierarchyLimits();
  const int bearing_bins = coarse ? kCoarseBearingBins : kBearingBins;
  const double distance_tolerance = round_trip_options.distance_tolerance();
  const int per_bearing = turns_per_bearing(distance_tolerance, coarse);

  // the first turning point lies in the counter clockwise half of the heading's sector and the second
  // one in the clockwise half, so the round trip runs clockwise
  boost::property_tree::ptree tree_config;
  tree_config.put("max_reserved_labels_count_dijkstras", 200000);
  const PointLL start_ll(start.ll().lng(), start.ll().lat());
  LoopTree out_tree(tree_config, {start_ll, kMinLegFraction * desired_distance,
                                  kMaxLegFraction * desired_distance, heading - tolerance / 2.,
                                  tolerance / 2., motor_vehicle, kDistanceBins, bearing_bins});
  LoopTree back_tree(tree_config, {start_ll, kMinLegFraction * desired_distance,
                                   kMaxLegFraction * desired_distance, heading + tolerance / 2.,
                                   tolerance / 2., motor_vehicle, kDistanceBins, bearing_bins});
  out_tree.Run(ExpansionType::forward, start, *reader, mode_costing, mode);
  back_tree.Run(ExpansionType::reverse, start, *reader, mode_costing, mode);

  const auto a_turns = out_tree.Turns();
  const auto b_turns = back_tree.Turns();
  LOG_DEBUG("Round trip turning points: {} outbound, {} inbound, expansions settled {} and {} edges",
            a_turns.size(), b_turns.size(), out_tree.LabelCount(), back_tree.LabelCount());
  if (a_turns.empty() || b_turns.empty()) {
    throw valhalla_exception_t{442};
  }

  // the first and last legs come straight from the expansions
  std::vector<Leg> out_legs(a_turns.size());
  for (size_t i = 0; i < a_turns.size(); ++i) {
    auto& leg = out_legs[i];
    leg.edges = out_tree.Path(a_turns[i].label);
    leg.distance = out_tree.Label(a_turns[i].label).path_distance();
    leg.cost = out_tree.Label(a_turns[i].label).cost().cost;
    leg.found = true;
    index_leg(leg, *reader);
  }
  std::vector<Leg> back_legs(b_turns.size());
  for (size_t j = 0; j < b_turns.size(); ++j) {
    auto& leg = back_legs[j];
    leg.edges = back_tree.Path(b_turns[j].label);
    leg.distance = back_tree.Label(b_turns[j].label).path_distance();
    leg.cost = back_tree.Label(b_turns[j].label).cost().cost;
    leg.found = true;
    index_leg(leg, *reader);
  }

  // a matrix for the middle legs between the selected turning points. Leaving a turning point on the
  // edge it was arrived on, or arriving on the edge it is left on, would turn the round trip around,
  // so those edges are left out
  std::vector<Leg> middle_legs(a_turns.size() * b_turns.size());
  const auto compute_middle_legs = [&](const std::vector<size_t>& sources,
                                       const std::vector<size_t>& targets) {
    if (sources.empty() || targets.empty()) {
      return;
    }
    options.mutable_sources()->Clear();
    options.mutable_targets()->Clear();
    for (auto i : sources) {
      graph_tile_ptr tile;
      const auto uturn = reader->GetOpposingEdgeId(out_legs[i].edges.back(), tile);
      *options.mutable_sources()->Add() = node_location(a_turns[i], uturn, true, *reader);
    }
    for (auto j : targets) {
      graph_tile_ptr tile;
      const auto uturn = reader->GetOpposingEdgeId(back_legs[j].edges.front(), tile);
      *options.mutable_targets()->Add() = node_location(b_turns[j], uturn, false, *reader);
    }

    // only the matrix paths are needed, not their shapes
    const auto requested_shape_format = options.shape_format();
    options.set_shape_format(no_shape);
    std::vector<std::vector<GraphId>> paths;
    request.clear_matrix();
    costmatrix_.Clear();
    costmatrix_.set_interrupt(interrupt);
    costmatrix_.set_has_time(check_matrix_time(request, Matrix::CostMatrix));
    costmatrix_.SourceToTarget(request, *reader, mode_costing, mode,
                               max_matrix_distance.find(costing)->second, &paths);
    options.set_shape_format(requested_shape_format);

    const auto& matrix = request.matrix();
    for (size_t s = 0; s < sources.size(); ++s) {
      for (size_t t = 0; t < targets.size(); ++t) {
        const size_t m = s * targets.size() + t;
        auto& leg = middle_legs[sources[s] * b_turns.size() + targets[t]];
        leg.found = m < paths.size() && !paths[m].empty();
        if (!leg.found) {
          continue;
        }
        leg.edges = std::move(paths[m]);
        leg.distance = matrix.distances(m);
        leg.cost = matrix.costs(m);
        index_leg(leg, *reader);
      }
    }
    options.mutable_sources()->Clear();
    options.mutable_targets()->Clear();
    request.clear_matrix();
  };

  // every pair of turning points with a middle leg, best first
  const double overlap_factor = round_trip_options.overlap_factor();
  const auto score_candidates = [&]() {
    std::vector<candidate> candidates;
    double min_cost_per_distance = std::numeric_limits<double>::max();
    for (size_t i = 0; i < a_turns.size(); ++i) {
      for (size_t j = 0; j < b_turns.size(); ++j) {
        const auto& middle = middle_legs[i * b_turns.size() + j];
        if (!middle.found) {
          continue;
        }
        candidate c;
        c.a = i;
        c.b = j;
        c.distance = out_legs[i].distance + middle.distance + back_legs[j].distance;
        c.cost = out_legs[i].cost + middle.cost + back_legs[j].cost;
        if (c.distance <= 0.) {
          continue;
        }
        c.overlap = compute_overlap({&out_legs[i], &middle, &back_legs[j]});
        min_cost_per_distance = std::min(min_cost_per_distance, c.cost / c.distance);
        candidates.push_back(c);
      }
    }

    // drop the heavy retracers unless that would leave nothing
    if (overlap_factor > 0. &&
        std::any_of(candidates.begin(), candidates.end(),
                    [](const candidate& c) { return c.overlap <= kMaxOverlap; })) {
      std::erase_if(candidates, [](const candidate& c) { return c.overlap > kMaxOverlap; });
    }

    for (auto& c : candidates) {
      const double distance_deviation = std::abs(c.distance - desired_distance) / desired_distance;
      const double cost_per_distance = (c.cost / c.distance) / min_cost_per_distance - 1.;
      // the triangle should open about as wide as an equilateral one and point into the heading
      const double opening = signed_angle(a_turns[c.a].bearing, b_turns[c.b].bearing);
      const double bisector = a_turns[c.a].bearing + opening / 2.;
      const double shape = (std::abs(opening - kIdealOpening) + angle_diff(bisector, heading)) / 90.;
      c.score = distance_deviation + kCostPerDistanceWeight * cost_per_distance +
                overlap_factor * c.overlap + kShapeWeight * shape;
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const candidate& a, const candidate& b) { return a.score < b.score; });
    return candidates;
  };

  // first pass with legs of about a third of the round trip
  std::vector<bool> used_a(a_turns.size(), false);
  std::vector<bool> used_b(b_turns.size(), false);
  const double first_target = kTargetLegFraction * desired_distance;
  compute_middle_legs(select_turns(a_turns, bearing_bins, per_bearing, first_target, used_a),
                      select_turns(b_turns, bearing_bins, per_bearing, first_target, used_b));
  auto candidates = score_candidates();

  // if that misses the requested tolerance, the round trip closest to the length tells how much
  // longer or shorter than planned the legs come out in this area, and a second pass picks turning
  // points accordingly
  const auto deviation = [&](const candidate& c) {
    return std::abs(c.distance - desired_distance) / desired_distance;
  };
  if (!candidates.empty() && deviation(candidates.front()) > distance_tolerance) {
    const auto& closest = *std::min_element(candidates.begin(), candidates.end(),
                                            [&](const candidate& x, const candidate& y) {
                                              return deviation(x) < deviation(y);
                                            });
    const double second_target =
        std::clamp((out_legs[closest.a].distance + back_legs[closest.b].distance) / 2. *
                       desired_distance / closest.distance,
                   kMinLegFraction * desired_distance, kMaxLegFraction * desired_distance);
    LOG_DEBUG("Round trip second pass, closest is {:.0f}m, legs now aim for {:.0f}m",
              closest.distance, second_target);
    compute_middle_legs(select_turns(a_turns, bearing_bins, per_bearing, second_target, used_a),
                        select_turns(b_turns, bearing_bins, per_bearing, second_target, used_b));
    candidates = score_candidates();
  }
  if (candidates.empty()) {
    throw valhalla_exception_t{442};
  }

  const size_t num_results =
      std::min(candidates.size(), static_cast<size_t>(options.alternates()) + 1);
  LOG_DEBUG("Round trip: {} candidates, best {:.0f}m = {:.0f} + {:.0f} + {:.0f}, overlap {:.2f}, "
            "score {:.3f}",
            candidates.size(), candidates.front().distance, out_legs[candidates.front().a].distance,
            middle_legs[candidates.front().a * b_turns.size() + candidates.front().b].distance,
            back_legs[candidates.front().b].distance, candidates.front().overlap,
            candidates.front().score);

  // the round trip starts and ends at the start
  options.mutable_locations()->Clear();
  options.mutable_locations()->Add()->CopyFrom(start);
  options.mutable_locations()->Add()->CopyFrom(start);

  for (size_t r = 0; r < num_results; ++r) {
    const auto& c = candidates[r];
    std::vector<GraphId> edges = out_legs[c.a].edges;
    const auto& middle = middle_legs[c.a * b_turns.size() + c.b].edges;
    edges.insert(edges.end(), middle.begin(), middle.end());
    edges.insert(edges.end(), back_legs[c.b].edges.begin(), back_legs[c.b].edges.end());

    const int routes_before = request.trip().routes_size();
    try {
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
      recost_forward(*reader, *mode_costing[int(mode)], edge_cb, label_cb,
                     find_percent_along(start, edges.front()),
                     find_percent_along(start, edges.back()), TimeInfo::invalid(), false, true);

      auto origin = options.locations(0);
      auto destination = options.locations(1);
      auto* route = request.mutable_trip()->mutable_routes()->Add();
      auto& leg = *route->mutable_legs()->Add();
      TripLegBuilder::Build(options, controller, *reader, mode_costing, path.begin(), path.end(),
                            origin, destination, leg, {"round_trip"}, interrupt);

      if (options.verbose()) {
        int group = 1;
        for (const auto* turn : {&a_turns[c.a], &b_turns[c.b]}) {
          auto* l = route->mutable_round_trip_seeds()->Add();
          l->mutable_ll()->set_lng(turn->ll.lng());
          l->mutable_ll()->set_lat(turn->ll.lat());
          l->set_round_trip_group_index(group++);
        }
      }
    } catch (const std::exception& e) {
      // if the best candidate fails, the next one simply becomes the first route
      LOG_DEBUG("Round trip candidate {} failed to build: {}", r, e.what());
      while (request.trip().routes_size() > routes_before) {
        request.mutable_trip()->mutable_routes()->RemoveLast();
      }
    }
  }

  if (request.trip().routes_size() == 0) {
    throw valhalla_exception_t{442};
  }
}

} // namespace thor
} // namespace valhalla
