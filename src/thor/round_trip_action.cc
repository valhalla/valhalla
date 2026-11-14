#include "baldr/complexrestriction.h"
#include "baldr/graphreader.h"
#include "baldr/tilehierarchy.h"
#include "midgard/logging.h"
#include "sif/recost.h"
#include "thor/bidirectional_astar.h"
#include "thor/dijkstras.h"
#include "thor/triplegbuilder.h"
#include "thor/worker.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace {

// A round trip is a triangle: start -> a -> b -> start. The first leg comes from a forward expansion
// from the start, which also finds the turning points a. A reverse expansion into the start finds
// the turning points b. The middle leg a -> b is routed with the first leg penalized, the last leg
// b -> start with the first and the middle leg penalized, so that each leg only reuses or crosses the
// roads of the earlier ones if there is no other way.

// each leg is about a third of the round trip, the expansions record turning points over a wider
// band of network distance so that pairs of them cover a range of round trip lengths
constexpr double kMinLegFraction = 0.1;
constexpr double kMaxLegFraction = 0.47;

// near the start the bearing of a node says little about where the path goes, so the expansions are
// only kept inside their cone beyond this distance, and with some slack so paths can bend around
// obstacles
constexpr double kConeStartDistance = 3000.;
constexpr double kConeMargin = 25.;

// turning points are recorded on a grid of distance and bearing, keeping the first junction in each
// cell
constexpr int kDistanceBins = 10;
constexpr int kBearingBins = 4;

// the inner angle of an equilateral triangle, the ideal opening at the start
constexpr double kIdealOpening = 60.;
// how much the shape of the triangle counts when ranking pairs of turning points, next to the
// expected deviation from the requested distance
constexpr double kShapeWeight = 0.02;

// a middle leg is expected to be the beeline between its turning points times this factor, until
// the first middle legs tell how much the network detours in this area
constexpr double kInitialDetourFactor = 1.3;

// a turning point reached on another edge this much later than the node itself was reached drives a
// detour to come back to it, e.g. around a roundabout
constexpr double kMaxFirstReachDetour = 500.;

// how many pairs of turning points get routed before the best round trip so far is taken
constexpr size_t kMaxAttempts = 8;

// cost factor for the edges earlier legs drive, in either direction
constexpr double kUsedEdgeFactor = 10.;

// a round trip retracing more than this fraction of its middle and last leg isn't acceptable
constexpr double kMaxOverlap = 0.1;

// Crossing an earlier leg doesn't retrace anything, but the bow tie it draws is confusing. A leg
// pays this factor on the edges at the junctions of earlier legs, but only for this distance next to
// the junction, so that a crossing costs about the same wherever it is.
constexpr double kCrossingEdgeFactor = 4.;
constexpr double kCrossingPenaltyLength = 100.;
// how much a crossing counts against a round trip, in terms of relative deviation from the
// requested distance
constexpr double kCrossingScore = 0.02;
// how much the cost per distance counts against a round trip, relative to the cheapest one per
// distance, so that preferences of the costing like curvy or quiet roads carry through
constexpr double kCostPerDistanceWeight = 0.25;
// A round trip whose shape crosses itself isn't used at all, including crossings on bridges that
// share no junction. Only near the start, where the first and last leg can't avoid each other, it
// may cross itself, within this fraction of its length from either end.
constexpr double kCrossingFreeStartFraction = 0.05;
// cell size of the grid the self intersection test sorts the shape's segments into
constexpr double kIntersectionCellSize = 100.;

// alternatives to the middle and last leg widen the range of lengths each pair of turning points
// offers
constexpr uint32_t kLegAlternates = 4;
constexpr float kAlternativeCostExtend = 1.5f;
constexpr uint32_t kAlternativeIterationsDelta = 200000;

// leaving a node at less than this angle to the road it was arrived on doubles back on that road,
// e.g. onto the other carriageway of a divided road
constexpr double kMaxDoubleBackAngle = 35.;

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

// the same node on the local level, so a node compares equal on whichever level a path passes it
GraphId local_node(const GraphId& node, GraphReader& reader) {
  const auto local_level = TileHierarchy::levels().back().level;
  if (node.level() == local_level) {
    return node;
  }
  graph_tile_ptr tile;
  const auto* info = reader.nodeinfo(node, tile);
  if (!info) {
    return node;
  }
  for (uint32_t i = 0; i < info->transition_count(); ++i) {
    const auto end = tile->transition(info->transition_index() + i)->endnode();
    if (end.level() == local_level) {
      return end;
    }
  }
  return node;
}

// a node where the round trip can turn towards its next leg
struct Turn {
  uint32_t label;
  GraphId node;
  PointLL ll;
  double distance;
  double bearing;
  // whether more than one road meets here. Elsewhere the round trip can only continue straight on
  bool junction;
};

// An expansion from or into the start that only grows inside a cone around one corner of the
// triangle. It collects the turning points for that corner on the way, but only where the
// expansion's path to the node is about the shortest one to reach it at all.
class LoopTree : public Dijkstras {
public:
  struct Params {
    PointLL start;
    double min_distance;
    double max_distance;
    double cone_center;
    double cone_half_width;
    bool motor_vehicle;
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
    use_hierarchy_limits_ = costings[static_cast<uint32_t>(mode)]->UseHierarchyLimits();
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
    // the expansion settles in cost order, so the first label at a node is the cheapest way there
    const auto first_reach =
        first_reach_.try_emplace(local_node(pred.endnode(), reader), distance).first->second;

    const PointLL ll = node->latlng(tile->header()->base_ll());
    const double bearing = params_.start.Heading(ll);
    const double off_center = angle_diff(bearing, params_.cone_center);
    if (distance > kConeStartDistance && off_center > params_.cone_half_width + kConeMargin) {
      return ExpansionRecommendation::prune_expansion;
    }

    if (distance >= params_.min_distance && off_center <= params_.cone_half_width &&
        node->edge_count() >= 2 && distance <= first_reach + kMaxFirstReachDetour) {
      const uint32_t label = edgestatus_.Get(pred.edgeid(), pred.path_id()).index();
      graph_tile_ptr edge_tile;
      const auto* edge = reader.directededge(DrivingEdge(label), edge_tile);
      if (edge && Usable(edge)) {
        const int distance_bin =
            std::clamp(static_cast<int>((distance - params_.min_distance) /
                                        (params_.max_distance - params_.min_distance) *
                                        kDistanceBins),
                       0, kDistanceBins - 1);
        const int bearing_bin =
            std::clamp(static_cast<int>(
                           (signed_angle(params_.cone_center, bearing) + params_.cone_half_width) /
                           (2. * params_.cone_half_width) * kBearingBins),
                       0, kBearingBins - 1);
        const Turn turn{label, pred.endnode(), ll, distance, bearing, node->edge_count() >= 3};
        auto [cell, inserted] = turns_.try_emplace(distance_bin * kBearingBins + bearing_bin, turn);
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
  // per node on the local level the distance at which the expansion reached it first
  std::unordered_map<GraphId, double> first_reach_;
};

// Bidirectional A* for the middle and last leg. Without shortcuts no penalized edge can hide inside
// one, and the wider search for alternatives offers more lengths per pair of turning points.
class LegAStar : public BidirectionalAStar {
public:
  LegAStar() : BidirectionalAStar() {
    allow_shortcuts_ = false;
    alternative_cost_extend_ = kAlternativeCostExtend;
    alternative_iterations_delta_ = kAlternativeIterationsDelta;
  }
};

// whether arriving at a node on the opposing edge of `back` and leaving on `out`, both edges leaving
// that node, turns around on the road
bool doubles_back(const NodeInfo* node, const DirectedEdge* back, const DirectedEdge* out) {
  if (back->localedgeidx() > kMaxLocalEdgeIndex || out->localedgeidx() > kMaxLocalEdgeIndex) {
    return back == out;
  }
  return angle_diff(node->heading(back->localedgeidx()), node->heading(out->localedgeidx())) <
         kMaxDoubleBackAngle;
}

// A location at a turning point: the edges leaving it as the origin of the next leg, or the edges
// arriving at it as the destination of a leg. Left out are edges the costing doesn't allow and, at an
// origin, edges that would turn the round trip around after `arrival` or that a simple turn
// restriction forbids after it.
Location turn_location(const Turn& turn,
                       const GraphId& arrival,
                       bool origin,
                       GraphReader& reader,
                       const DynamicCost& costing) {
  Location location;
  location.mutable_ll()->set_lng(turn.ll.lng());
  location.mutable_ll()->set_lat(turn.ll.lat());
  graph_tile_ptr tile;
  const auto* node = reader.nodeinfo(turn.node, tile);
  if (!node) {
    return location;
  }
  graph_tile_ptr arrival_tile;
  graph_tile_ptr back_tile;
  const DirectedEdge* arrival_edge = nullptr;
  const DirectedEdge* back = nullptr;
  if (origin) {
    arrival_edge = reader.directededge(arrival, arrival_tile);
    back = reader.GetOpposingEdge(arrival, back_tile);
    if (!arrival_edge) {
      return location;
    }
  }

  GraphId edge_id(turn.node.tileid(), turn.node.level(), node->edge_index());
  for (uint32_t i = 0; i < node->edge_count(); ++i, ++edge_id) {
    const auto* edge = tile->directededge(edge_id);
    if (edge->is_shortcut()) {
      continue;
    }
    GraphId id = edge_id;
    if (origin) {
      if (!costing.Allowed(edge, tile) || (back && doubles_back(node, back, edge)) ||
          (arrival_edge->restrictions() & (1 << edge->localedgeidx()))) {
        continue;
      }
    } else {
      graph_tile_ptr opp_tile = tile;
      id = reader.GetOpposingEdgeId(edge_id, opp_tile);
      const auto* inbound = id.is_valid() ? opp_tile->directededge(id) : nullptr;
      if (!inbound || !costing.Allowed(inbound, opp_tile)) {
        continue;
      }
    }
    auto* path_edge = location.mutable_correlation()->mutable_edges()->Add();
    path_edge->set_graph_id(id);
    path_edge->set_percent_along(origin ? 0. : 1.);
    path_edge->mutable_ll()->CopyFrom(location.ll());
    path_edge->set_distance(0.);
    path_edge->set_begin_node(origin);
    path_edge->set_end_node(!origin);
  }
  return location;
}

// whether a round trip arriving on `arrival` may continue on `departure`, by the same rules as
// turn_location
bool turn_allowed(const GraphId& arrival, const GraphId& departure, GraphReader& reader) {
  graph_tile_ptr arrival_tile;
  graph_tile_ptr back_tile;
  graph_tile_ptr departure_tile;
  graph_tile_ptr node_tile;
  const auto* arrival_edge = reader.directededge(arrival, arrival_tile);
  const auto* back = reader.GetOpposingEdge(arrival, back_tile);
  const auto* departure_edge = reader.directededge(departure, departure_tile);
  const auto* node = arrival_edge ? reader.nodeinfo(arrival_edge->endnode(), node_tile) : nullptr;
  if (!arrival_edge || !departure_edge || !node) {
    return false;
  }
  return !(arrival_edge->restrictions() & (1 << departure_edge->localedgeidx())) &&
         !(back && doubles_back(node, back, departure_edge));
}

// Whether a complex restriction runs across the joint in front of edges[joint]. Restrictions on
// either side were respected by the search that found that side. Timed and probable restrictions
// count as active, a round trip has no time to evaluate them at.
bool restricted_across(const std::vector<GraphId>& edges,
                       size_t joint,
                       uint64_t access_mode,
                       GraphReader& reader) {
  graph_tile_ptr tile;
  const size_t end = std::min(edges.size(), joint + kMaxViasPerRestriction + 1);
  for (size_t k = joint; k < end; ++k) {
    const auto* edge = reader.directededge(edges[k], tile);
    if (!edge || !(edge->end_restriction() & access_mode)) {
      continue;
    }
    for (const auto& cr : tile->GetComplexRestrictions(true, edges[k], access_mode)) {
      // the vias come in reverse driving order, followed by the edge the restriction starts on
      size_t i = k;
      bool match = true;
      cr.WalkVias([&](const GraphId* via) {
        if (i == 0 || edges[i - 1] != *via) {
          match = false;
          return WalkingVia::StopWalking;
        }
        --i;
        return WalkingVia::KeepWalking;
      });
      if (match && i > 0 && i - 1 < joint && edges[i - 1] == cr.from_graphid()) {
        return true;
      }
    }
  }
  return false;
}

// a leg of a round trip: its edges, their lengths, their end nodes on the local level and its shape
struct Leg {
  std::vector<GraphId> edges;
  std::vector<double> lengths;
  std::vector<GraphId> nodes;
  std::vector<PointLL> shape;
  double distance = 0.;
  // without the round trip's penalties
  double cost = 0.;
};

Leg make_leg(std::vector<GraphId> edges, double distance, GraphReader& reader) {
  Leg leg;
  leg.edges = std::move(edges);
  leg.distance = distance;
  leg.lengths.reserve(leg.edges.size());
  leg.nodes.reserve(leg.edges.size());
  graph_tile_ptr tile;
  for (const auto& edge_id : leg.edges) {
    const auto* edge = reader.directededge(edge_id, tile);
    leg.lengths.push_back(edge ? edge->length() : 0.);
    leg.nodes.push_back(edge ? local_node(edge->endnode(), reader) : GraphId{});
    if (!edge) {
      continue;
    }
    std::vector<PointLL> shape = tile->edgeinfo(edge).shape();
    if (!edge->forward()) {
      std::reverse(shape.begin(), shape.end());
    }
    const bool connected = !leg.shape.empty() && !shape.empty() && leg.shape.back() == shape.front();
    leg.shape.insert(leg.shape.end(), shape.begin() + (connected ? 1 : 0), shape.end());
  }
  return leg;
}

// Whether a shape crosses itself between `skip` meters from its start and `skip` meters from its
// end: two of its segments cross, or it passes a point twice and the second pass goes from one side
// of the first pass to the other. Passing a point twice on the same side, or along the same road,
// doesn't cross.
bool crosses_itself(const std::vector<PointLL>& shape, double skip) {
  if (shape.size() < 4) {
    return false;
  }
  // local planar coordinates in meters
  const double lng_scale = std::cos(shape.front().lat() * kRadPerDegD) * kMetersPerDegreeLat;
  std::vector<Point2> points;
  points.reserve(shape.size());
  for (const auto& p : shape) {
    points.emplace_back(p.lng() * lng_scale, p.lat() * kMetersPerDegreeLat);
  }
  std::vector<double> along(points.size(), 0.);
  for (size_t i = 1; i < points.size(); ++i) {
    along[i] = along[i - 1] + points[i - 1].Distance(points[i]);
  }
  const double begin = skip;
  const double end = along.back() - skip;
  if (end <= begin) {
    return false;
  }

  const auto cross = [](const Point2& o, const Point2& a, const Point2& b) {
    return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
  };
  const auto cell = [](double x, double y) {
    return static_cast<uint64_t>(static_cast<int64_t>(std::floor(x / kIntersectionCellSize))) << 32 ^
           static_cast<uint32_t>(static_cast<int64_t>(std::floor(y / kIntersectionCellSize)));
  };

  // segment i runs from points[i] to points[i + 1], each goes into every cell it passes
  std::unordered_map<uint64_t, std::vector<size_t>> grid;
  for (size_t i = 0; i + 1 < points.size(); ++i) {
    if (along[i + 1] < begin || along[i] > end) {
      continue;
    }
    const auto& a = points[i];
    const auto& b = points[i + 1];
    const int steps = static_cast<int>(a.Distance(b) / (kIntersectionCellSize / 2.)) + 1;
    uint64_t previous = 0;
    for (int step = 0; step <= steps; ++step) {
      const double f = static_cast<double>(step) / steps;
      const auto c = cell(a.x() + f * (b.x() - a.x()), a.y() + f * (b.y() - a.y()));
      if (step == 0 || c != previous) {
        auto& segments = grid[c];
        if (segments.empty() || segments.back() != i) {
          segments.push_back(i);
        }
      }
      previous = c;
    }
  }

  // segments crossing each other, touching ones are left to the point check below
  for (const auto& [c, segments] : grid) {
    for (size_t x = 0; x < segments.size(); ++x) {
      for (size_t y = x + 1; y < segments.size(); ++y) {
        const size_t i = std::min(segments[x], segments[y]);
        const size_t j = std::max(segments[x], segments[y]);
        if (j <= i + 1) {
          continue;
        }
        const auto &a = points[i], &b = points[i + 1], &c1 = points[j], &d = points[j + 1];
        if (cross(c1, d, a) * cross(c1, d, b) < 0. && cross(a, b, c1) * cross(a, b, d) < 0.) {
          return true;
        }
      }
    }
  }

  // points passed twice, compared by the directions in and out of them on both passes
  std::unordered_map<PointLL, std::vector<size_t>> passes;
  for (size_t i = 1; i + 1 < shape.size(); ++i) {
    if (along[i] >= begin && along[i] <= end) {
      passes[shape[i]].push_back(i);
    }
  }
  const auto direction = [&](size_t at, size_t to) {
    return std::atan2(points[to].y() - points[at].y(), points[to].x() - points[at].x());
  };
  // whether angle t lies strictly between from and to, counter clockwise
  const auto between = [](double from, double to, double t) {
    const double span = std::fmod(to - from + 4. * kPiD, 2. * kPiD);
    const double offset = std::fmod(t - from + 4. * kPiD, 2. * kPiD);
    return offset > 0. && offset < span;
  };
  constexpr double kSameDirection = 1. * kRadPerDegD;
  for (const auto& [point, indices] : passes) {
    for (size_t x = 0; x < indices.size(); ++x) {
      for (size_t y = x + 1; y < indices.size(); ++y) {
        const size_t i = indices[x];
        const size_t j = indices[y];
        if (j <= i + 1) {
          continue;
        }
        const double a1 = direction(i, i - 1), a2 = direction(i, i + 1);
        const double b1 = direction(j, j - 1), b2 = direction(j, j + 1);
        // a pass sharing a direction with the other one runs along the same road there
        bool shared = false;
        for (double a : {a1, a2}) {
          for (double b : {b1, b2}) {
            shared |= std::abs(std::remainder(a - b, 2. * kPiD)) < kSameDirection;
          }
        }
        if (!shared && between(a1, a2, b1) != between(a1, a2, b2)) {
          return true;
        }
      }
    }
  }
  return false;
}

// the roads a leg drives, i.e. its edges and their opposing edges
void add_roads(const Leg& leg, std::unordered_set<GraphId>& roads, GraphReader& reader) {
  graph_tile_ptr tile;
  for (const auto& edge_id : leg.edges) {
    roads.insert(edge_id);
    const auto opp_id = reader.GetOpposingEdgeId(edge_id, tile);
    if (opp_id.is_valid()) {
      roads.insert(opp_id);
    }
  }
}

// The junctions a leg passes, without the first and last few where the next legs connect to it
void add_junctions(const Leg& leg,
                   size_t skip_front,
                   size_t skip_back,
                   std::unordered_set<GraphId>& junctions) {
  for (size_t i = skip_front; i + skip_back < leg.nodes.size(); ++i) {
    junctions.insert(leg.nodes[i]);
  }
}

// how much of a leg runs on roads of earlier legs, and how many of their junctions it passes
struct Reuse {
  double overlap = 0.;
  uint32_t crossings = 0;
};

Reuse measure_reuse(const Leg& leg,
                    const std::unordered_set<GraphId>& roads,
                    const std::unordered_set<GraphId>& junctions) {
  Reuse reuse;
  for (size_t i = 0; i < leg.edges.size(); ++i) {
    if (roads.contains(leg.edges[i])) {
      reuse.overlap += leg.lengths[i];
    }
    // passing a junction while retracing an earlier leg is overlap, not a crossing
    if (i + 1 < leg.edges.size() && junctions.contains(leg.nodes[i]) &&
        !(roads.contains(leg.edges[i]) && roads.contains(leg.edges[i + 1]))) {
      ++reuse.crossings;
    }
  }
  return reuse;
}

// Penalizes the edges at the junctions of earlier legs, except those of the legs themselves, over
// kCrossingPenaltyLength next to the junction: the start of the edges leaving it and the end of the
// edges arriving at it.
void add_crossing_penalties(const std::unordered_set<GraphId>& junctions,
                            const std::unordered_set<GraphId>& used,
                            GraphReader& reader,
                            Costing_Options& penalties) {
  // per edge the fraction penalized from its start and the one penalized up to its end
  std::unordered_map<GraphId, std::pair<double, double>> ends;
  graph_tile_ptr tile;
  const auto add_node = [&](const GraphId& node_id) {
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
      const double fraction = std::min(1., kCrossingPenaltyLength / edge->length());
      if (!used.contains(edge_id)) {
        auto& end = ends[edge_id];
        end.first = std::max(end.first, fraction);
      }
      graph_tile_ptr opp_tile = tile;
      const auto opp_id = reader.GetOpposingEdgeId(edge_id, opp_tile);
      if (opp_id.is_valid() && !used.contains(opp_id)) {
        auto& end = ends[opp_id];
        end.second = std::max(end.second, fraction);
      }
    }
  };
  for (const auto& junction : junctions) {
    add_node(junction);
    // the same junction on the other levels, a path might pass it on any of them
    const auto* node = reader.nodeinfo(junction, tile);
    if (!node) {
      continue;
    }
    std::vector<GraphId> others;
    for (uint32_t i = 0; i < node->transition_count(); ++i) {
      others.push_back(tile->transition(node->transition_index() + i)->endnode());
    }
    for (const auto& other : others) {
      add_node(other);
    }
  }

  const auto add_range = [&penalties](const GraphId& edge_id, double start, double end) {
    auto* factor = penalties.mutable_cost_factor_edges()->Add();
    factor->set_id(edge_id.value);
    factor->set_factor(kCrossingEdgeFactor);
    factor->set_start(start);
    factor->set_end(end);
  };
  for (const auto& [edge_id, end] : ends) {
    if (end.first + end.second >= 1.) {
      add_range(edge_id, 0., 1.);
      continue;
    }
    if (end.first > 0.) {
      add_range(edge_id, 0., end.first);
    }
    if (end.second > 0.) {
      add_range(edge_id, 1. - end.second, 1.);
    }
  }
}

// the cost of driving these edges without any of the round trip's penalties
double unpenalized_cost(const std::vector<GraphId>& edges,
                        float source_pct,
                        float target_pct,
                        GraphReader& reader,
                        const DynamicCost& costing) {
  double cost = 0.;
  auto edge_itr = edges.begin();
  const auto edge_cb = [&edge_itr, &edges]() {
    return edge_itr == edges.end() ? GraphId{} : *edge_itr++;
  };
  const auto label_cb = [&cost](const PathEdgeLabel& label) { cost = label.cost().cost; };
  recost_forward(reader, costing, edge_cb, label_cb, source_pct, target_pct, TimeInfo::invalid(),
                 false, true);
  return cost;
}

// where along an edge a location lies, or `fallback` if the edge isn't one of its edges
float percent_along(const Location& location, const GraphId& edge_id, float fallback) {
  for (const auto& edge : location.correlation().edges()) {
    if (edge.graph_id() == edge_id) {
      return edge.percent_along();
    }
  }
  return fallback;
}

// the user's own cost factors plus the penalties for driving on or crossing earlier legs
Costing_Options make_penalties(const Costing_Options& costing_options,
                               const std::unordered_set<GraphId>& roads,
                               const std::unordered_set<GraphId>& junctions,
                               GraphReader& reader) {
  Costing_Options penalties;
  penalties.mutable_cost_factor_edges()->CopyFrom(costing_options.cost_factor_edges());
  for (const auto& edge_id : roads) {
    auto* factor = penalties.mutable_cost_factor_edges()->Add();
    factor->set_id(edge_id.value);
    factor->set_factor(kUsedEdgeFactor);
    factor->set_start(0.);
    factor->set_end(1.);
  }
  add_crossing_penalties(junctions, roads, reader, penalties);
  return penalties;
}

float find_percent_along(const Location& location, const GraphId& edge_id) {
  for (const auto& edge : location.correlation().edges()) {
    if (edge.graph_id() == edge_id) {
      return edge.percent_along();
    }
  }
  throw std::logic_error("Round trip doesn't start or end on one of the start's edges");
}

// a pair of turning points, ranked by what the round trip through them is expected to look like
struct TurnPair {
  size_t a;
  size_t b;
  double beeline;
  double score;
};

// a round trip through a pair of turning points
struct Candidate {
  size_t a;
  size_t b;
  std::vector<GraphId> edges;
  double distance;
  double cost;
  double deviation;   // relative to the requested distance
  double overlap;     // fraction of the middle and last leg driven on roads of earlier legs
  uint32_t crossings; // junctions of earlier legs the middle and last leg cross
  double score;
};

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

  const Location start = options.locations(0);
  const auto& round_trip_options = options.round_trip_options();
  const double desired_distance = round_trip_options.distance();
  const double heading = round_trip_options.heading();
  const double tolerance = round_trip_options.heading_tolerance();
  const double distance_tolerance = round_trip_options.distance_tolerance();
  const double overlap_factor = round_trip_options.overlap_factor();
  const bool motor_vehicle = mode == travel_mode_t::kDrive;

  // the first turning point lies in the counter clockwise half of the heading's sector and the second
  // one in the clockwise half, so the round trip runs clockwise
  boost::property_tree::ptree tree_config;
  tree_config.put("max_reserved_labels_count_dijkstras", 200000);
  const PointLL start_ll(start.ll().lng(), start.ll().lat());
  LoopTree out_tree(tree_config,
                    {start_ll, kMinLegFraction * desired_distance, kMaxLegFraction * desired_distance,
                     heading - tolerance / 2., tolerance / 2., motor_vehicle});
  LoopTree back_tree(tree_config, {start_ll, kMinLegFraction * desired_distance,
                                   kMaxLegFraction * desired_distance, heading + tolerance / 2.,
                                   tolerance / 2., motor_vehicle});
  out_tree.Run(ExpansionType::forward, start, *reader, mode_costing, mode);
  back_tree.Run(ExpansionType::reverse, start, *reader, mode_costing, mode);

  const auto a_turns = out_tree.Turns();
  const auto b_turns = back_tree.Turns();
  LOG_DEBUG("Round trip turning points: {} outbound, {} inbound, expansions settled {} and {} edges",
            a_turns.size(), b_turns.size(), out_tree.LabelCount(), back_tree.LabelCount());
  if (a_turns.empty() || b_turns.empty()) {
    throw valhalla_exception_t{442};
  }

  // pairs are ranked by the round trip length they are expected to give and by how well the triangle
  // opens and points into the heading, the best one last
  std::vector<TurnPair> pairs;
  pairs.reserve(a_turns.size() * b_turns.size());
  for (size_t i = 0; i < a_turns.size(); ++i) {
    for (size_t j = 0; j < b_turns.size(); ++j) {
      pairs.push_back({i, j, a_turns[i].ll.Distance(b_turns[j].ll), 0.});
    }
  }
  double detour_factor = kInitialDetourFactor;
  // the last leg avoids the earlier ones, so it is expected to come out somewhat longer than the
  // reverse expansion's path to its turning point, by this factor learned from the last legs so far
  double back_factor = 1.;
  const auto rank_pairs = [&]() {
    for (auto& pair : pairs) {
      const auto& a = a_turns[pair.a];
      const auto& b = b_turns[pair.b];
      const double expected = a.distance + detour_factor * pair.beeline + back_factor * b.distance;
      const double opening = signed_angle(a.bearing, b.bearing);
      const double bisector = a.bearing + opening / 2.;
      const double shape = (std::abs(opening - kIdealOpening) + angle_diff(bisector, heading)) / 90.;
      pair.score = std::abs(expected - desired_distance) / desired_distance + kShapeWeight * shape;
    }
    std::sort(pairs.begin(), pairs.end(),
              [](const TurnPair& x, const TurnPair& y) { return x.score > y.score; });
  };

  // routes one leg with the given penalties, the best path and its alternatives
  Options leg_options = options;
  leg_options.set_alternates(kLegAlternates);
  LegAStar astar;
  const auto route = [&](Location& origin, Location& destination, const Costing_Options& penalties) {
    std::vector<Leg> legs;
    if (origin.correlation().edges_size() == 0 || destination.correlation().edges_size() == 0) {
      return legs;
    }
    costing->SetCostFactorEdges(penalties);
    astar.Clear();
    const auto paths =
        astar.GetBestPath(origin, destination, *reader, mode_costing, mode, leg_options);
    costing->SetCostFactorEdges(costing_options);
    for (const auto& path : paths) {
      if (path.empty()) {
        continue;
      }
      std::vector<GraphId> edges;
      edges.reserve(path.size());
      for (const auto& info : path) {
        edges.push_back(info.edgeid);
      }
      auto leg = make_leg(std::move(edges), path.back().path_distance, *reader);
      try {
        leg.cost =
            unpenalized_cost(leg.edges, percent_along(origin, leg.edges.front(), 0.f),
                             percent_along(destination, leg.edges.back(), 1.f), *reader, *costing);
      } catch (const std::exception&) { leg.cost = path.back().elapsed_cost.cost; }
      legs.push_back(std::move(leg));
    }
    return legs;
  };

  std::vector<Candidate> candidates;
  double detour_sum = 0.;
  size_t detour_count = 0;
  double back_sum = 0.;
  size_t back_count = 0;

  for (size_t attempt = 0; attempt < kMaxAttempts && !pairs.empty(); ++attempt) {
    rank_pairs();
    const auto pair = pairs.back();
    pairs.pop_back();
    const auto& a = a_turns[pair.a];
    const auto& b = b_turns[pair.b];
    auto first = make_leg(out_tree.Path(a.label), out_tree.Label(a.label).path_distance(), *reader);
    first.cost = out_tree.Label(a.label).cost().cost;

    // the middle leg avoids the first one, except where it connects to it at a
    std::unordered_set<GraphId> first_roads;
    add_roads(first, first_roads, *reader);
    std::unordered_set<GraphId> first_junctions;
    add_junctions(first, 2, 2, first_junctions);
    auto origin = turn_location(a, first.edges.back(), true, *reader, *costing);
    auto destination = turn_location(b, {}, false, *reader, *costing);
    const auto middles =
        route(origin, destination,
              make_penalties(costing_options, first_roads, first_junctions, *reader));
    if (middles.empty()) {
      LOG_DEBUG("Round trip attempt {}: no middle leg", attempt);
      continue;
    }
    // the best middle leg tells how much longer than the beeline middle legs come out here
    if (pair.beeline > 0.) {
      detour_sum += middles.front().distance / pair.beeline;
      detour_factor = detour_sum / ++detour_count;
    }

    // the last leg avoids the first and the best middle leg, except where it connects to them at b
    // and at the start
    auto last_roads = first_roads;
    add_roads(middles.front(), last_roads, *reader);
    auto last_junctions = first_junctions;
    add_junctions(middles.front(), 0, 2, last_junctions);
    auto last_origin = turn_location(b, middles.front().edges.back(), true, *reader, *costing);
    auto last_destination = start;
    const auto lasts = route(last_origin, last_destination,
                             make_penalties(costing_options, last_roads, last_junctions, *reader));
    if (lasts.empty()) {
      LOG_DEBUG("Round trip attempt {}: no last leg", attempt);
      continue;
    }
    if (b.distance > 0.) {
      back_sum += lasts.front().distance / b.distance;
      back_factor = back_sum / ++back_count;
    }

    bool acceptable = false;
    for (const auto& middle : middles) {
      const auto middle_reuse = measure_reuse(middle, first_roads, first_junctions);
      auto roads = first_roads;
      add_roads(middle, roads, *reader);
      auto junctions = first_junctions;
      add_junctions(middle, 0, 2, junctions);

      for (const auto& last : lasts) {
        if (!turn_allowed(middle.edges.back(), last.edges.front(), *reader)) {
          continue;
        }
        Candidate candidate{pair.a, pair.b, first.edges, 0., 0., 0., 0., 0, 0.};
        candidate.edges.insert(candidate.edges.end(), middle.edges.begin(), middle.edges.end());
        candidate.edges.insert(candidate.edges.end(), last.edges.begin(), last.edges.end());
        const size_t middle_begin = first.edges.size();
        const size_t last_begin = middle_begin + middle.edges.size();
        if (restricted_across(candidate.edges, middle_begin, costing->access_mode(), *reader) ||
            restricted_across(candidate.edges, last_begin, costing->access_mode(), *reader)) {
          continue;
        }

        std::vector<PointLL> shape = first.shape;
        shape.insert(shape.end(), middle.shape.begin(), middle.shape.end());
        shape.insert(shape.end(), last.shape.begin(), last.shape.end());
        const double total = first.distance + middle.distance + last.distance;
        if (crosses_itself(shape, kCrossingFreeStartFraction * total)) {
          continue;
        }

        const auto last_reuse = measure_reuse(last, roads, junctions);
        const double later_distance = middle.distance + last.distance;
        candidate.distance = first.distance + later_distance;
        candidate.cost = first.cost + middle.cost + last.cost;
        candidate.overlap =
            later_distance > 0. ? (middle_reuse.overlap + last_reuse.overlap) / later_distance : 0.;
        candidate.crossings = middle_reuse.crossings + last_reuse.crossings;
        candidate.deviation = std::abs(candidate.distance - desired_distance) / desired_distance;
        acceptable |= candidate.deviation <= distance_tolerance && candidate.overlap <= kMaxOverlap &&
                      candidate.crossings == 0;
        candidates.push_back(std::move(candidate));
      }
    }
    LOG_DEBUG("Round trip attempt {}: legs of {:.0f}m, {:.0f}m and {:.0f}m, {} middle and {} last "
              "legs, detour factor now {:.2f}, last leg factor {:.2f}",
              attempt, first.distance, middles.front().distance, lasts.front().distance,
              middles.size(), lasts.size(), detour_factor, back_factor);
    if (acceptable) {
      break;
    }
  }
  if (candidates.empty()) {
    throw valhalla_exception_t{442};
  }
  double min_cost_per_distance = std::numeric_limits<double>::max();
  for (const auto& candidate : candidates) {
    min_cost_per_distance = std::min(min_cost_per_distance, candidate.cost / candidate.distance);
  }
  for (auto& candidate : candidates) {
    const double cost_per_distance = candidate.cost / candidate.distance / min_cost_per_distance;
    candidate.score = candidate.deviation + overlap_factor * candidate.overlap +
                      kCrossingScore * static_cast<double>(candidate.crossings) +
                      kCostPerDistanceWeight * (cost_per_distance - 1.);
  }
  std::sort(candidates.begin(), candidates.end(),
            [](const Candidate& x, const Candidate& y) { return x.score < y.score; });
  LOG_DEBUG("Round trip: {} candidates, best {:.0f}m, overlap {:.2f}, {} crossings, score {:.3f}",
            candidates.size(), candidates.front().distance, candidates.front().overlap,
            candidates.front().crossings, candidates.front().score);

  // the round trip starts and ends at the start
  options.mutable_locations()->Clear();
  options.mutable_locations()->Add()->CopyFrom(start);
  options.mutable_locations()->Add()->CopyFrom(start);

  const int num_results = static_cast<int>(options.alternates()) + 1;
  for (const auto& candidate : candidates) {
    if (request.trip().routes_size() >= num_results) {
      break;
    }
    const auto& edges = candidate.edges;
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
      // with access checked, a turn the costing doesn't allow at one of the joints throws here
      recost_forward(*reader, *costing, edge_cb, label_cb, find_percent_along(start, edges.front()),
                     find_percent_along(start, edges.back()), TimeInfo::invalid(), false, false);

      auto origin = options.locations(0);
      auto destination = options.locations(1);
      auto* route = request.mutable_trip()->mutable_routes()->Add();
      auto& leg = *route->mutable_legs()->Add();
      TripLegBuilder::Build(options, controller, *reader, mode_costing, path.begin(), path.end(),
                            origin, destination, leg, {"round_trip"}, interrupt);

      if (options.verbose()) {
        int group = 1;
        for (const auto* turn : {&a_turns[candidate.a], &b_turns[candidate.b]}) {
          auto* seed = route->mutable_round_trip_seeds()->Add();
          seed->mutable_ll()->set_lng(turn->ll.lng());
          seed->mutable_ll()->set_lat(turn->ll.lat());
          seed->set_round_trip_group_index(group++);
        }
      }
    } catch (const std::exception& e) {
      // if a candidate fails, the next one takes its place
      LOG_DEBUG("Round trip candidate failed to build: {}", e.what());
      {
        size_t lo = 1, hi = edges.size();
        while (lo < hi) {
          size_t mid = (lo + hi) / 2;
          bool ok = true;
          try {
            auto it2 = edges.begin();
            auto end2 = edges.begin() + mid;
            recost_forward(
                *reader, *costing, [&]() { return it2 == end2 ? GraphId{} : *it2++; },
                [](const PathEdgeLabel&) {}, find_percent_along(start, edges.front()), 1.f,
                TimeInfo::invalid(), false, false);
          } catch (...) { ok = false; }
          if (ok)
            lo = mid + 1;
          else
            hi = mid;
        }
        graph_tile_ptr t;
        const auto* e1 = reader->directededge(edges[lo - 1], t);
        const auto* e0 = lo >= 2 ? reader->directededge(edges[lo - 2], t) : nullptr;
        LOG_DEBUG(
            "RTDBG first failing prefix {} of {} (out leg {}), edge {} use {} destonly {} prev {} prevopp {} fwdaccess {}",
            lo, edges.size(), out_tree.Path(a_turns[candidate.a].label).size(), edges[lo - 1].value,
            int(e1->use()), e1->destonly(), lo >= 2 ? edges[lo - 2].value : 0,
            e0 ? reader->GetOpposingEdgeId(edges[lo - 2], t).value == edges[lo - 1].value : false,
            e1->forwardaccess());
      }
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
