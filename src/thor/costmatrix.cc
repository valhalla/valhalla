#include <algorithm>
#include <cmath>
#include <vector>

#include "baldr/datetime.h"
#include "midgard/encoded.h"
#include "midgard/logging.h"
#include "sif/recost.h"
#include "thor/costmatrix.h"
#include "worker.h"

#include <robin_hood.h>

using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace {

constexpr uint32_t kMaxMatrixIterations = 2000000;
constexpr uint32_t kMaxThreshold = std::numeric_limits<int>::max();
constexpr uint32_t kMaxLocationReservation = 25; // the default config for max matrix locations

// Find a threshold to continue the search - should be based on
// the max edge cost in the adjacency set?
int GetThreshold(const travel_mode_t mode, const int n) {
  return (mode == travel_mode_t::kDrive) ? std::min(2800, std::max(100, n / 3)) : 500;
}

bool equals(const valhalla::LatLng& a, const valhalla::LatLng& b) {
  return a.has_lat_case() == b.has_lat_case() && a.has_lng_case() == b.has_lng_case() &&
         (!a.has_lat_case() || a.lat() == b.lat()) && (!a.has_lng_case() || a.lng() == b.lng());
}

inline const valhalla::PathEdge& find_correlated_edge(const valhalla::Location& location,
                                                      const GraphId& edge_id) {
  for (const auto& e : location.correlation().edges()) {
    if (e.graph_id() == edge_id)
      return e;
  }

  throw std::logic_error("Could not find candidate edge used for label");
}
} // namespace

namespace valhalla {
namespace thor {

class CostMatrix::ReachedMap : public robin_hood::unordered_map<uint64_t, std::vector<uint32_t>> {};

// Constructor with cost threshold.
CostMatrix::CostMatrix(const boost::property_tree::ptree& config)
    : MatrixAlgorithm(config),
      max_reserved_labels_count_(config.get<uint32_t>("max_reserved_labels_count_bidir_dijkstras",
                                                      kInitialEdgeLabelCountBidirDijkstra)),
      max_reserved_locations_count_(
          config.get<uint32_t>("max_reserved_locations_costmatrix", kMaxLocationReservation)),
      check_reverse_connections_(config.get<bool>("costmatrix_check_reverse_connection", false)),
      access_mode_(kAutoAccess),
      mode_(travel_mode_t::kDrive), locs_count_{0, 0}, locs_remaining_{0, 0},
      current_pathdist_threshold_(0), targets_{new ReachedMap}, sources_{new ReachedMap} {
}

CostMatrix::~CostMatrix() {
}

// Clear the temporary information generated during time + distance matrix
// construction.
void CostMatrix::Clear() {
  // Clear the target edge markings
  targets_->clear();
  if (check_reverse_connections_)
    sources_->clear();

  // Clear all adjacency lists, edge labels, and edge status
  // Resize and shrink_to_fit so all capacity is reduced.
  auto label_reservation = clear_reserved_memory_ ? 0 : max_reserved_labels_count_;
  auto locs_reservation = clear_reserved_memory_ ? 0 : max_reserved_locations_count_;
  for (const auto is_fwd : {MATRIX_FORW, MATRIX_REV}) {
    // resize all relevant structures down to configured amount of locations (25 default)
    if (locs_count_[is_fwd] > locs_reservation) {
      edgelabel_[is_fwd].resize(locs_reservation);
      edgelabel_[is_fwd].shrink_to_fit();
      adjacency_[is_fwd].resize(locs_reservation);
      adjacency_[is_fwd].shrink_to_fit();
      edgestatus_[is_fwd].resize(locs_reservation);
      edgestatus_[is_fwd].shrink_to_fit();
      astar_heuristics_[is_fwd].resize(locs_reservation);
      astar_heuristics_[is_fwd].shrink_to_fit();
    }
    for (auto& iter : edgelabel_[is_fwd]) {
      if (iter.size() > label_reservation) {
        iter.resize(label_reservation);
        iter.shrink_to_fit();
      }
      iter.clear();
    }
    for (auto& iter : edgestatus_[is_fwd]) {
      iter.clear();
    }
    for (auto& iter : adjacency_[is_fwd]) {
      iter.clear();
    }
    hierarchy_limits_[is_fwd].clear();
    locs_status_[is_fwd].clear();
    astar_heuristics_[is_fwd].clear();
  }
  best_connection_.clear();
  set_not_thru_pruning(true);
  ignore_hierarchy_limits_ = false;
}

// Form a time distance matrix from the set of source locations
// to the set of target locations.
bool CostMatrix::SourceToTarget(Api& request,
                                baldr::GraphReader& graphreader,
                                const sif::mode_costing_t& mode_costing,
                                const sif::travel_mode_t mode,
                                const float max_matrix_distance) {
  request.mutable_matrix()->set_algorithm(Matrix::CostMatrix);
  bool invariant = request.options().date_time_type() == Options::invariant;
  auto shape_format = request.options().shape_format();

  // Set the mode and costing
  mode_ = mode;
  costing_ = mode_costing[static_cast<uint32_t>(mode_)];
  access_mode_ = costing_->access_mode();

  auto& source_location_list = *request.mutable_options()->mutable_sources();
  auto& target_location_list = *request.mutable_options()->mutable_targets();

  current_pathdist_threshold_ = max_matrix_distance / 2;

  auto time_infos = SetOriginTimes(source_location_list, graphreader);

  // Initialize best connections and status. Any locations that are the
  // same get set to 0 time, distance and are not added to the remaining
  // location set.
  Initialize(source_location_list, target_location_list, request.matrix());

  // Set the source and target locations
  // TODO: for now we only allow depart_at/current date_time
  SetSources(graphreader, source_location_list, time_infos);
  SetTargets(graphreader, target_location_list);

  // Update hierarchy limits
  if (!ignore_hierarchy_limits_)
    ModifyHierarchyLimits();

  // Perform backward search from all target locations. Perform forward
  // search from all source locations. Connections between the 2 search
  // spaces is checked during the forward search.
  uint32_t n = 0;
  uint32_t interrupt_n = 0;
  while (true) {
    // First iterate over all targets, then over all sources: we only for sure
    // check the connection between both trees on the forward search, so reverse
    // has to come first
    for (uint32_t i = 0; i < locs_count_[MATRIX_REV]; i++) {
      if (locs_status_[MATRIX_REV][i].threshold > 0) {
        locs_status_[MATRIX_REV][i].threshold--;
        Expand<MatrixExpansionType::reverse>(i, n, graphreader, request.options());
        // if we exhausted this search
        if (locs_status_[MATRIX_REV][i].threshold == 0) {
          for (uint32_t source = 0; source < locs_count_[MATRIX_FORW]; source++) {
            // update the target for each source's remaining targets, if it still exists
            auto& targets = locs_status_[MATRIX_FORW][source].unfound_connections;
            auto it = targets.find(i);
            if (it != targets.end()) {
              // remove the target so we don't come here again
              targets.erase(it);
              // if there's no more target and the current source has not exhausted
              // we update the source's threshold so that it doesn't enter its outer "if" statement
              // anymore
              if (targets.empty() && locs_status_[MATRIX_FORW][source].threshold > 0) {
                // TODO(nils): shouldn't we extend the search here similar to bidir A*
                //   i.e. if pruning was disabled we extend the search in the other direction
                locs_status_[MATRIX_FORW][source].threshold = -1;
                if (locs_remaining_[MATRIX_FORW] > 0) {
                  locs_remaining_[MATRIX_FORW]--;
                }
              }
            }
          }
          // in any case make sure this was the last time we looked at this target
          locs_status_[MATRIX_REV][i].threshold = -1;
          if (locs_remaining_[MATRIX_REV] > 0) {
            locs_remaining_[MATRIX_REV]--;
          }
        }
      }
    }

    for (uint32_t i = 0; i < locs_count_[MATRIX_FORW]; i++) {
      if (locs_status_[MATRIX_FORW][i].threshold > 0) {
        locs_status_[MATRIX_FORW][i].threshold--;
        Expand<MatrixExpansionType::forward>(i, n, graphreader, request.options(), time_infos[i],
                                             invariant);
        // if we exhausted this search
        if (locs_status_[MATRIX_FORW][i].threshold == 0) {
          for (uint32_t target = 0; target < locs_count_[MATRIX_REV]; target++) {
            // if we still didn't find the connection between this pair
            auto& sources = locs_status_[MATRIX_REV][target].unfound_connections;
            auto it = sources.find(i);
            if (it != sources.end()) {
              // remove the source so we don't come here again
              sources.erase(it);
              // if there's no more sources and the current target has not exhausted
              // we update the target's threshold so that it doesn't enter this outer "if" statement
              // anymore
              if (sources.empty() && locs_status_[MATRIX_REV][target].threshold > 0) {
                // TODO(nils): shouldn't we extend the search here similar to bidir A*
                //   i.e. if pruning was disabled we extend the search in the other direction
                locs_status_[MATRIX_REV][target].threshold = -1;
                if (locs_remaining_[MATRIX_REV] > 0) {
                  locs_remaining_[MATRIX_REV]--;
                }
              }
            }
          }
          // in any case make sure this was the last time we looked at this source
          locs_status_[MATRIX_FORW][i].threshold = -1;
          if (locs_remaining_[MATRIX_FORW] > 0) {
            locs_remaining_[MATRIX_FORW]--;
          }
        }
      }
    }

    // Break out when remaining sources and targets to expand are both 0
    if (locs_remaining_[MATRIX_FORW] == 0 && locs_remaining_[MATRIX_REV] == 0) {
      LOG_DEBUG("SourceToTarget iterations: n = " + std::to_string(n));
      break;
    }

    // Protect against edge cases that may lead to never breaking out of
    // this loop. This should never occur but lets make sure.
    if (n >= kMaxMatrixIterations) {
      throw valhalla_exception_t{430};
    }
    // Allow this process to be aborted
    if (interrupt_ && (interrupt_n++ % kInterruptIterationsInterval) == 0) {
      (*interrupt_)();
    }
    n++;
  }

  // resize/reserve all properties of Matrix on first pass only
  valhalla::Matrix& matrix = *request.mutable_matrix();
  reserve_pbf_arrays(matrix, best_connection_.size(), costing_->pass());

  // Form the matrix PBF output
  graph_tile_ptr tile;
  bool connection_failed = false;
  for (uint32_t connection_idx = 0; connection_idx < best_connection_.size(); connection_idx++) {
    auto best_connection = best_connection_[connection_idx];
    // if this is the second pass we don't have to process previously found ones again
    if (costing_->pass() > 0 && !(matrix.second_pass(connection_idx))) {
      continue;
    }
    uint32_t target_idx = connection_idx % target_location_list.size();
    uint32_t source_idx = connection_idx / target_location_list.size();

    // first recost and form the path, if desired (either time and/or geometry requested)
    const auto shape = RecostFormPath(graphreader, best_connection, source_location_list[source_idx],
                                      target_location_list[target_idx], source_idx, target_idx,
                                      time_infos[source_idx], invariant, shape_format);

    float time = best_connection.cost.secs;
    if (time < kMaxCost) {
      auto dt_info =
          DateTime::offset_date(source_location_list[source_idx].date_time(),
                                time_infos[source_idx].timezone_index,
                                graphreader.GetTimezoneFromEdge(edgelabel_[MATRIX_REV][target_idx]
                                                                    .front()
                                                                    .edgeid(),
                                                                tile),
                                time);
      *matrix.mutable_date_times(connection_idx) = dt_info.date_time;
      *matrix.mutable_time_zone_offsets(connection_idx) = dt_info.time_zone_offset;
      *matrix.mutable_time_zone_names(connection_idx) = dt_info.time_zone_name;
    } else {
      // let's try a second pass for this connection
      matrix.mutable_second_pass()->Set(connection_idx, true);
      connection_failed = true;
    }
    matrix.mutable_from_indices()->Set(connection_idx, source_idx);
    matrix.mutable_to_indices()->Set(connection_idx, target_idx);
    matrix.mutable_distances()->Set(connection_idx, best_connection.distance);
    matrix.mutable_times()->Set(connection_idx, time);
    *matrix.mutable_shapes(connection_idx) = shape;
  }

  return !connection_failed;
}

// Initialize all time distance to "not found". Any locations that
// are the same get set to 0 time, distance and do not add to the
// remaining locations set.
void CostMatrix::Initialize(
    const google::protobuf::RepeatedPtrField<valhalla::Location>& source_locations,
    const google::protobuf::RepeatedPtrField<valhalla::Location>& target_locations,
    const valhalla::Matrix& matrix) {

  locs_count_[MATRIX_FORW] = source_locations.size();
  locs_count_[MATRIX_REV] = target_locations.size();
  astar_heuristics_[MATRIX_FORW].resize(target_locations.size());
  astar_heuristics_[MATRIX_REV].resize(source_locations.size());

  const auto& hlimits = costing_->GetHierarchyLimits();
  ignore_hierarchy_limits_ =
      std::all_of(hlimits.begin() + 1, hlimits.begin() + TileHierarchy::levels().size(),
                  [](const HierarchyLimits& limits) {
                    return limits.max_up_transitions == kUnlimitedTransitions;
                  });

  const uint32_t bucketsize = costing_->UnitSize();
  const float range = kBucketCount * bucketsize;

  // Add initial sources & targets properties
  for (const auto is_fwd : {MATRIX_FORW, MATRIX_REV}) {
    const auto count = locs_count_[is_fwd];
    const auto other_count = locs_count_[!is_fwd];

    const auto& locations = is_fwd ? source_locations : target_locations;
    const auto& other_locations = is_fwd ? target_locations : source_locations;

    locs_status_[is_fwd].reserve(count);
    hierarchy_limits_[is_fwd].resize(count);
    adjacency_[is_fwd].resize(count);
    edgestatus_[is_fwd].resize(count);
    edgelabel_[is_fwd].resize(count);
    for (uint32_t i = 0; i < count; i++) {
      // Allocate the adjacency list and hierarchy limits for this source.
      // Use the cost threshold to size the adjacency list.
      edgelabel_[is_fwd][i].reserve(max_reserved_labels_count_);
      locs_status_[is_fwd].emplace_back(kMaxThreshold);
      hierarchy_limits_[is_fwd][i] = hlimits;
      // for each source/target init the other direction's astar heuristic
      auto& ll = locations[i].ll();
      astar_heuristics_[!is_fwd][i].Init({ll.lng(), ll.lat()}, costing_->AStarCostFactor());

      // get the min heuristic to all targets/sources for this source's/target's adjacency list
      float min_heuristic = std::numeric_limits<float>::max();
      for (uint32_t j = 0; j < other_count; j++) {
        auto& other_ll = other_locations[j].ll();
        auto heuristic = astar_heuristics_[!is_fwd][i].Get({other_ll.lng(), other_ll.lat()});
        min_heuristic = std::min(min_heuristic, heuristic);
      }
      // TODO(nils): previously we'd estimate the bucket range by the max matrix distance,
      // which would lead to tons of RAM if a high value was chosen in the config; ideally
      // this would be chosen based on the request (e.g. some factor to the A* distance)
      adjacency_[is_fwd][i].reuse(min_heuristic, range, bucketsize, &edgelabel_[is_fwd][i]);
    }
  }

  // Initialize best connection
  GraphId empty;
  Cost trivial_cost(0.0f, 0.0f);
  Cost max_cost(kMaxCost, kMaxCost);
  best_connection_.reserve(locs_count_[MATRIX_FORW] * locs_count_[MATRIX_REV]);
  for (uint32_t i = 0; i < locs_count_[MATRIX_FORW]; i++) {
    for (uint32_t j = 0; j < locs_count_[MATRIX_REV]; j++) {
      const auto connection_idx = i * static_cast<uint32_t>(target_locations.size()) + j;
      if (equals(source_locations.Get(i).ll(), target_locations.Get(j).ll())) {
        best_connection_.emplace_back(empty, empty, trivial_cost, 0.0f);
        best_connection_.back().found = true;
      } else if (costing_->pass() > 0 && !matrix.second_pass(connection_idx)) {
        // we've found this connection in a previous pass, we only need the time & distance
        best_connection_.emplace_back(empty, empty, Cost{0.0f, matrix.times(connection_idx)},
                                      matrix.distances(connection_idx));
        best_connection_.back().found = true;
      } else {
        // in a second pass this block makes sure that if e.g. A -> B is found, but B -> A isn't,
        // we still expand both A & B to get the bidirectional benefit
        best_connection_.emplace_back(empty, empty, max_cost, static_cast<uint32_t>(kMaxCost));
        locs_status_[MATRIX_FORW][i].unfound_connections.insert(j);
        locs_status_[MATRIX_REV][j].unfound_connections.insert(i);
      }
    }
  }

  // Set the remaining number of sources and targets
  locs_remaining_[MATRIX_FORW] = 0;
  for (auto& s : locs_status_[MATRIX_FORW]) {
    if (!s.unfound_connections.empty()) {
      locs_remaining_[MATRIX_FORW]++;
    } else {
      // don't look at sources which don't have unfound connections, important for second pass
      s.threshold = 0;
    }
  }
  locs_remaining_[MATRIX_REV] = 0;
  for (auto& t : locs_status_[MATRIX_REV]) {
    if (!t.unfound_connections.empty()) {
      locs_remaining_[MATRIX_REV]++;
    } else {
      // don't look at targets which don't have unfound connections, important for second pass
      t.threshold = 0;
    }
  }
}

template <const MatrixExpansionType expansion_direction, const bool FORWARD>
bool CostMatrix::ExpandInner(baldr::GraphReader& graphreader,
                             const uint32_t index,
                             const sif::BDEdgeLabel& pred,
                             const baldr::DirectedEdge* opp_pred_edge,
                             const baldr::NodeInfo* nodeinfo,
                             const uint32_t pred_idx,
                             const EdgeMetadata& meta,
                             uint32_t& shortcuts,
                             const graph_tile_ptr& tile,
                             const baldr::TimeInfo& time_info) {
  // Skip if this is a regular edge superseded by a shortcut.
  if (shortcuts & meta.edge->superseded()) {
    return false;
  }

  graph_tile_ptr t2 = nullptr;
  baldr::GraphId opp_edge_id;
  const auto get_opp_edge_data = [&t2, &opp_edge_id, &graphreader, &meta, &tile]() {
    t2 = meta.edge->leaves_tile() ? graphreader.GetGraphTile(meta.edge->endnode()) : tile;
    if (t2 == nullptr) {
      return false;
    }

    opp_edge_id = t2->GetOpposingEdgeId(meta.edge);
    return true;
  };

  if (meta.edge->is_shortcut()) {
    // Skip shortcuts if hierarchy limits are disabled or the opposing tile doesn't exist
    if (ignore_hierarchy_limits_ || !get_opp_edge_data())
      return false;

    // Skip shortcut edges until we have stopped expanding on the next level. Use regular
    // edges while still expanding on the next level since we can still transition down to
    // that level. If using a shortcut, set the shortcuts mask. Skip if this is a regular
    // edge superseded by a shortcut.
    if (hierarchy_limits_[FORWARD][index][meta.edge_id.level() + 1].StopExpanding()) {
      shortcuts |= meta.edge->shortcut();
    } else {
      return false;
    }
  }

  // Skip this edge if permanently labeled (best path already found to this
  // directed edge) or if no access for this mode.
  if (meta.edge_status->set() == EdgeSet::kPermanent) {
    return true;
  }

  // TODO(nils): refactor this whole thing to only have a single if (FORWARD) {}

  const baldr::DirectedEdge* opp_edge = nullptr;
  if (!FORWARD) {
    // Check the access mode and skip this edge if access is not allowed in the reverse
    // direction. This avoids the (somewhat expensive) retrieval of the opposing directed
    // edge when no access is allowed in the reverse direction.
    if (!(meta.edge->reverseaccess() & access_mode_)) {
      return false;
    }

    if (t2 == nullptr && !get_opp_edge_data()) {
      return false;
    }

    opp_edge = t2->directededge(opp_edge_id);
  }

  auto& edgelabels = edgelabel_[FORWARD][index];
  // Skip this edge if no access is allowed (based on costing method)
  // or if a complex restriction prevents transition onto this edge.
  uint8_t restriction_idx = kInvalidRestriction;
  if (FORWARD) {
    if (!costing_->Allowed(meta.edge, false, pred, tile, meta.edge_id, time_info.local_time,
                           time_info.timezone_index, restriction_idx) ||
        costing_->Restricted(meta.edge, pred, edgelabels, tile, meta.edge_id, true,
                             &edgestatus_[FORWARD][index], time_info.local_time,
                             time_info.timezone_index)) {
      return false;
    }
  } else {
    if (!costing_->AllowedReverse(meta.edge, pred, opp_edge, t2, opp_edge_id, time_info.local_time,
                                  time_info.timezone_index, restriction_idx) ||
        costing_->Restricted(meta.edge, pred, edgelabels, tile, meta.edge_id, false,
                             &edgestatus_[FORWARD][index], time_info.local_time,
                             time_info.timezone_index)) {
      return false;
    }
  }

  // Get cost. Separate out transition cost.
  uint8_t flow_sources;
  Cost newcost = pred.cost() + (FORWARD ? costing_->EdgeCost(meta.edge, tile, time_info, flow_sources)
                                        : costing_->EdgeCost(opp_edge, t2, time_info, flow_sources));
  sif::Cost tc =
      FORWARD ? costing_->TransitionCost(meta.edge, nodeinfo, pred)
              : costing_->TransitionCostReverse(meta.edge->localedgeidx(), nodeinfo, opp_edge,
                                                opp_pred_edge,
                                                static_cast<bool>(flow_sources & kDefaultFlowMask),
                                                pred.internal_turn());
  newcost += tc;

  const auto pred_dist = pred.path_distance() + meta.edge->length();
  auto& adj = adjacency_[FORWARD][index];
  // Check if edge is temporarily labeled and this path has less cost. If
  // less cost the predecessor is updated and the sort cost is decremented
  // by the difference in real cost (A* heuristic doesn't change)
  if (meta.edge_status->set() == EdgeSet::kTemporary) {
    BDEdgeLabel& lab = edgelabel_[FORWARD][index][meta.edge_status->index()];
    if (newcost.cost < lab.cost().cost) {
      float newsortcost = lab.sortcost() - (lab.cost().cost - newcost.cost);
      adj.decrease(meta.edge_status->index(), newsortcost);
      lab.Update(pred_idx, newcost, newsortcost, tc, pred_dist, restriction_idx);
    }
    // Returning true since this means we approved the edge
    return true;
  }

  // Get end node tile (skip if tile is not found) and opposing edge Id
  if (t2 == nullptr && !get_opp_edge_data()) {
    return false;
  }

  // not_thru_pruning_ is only set to false on the 2nd pass in matrix_action.
  // We allow settling not_thru edges so we can connect both trees on them.
  bool not_thru_pruning =
      not_thru_pruning_ ? (pred.not_thru_pruning() || !meta.edge->not_thru()) : false;

  // TODO(nils): we could use the distance to the source/target to disable hierarchy limits
  // , just as bidir a* /route does; that would make for more optimal paths in some edge cases but
  // we'd pay a severe performance penalty, e.g. a request with distance checks (i.e. more expansion
  // on lower levels) takes 100 secs, while without it takes 60 secs.

  // Add edge label, add to the adjacency list and set edge status
  uint32_t idx = edgelabels.size();
  *meta.edge_status = {EdgeSet::kTemporary, idx};
  if (FORWARD) {
    edgelabels.emplace_back(pred_idx, meta.edge_id, opp_edge_id, meta.edge, newcost, mode_, tc,
                            pred_dist, not_thru_pruning,
                            (pred.closure_pruning() || !costing_->IsClosed(meta.edge, tile)),
                            static_cast<bool>(flow_sources & kDefaultFlowMask),
                            costing_->TurnType(pred.opp_local_idx(), nodeinfo, meta.edge),
                            restriction_idx, 0,
                            meta.edge->destonly() ||
                                (costing_->is_hgv() && meta.edge->destonly_hgv()),
                            meta.edge->forwardaccess() & kTruckAccess);
  } else {
    edgelabels.emplace_back(pred_idx, meta.edge_id, opp_edge_id, meta.edge, newcost, mode_, tc,
                            pred_dist, not_thru_pruning,
                            (pred.closure_pruning() || !costing_->IsClosed(opp_edge, t2)),
                            static_cast<bool>(flow_sources & kDefaultFlowMask),
                            costing_->TurnType(meta.edge->localedgeidx(), nodeinfo, opp_edge,
                                               opp_pred_edge),
                            restriction_idx, 0,
                            opp_edge->destonly() || (costing_->is_hgv() && opp_edge->destonly_hgv()),
                            opp_edge->forwardaccess() & kTruckAccess);
  }
  auto newsortcost =
      GetAstarHeuristic<expansion_direction>(index, t2->get_node_ll(meta.edge->endnode()));
  edgelabels.back().SetSortCost(newcost.cost + newsortcost);
  adj.add(idx);

  // mark the edge as settled for the connection check
  if (!FORWARD) {
    (*targets_)[meta.edge_id].push_back(index);
  } else if (check_reverse_connections_) {
    (*sources_)[meta.edge_id].push_back(index);
  }

  // setting this edge as reached
  if (expansion_callback_) {
    expansion_callback_(graphreader, meta.edge_id, pred.edgeid(), "costmatrix",
                        Expansion_EdgeStatus_reached, newcost.secs, pred_dist, newcost.cost,
                        static_cast<Expansion_ExpansionType>(expansion_direction));
  }

  return !(pred.not_thru_pruning() && meta.edge->not_thru());
}

template <const MatrixExpansionType expansion_direction, const bool FORWARD>
bool CostMatrix::Expand(const uint32_t index,
                        const uint32_t n,
                        baldr::GraphReader& graphreader,
                        const valhalla::Options& options,
                        const baldr::TimeInfo& time_info,
                        const bool invariant) {

  auto& adj = adjacency_[FORWARD][index];
  auto& edgelabels = edgelabel_[FORWARD][index];
  uint32_t pred_idx = adj.pop();
  if (pred_idx == kInvalidLabel) {
    // search is exhausted - mark this and update so we don't
    // extend searches more than we need to
    for (uint32_t st = 0; st < locs_count_[!FORWARD]; st++) {
      if (FORWARD) {
        UpdateStatus(index, st);
      } else {
        UpdateStatus(st, index);
      }
    }
    locs_status_[FORWARD][index].threshold = 0;
    return false;
  }

  // Get edge label and check cost threshold
  auto pred = edgelabels[pred_idx];
  if (pred.path_distance() > current_pathdist_threshold_) {
    locs_status_[FORWARD][index].threshold = 0;
    return false;
  }

  // Settle this edge and log it if requested
  auto& edgestatus = edgestatus_[FORWARD][index];
  edgestatus.Update(pred.edgeid(), EdgeSet::kPermanent);
  if (expansion_callback_) {
    auto prev_pred =
        pred.predecessor() == kInvalidLabel ? GraphId{} : edgelabels[pred.predecessor()].edgeid();
    expansion_callback_(graphreader, pred.edgeid(), prev_pred, "costmatrix",
                        Expansion_EdgeStatus_settled, pred.cost().secs, pred.path_distance(),
                        pred.cost().cost, static_cast<Expansion_ExpansionType>(expansion_direction));
  }

  if (FORWARD) {
    CheckForwardConnections(index, pred, n, graphreader, options);
  } else if (check_reverse_connections_) {
    CheckReverseConnections(index, pred, n, graphreader, options);
  }

  GraphId node = pred.endnode();
  // Prune path if predecessor is not a through edge or if the maximum
  // number of upward transitions has been exceeded on this hierarchy level.
  if ((pred.not_thru() && pred.not_thru_pruning()) ||
      (!ignore_hierarchy_limits_ &&
       hierarchy_limits_[FORWARD][index][node.level()].StopExpanding())) {
    return false;
  }

  // Get the tile and the node info. Skip if tile is null (can happen
  // with regional data sets) or if no access at the node.
  graph_tile_ptr tile = graphreader.GetGraphTile(node);
  if (tile == nullptr) {
    return false;
  }
  const NodeInfo* nodeinfo = tile->node(node);

  // set the time info
  auto seconds_offset = invariant ? 0.f : pred.cost().secs;
  auto offset_time = FORWARD
                         ? time_info.forward(seconds_offset, static_cast<int>(nodeinfo->timezone()))
                         : time_info.reverse(seconds_offset, static_cast<int>(nodeinfo->timezone()));

  // Get the opposing predecessor directed edge if this is reverse.
  const DirectedEdge* opp_pred_edge = nullptr;
  if (!FORWARD) {
    const auto rev_pred_tile = graphreader.GetGraphTile(pred.opp_edgeid(), tile);
    if (rev_pred_tile == nullptr) {
      return false;
    }
    opp_pred_edge = rev_pred_tile->directededge(pred.opp_edgeid());
  }

  // keep track of shortcuts
  uint32_t shortcuts = 0;
  // If we encounter a node with an access restriction like a barrier we allow a uturn
  if (!costing_->Allowed(nodeinfo)) {
    const DirectedEdge* opp_edge = nullptr;
    const GraphId opp_edge_id = graphreader.GetOpposingEdgeId(pred.edgeid(), opp_edge, tile);
    // Mark the predecessor as a deadend to be consistent with how the
    // edgelabels are set when an *actual* deadend (i.e. some dangling OSM geometry)
    // is labelled
    pred.set_deadend(true);
    // Check if edge is null before using it (can happen with regional data sets)
    return opp_edge && ExpandInner<expansion_direction>(graphreader, index, pred, opp_pred_edge,
                                                        nodeinfo, pred_idx,
                                                        {opp_edge, opp_edge_id,
                                                         edgestatus.GetPtr(opp_edge_id, tile)},
                                                        shortcuts, tile, offset_time);
  }

  // catch u-turn attempts
  bool disable_uturn = false;
  EdgeMetadata meta = EdgeMetadata::make(node, nodeinfo, tile, edgestatus);
  EdgeMetadata uturn_meta{};

  // Expand from end node in <expansion_direction> direction.
  for (uint32_t i = 0; i < nodeinfo->edge_count(); ++i, ++meta) {

    // Begin by checking if this is the opposing edge to pred.
    // If so, it means we are attempting a u-turn. In that case, lets wait with evaluating
    // this edge until last. If any other edges were emplaced, it means we should not
    // even try to evaluate a u-turn since u-turns should only happen for deadends
    const bool is_uturn = pred.opp_local_idx() == meta.edge->localedgeidx();
    uturn_meta = is_uturn ? meta : uturn_meta;

    // Expand but only if this isnt the uturn, we'll try that later if nothing else works out
    disable_uturn =
        (!is_uturn &&
         ExpandInner<expansion_direction>(graphreader, index, pred, opp_pred_edge, nodeinfo, pred_idx,
                                          meta, shortcuts, tile, offset_time)) ||
        disable_uturn;
  }

  // Handle transitions - expand from the end node of each transition
  if (nodeinfo->transition_count() > 0) {
    const NodeTransition* trans = tile->transition(nodeinfo->transition_index());
    auto& hierarchy_limits = hierarchy_limits_[FORWARD][index];
    for (uint32_t i = 0; i < nodeinfo->transition_count(); ++i, ++trans) {
      // if this is a downward transition (ups are always allowed) AND we are no longer allowed OR
      // we cant get the tile at that level (local extracts could have this problem) THEN bail
      graph_tile_ptr trans_tile = nullptr;
      if ((!trans->up() && !ignore_hierarchy_limits_ &&
           hierarchy_limits[trans->endnode().level()].StopExpanding()) ||
          !(trans_tile = graphreader.GetGraphTile(trans->endnode()))) {
        continue;
      }

      // setup for expansion at this level
      hierarchy_limits[node.level()].up_transition_count += trans->up();
      const auto* trans_node = trans_tile->node(trans->endnode());
      EdgeMetadata trans_meta =
          EdgeMetadata::make(trans->endnode(), trans_node, trans_tile, edgestatus);
      uint32_t trans_shortcuts = 0;
      // expand the edges from this node at this level
      for (uint32_t i = 0; i < trans_node->edge_count(); ++i, ++trans_meta) {
        disable_uturn = ExpandInner<expansion_direction>(graphreader, index, pred, opp_pred_edge,
                                                         trans_node, pred_idx, trans_meta,
                                                         trans_shortcuts, trans_tile, offset_time) ||
                        disable_uturn;
      }
    }
  }

  // Now, after having looked at all the edges, including edges on other levels,
  // we can say if this is a deadend or not, and if so, evaluate the uturn-edge (if it exists)
  if (!disable_uturn && uturn_meta) {
    // If we found no suitable edge to add, it means we're at a deadend
    // so lets go back and re-evaluate a potential u-turn
    pred.set_deadend(true);

    // TODO(nils): what if there is a shortcut that supersedes our u-turn? can that even be?
    // We then need to decide if we should expand the shortcut or the non-shortcut edge...

    // Expand the uturn possibility
    disable_uturn =
        ExpandInner<expansion_direction>(graphreader, index, pred, opp_pred_edge, nodeinfo, pred_idx,
                                         uturn_meta, shortcuts, tile, offset_time);
  }

  return disable_uturn;
}

// Check if the edge on the forward search connects to a reached edge
// on the reverse search trees.
void CostMatrix::CheckForwardConnections(const uint32_t source,
                                         const BDEdgeLabel& fwd_pred,
                                         const uint32_t n,
                                         GraphReader& graphreader,
                                         const valhalla::Options& options) {

  // Disallow connections that are part of an uturn on an internal edge
  if (fwd_pred.internal_turn() != InternalTurn::kNoTurn) {
    return;
  }
  // Disallow connections that are part of a complex restriction.
  // TODO - validate that we do not need to "walk" the paths forward
  // and backward to see if they match a restriction.
  if (fwd_pred.on_complex_rest()) {
    // TODO(nils): bidir a* is digging deeper
    return;
  }

  // Get the opposing edge. Get a list of target locations whose reverse
  // search has reached this edge.
  GraphId rev_edgeid = fwd_pred.opp_edgeid();
  auto targets = targets_->find(rev_edgeid);
  if (targets == targets_->end()) {
    return;
  }

  // Iterate through the targets
  for (auto target : targets->second) {
    uint32_t idx = source * locs_count_[MATRIX_REV] + target;
    if (best_connection_[idx].found) {
      continue;
    }

    // Update any targets whose threshold has been reached
    if (best_connection_[idx].max_iterations > 0 && n > best_connection_[idx].max_iterations) {
      best_connection_[idx].found = true;
      continue;
    }

    // If we came down here, we know this opposing edge is either settled, or it's a
    // target correlated edge which hasn't been pulled out of the queue yet, so a path
    // has been found to the end node of this directed edge
    const auto& rev_edgestate = edgestatus_[MATRIX_REV][target];
    EdgeStatusInfo rev_edgestatus = rev_edgestate.Get(rev_edgeid);
    const auto& rev_edgelabels = edgelabel_[MATRIX_REV][target];
    uint32_t rev_predidx = rev_edgelabels[rev_edgestatus.index()].predecessor();
    const BDEdgeLabel& rev_label = rev_edgelabels[rev_edgestatus.index()];

    // Special case - common edge for source and target are both initial edges
    if (fwd_pred.predecessor() == kInvalidLabel && rev_predidx == kInvalidLabel) {
      // bail if forward edge wasn't allowed (see notes in SetSources/Targets)
      if (!fwd_pred.path_id()) {
        return;
      }

      // if source percent along edge is larger than target percent along,
      // can't connect on this edge
      if (find_correlated_edge(options.sources(source), fwd_pred.edgeid()).percent_along() >
          find_correlated_edge(options.targets(target), fwd_pred.edgeid()).percent_along()) {
        return;
      }

      // remember: transition_cost is abused in SetSources/Targets: cost is secs, secs is length
      float s =
          std::abs(fwd_pred.cost().secs + rev_label.cost().secs - rev_label.transition_cost().cost);

      // Update best connection and set found = true.
      // distance computation only works with the casts.
      uint32_t d = std::abs(static_cast<int>(fwd_pred.path_distance()) +
                            static_cast<int>(rev_label.path_distance()) -
                            static_cast<int>(rev_label.transition_cost().secs));
      best_connection_[idx].Update(fwd_pred.edgeid(), rev_edgeid, Cost(s, s), d);
      best_connection_[idx].found = true;

      // Update status and update threshold if this is the last location
      // to find for this source or target
      UpdateStatus(source, target);
    } else {
      float oppcost = (rev_predidx == kInvalidLabel) ? 0.f : rev_edgelabels[rev_predidx].cost().cost;
      float c = fwd_pred.cost().cost + oppcost + rev_label.transition_cost().cost;

      // Check if best connection
      if (c < best_connection_[idx].cost.cost) {
        float oppsec = (rev_predidx == kInvalidLabel) ? 0.f : rev_edgelabels[rev_predidx].cost().secs;
        uint32_t oppdist =
            (rev_predidx == kInvalidLabel) ? 0U : rev_edgelabels[rev_predidx].path_distance();
        float s = fwd_pred.cost().secs + oppsec + rev_label.transition_cost().secs;
        uint32_t d = fwd_pred.path_distance() + oppdist;

        // Update best connection and set a threshold
        best_connection_[idx].Update(fwd_pred.edgeid(), rev_edgeid, Cost(c, s), d);
        if (best_connection_[idx].max_iterations == 0) {
          best_connection_[idx].max_iterations =
              n + GetThreshold(mode_, edgelabel_[MATRIX_FORW][source].size() +
                                          edgelabel_[MATRIX_REV][target].size());
        }

        // Update status and update threshold if this is the last location
        // to find for this source or target
        UpdateStatus(source, target);
      }
    }
    // setting this edge as connected
    if (expansion_callback_) {
      auto prev_pred = fwd_pred.predecessor() == kInvalidLabel
                           ? GraphId{}
                           : edgelabel_[MATRIX_FORW][source][fwd_pred.predecessor()].edgeid();
      expansion_callback_(graphreader, fwd_pred.edgeid(), prev_pred, "costmatrix",
                          Expansion_EdgeStatus_connected, fwd_pred.cost().secs,
                          fwd_pred.path_distance(), fwd_pred.cost().cost,
                          Expansion_ExpansionType_forward);
    }
  }

  return;
}

void CostMatrix::CheckReverseConnections(const uint32_t target,
                                         const BDEdgeLabel& rev_pred,
                                         const uint32_t n,
                                         GraphReader& graphreader,
                                         const valhalla::Options& options) {

  // Disallow connections that are part of an uturn on an internal edge
  if (rev_pred.internal_turn() != InternalTurn::kNoTurn) {
    return;
  }
  // Disallow connections that are part of a complex restriction.
  // TODO - validate that we do not need to "walk" the paths forward
  // and backward to see if they match a restriction.
  if (rev_pred.on_complex_rest()) {
    return;
  }

  // Get the opposing edge. Get a list of source locations whose forward
  // search has reached this edge.
  GraphId fwd_edgeid = rev_pred.opp_edgeid();
  auto sources = sources_->find(fwd_edgeid);
  if (sources == sources_->end()) {
    return;
  }

  // Iterate through the sources
  for (auto source : sources->second) {
    uint32_t source_idx = source * locs_count_[MATRIX_REV] + target;
    if (best_connection_[source_idx].found) {
      continue;
    }

    // Update any targets whose threshold has been reached
    if (best_connection_[source_idx].max_iterations > 0 &&
        n > best_connection_[source_idx].max_iterations) {
      best_connection_[source_idx].found = true;
      continue;
    }

    // If this edge has been reached then a shortest path has been found
    // to the end node of this directed edge.
    EdgeStatusInfo fwd_edgestatus = edgestatus_[MATRIX_FORW][source].Get(fwd_edgeid);
    if (fwd_edgestatus.set() != EdgeSet::kUnreachedOrReset) {
      const auto& fwd_edgelabels = edgelabel_[MATRIX_FORW][source];
      uint32_t fwd_predidx = fwd_edgelabels[fwd_edgestatus.index()].predecessor();
      const BDEdgeLabel& fwd_label = fwd_edgelabels[fwd_edgestatus.index()];

      // Special case - common edge for source and target are both initial edges
      if (rev_pred.predecessor() == kInvalidLabel && fwd_predidx == kInvalidLabel) {
        // bail if the edge wasn't allowed
        if (!rev_pred.path_id()) {
          return;
        }

        if (find_correlated_edge(options.sources(source), fwd_label.edgeid()).percent_along() >
            find_correlated_edge(options.targets(target), fwd_label.edgeid()).percent_along()) {
          return;
        }

        // remember: transition_cost is abused in SetSources/Targets: cost is secs, secs is length
        float s =
            std::abs(rev_pred.cost().secs + fwd_label.cost().secs - fwd_label.transition_cost().cost);

        // Update best connection and set found = true.
        // distance computation only works with the casts.
        uint32_t d = std::abs(static_cast<int>(rev_pred.path_distance()) +
                              static_cast<int>(fwd_label.path_distance()) -
                              static_cast<int>(fwd_label.transition_cost().secs));
        best_connection_[source_idx].Update(fwd_edgeid, rev_pred.edgeid(), Cost(s, s), d);
        best_connection_[source_idx].found = true;

        // Update status and update threshold if this is the last location
        // to find for this source or target
        UpdateStatus(source, target);
      } else {
        float oppcost = (fwd_predidx == kInvalidLabel) ? 0 : fwd_edgelabels[fwd_predidx].cost().cost;
        float c = rev_pred.cost().cost + oppcost + fwd_label.transition_cost().cost;

        // Check if best connection
        if (c < best_connection_[source_idx].cost.cost) {
          float fwd_sec =
              (fwd_predidx == kInvalidLabel) ? 0 : fwd_edgelabels[fwd_predidx].cost().secs;
          uint32_t fwd_dist =
              (fwd_predidx == kInvalidLabel) ? 0 : fwd_edgelabels[fwd_predidx].path_distance();
          float s = rev_pred.cost().secs + fwd_sec + fwd_label.transition_cost().secs;
          uint32_t d = rev_pred.path_distance() + fwd_dist;

          // Update best connection and set a threshold
          best_connection_[source_idx].Update(fwd_edgeid, rev_pred.edgeid(), Cost(c, s), d);
          if (best_connection_[source_idx].max_iterations == 0) {
            best_connection_[source_idx].max_iterations =
                n + GetThreshold(mode_, edgelabel_[MATRIX_FORW][source].size() +
                                            edgelabel_[MATRIX_REV][target].size());
          }

          // Update status and update threshold if this is the last location
          // to find for this source or target
          UpdateStatus(source, target);
        }
      }
      // setting this edge as connected
      if (expansion_callback_) {
        auto prev_pred = rev_pred.predecessor() == kInvalidLabel
                             ? GraphId{}
                             : edgelabel_[MATRIX_REV][target][rev_pred.predecessor()].edgeid();
        expansion_callback_(graphreader, rev_pred.edgeid(), prev_pred, "costmatrix",
                            Expansion_EdgeStatus_connected, rev_pred.cost().secs,
                            rev_pred.path_distance(), rev_pred.cost().cost,
                            Expansion_ExpansionType_reverse);
      }
    }
  }

  return;
}

// Update status when a connection is found.
void CostMatrix::UpdateStatus(const uint32_t source, const uint32_t target) {
  // Remove the target from the source status
  auto& s = locs_status_[MATRIX_FORW][source].unfound_connections;
  auto it = s.find(target);
  if (it != s.end()) {
    s.erase(it);
    if (s.empty() && locs_status_[MATRIX_FORW][source].threshold > 0) {
      // At least 1 connection has been found to each target for this source.
      // Set a threshold to continue search for a limited number of times.
      locs_status_[MATRIX_FORW][source].threshold =
          GetThreshold(mode_, edgelabel_[MATRIX_FORW][source].size() +
                                  edgelabel_[MATRIX_REV][target].size());
    }
  }

  // Remove the source from the target status
  auto& t = locs_status_[MATRIX_REV][target].unfound_connections;
  it = t.find(source);
  if (it != t.end()) {
    t.erase(it);
    if (t.empty() && locs_status_[MATRIX_REV][target].threshold > 0) {
      // At least 1 connection has been found to each source for this target.
      // Set a threshold to continue search for a limited number of times.
      locs_status_[MATRIX_REV][target].threshold =
          GetThreshold(mode_, edgelabel_[MATRIX_FORW][source].size() +
                                  edgelabel_[MATRIX_REV][target].size());
    }
  }
}

// Sets the source/origin locations. Search expands forward from these
// locations.
void CostMatrix::SetSources(GraphReader& graphreader,
                            const google::protobuf::RepeatedPtrField<valhalla::Location>& sources,
                            const std::vector<baldr::TimeInfo>& time_infos) {
  // Go through each source location
  uint32_t index = 0;
  Cost empty_cost;
  for (const auto& origin : sources) {
    // Only skip inbound edges if we have other options
    bool has_other_edges = false;
    std::for_each(origin.correlation().edges().begin(), origin.correlation().edges().end(),
                  [&has_other_edges](const valhalla::PathEdge& e) {
                    has_other_edges = has_other_edges || !e.end_node();
                  });

    // Iterate through edges and add to adjacency list
    for (const auto& edge : origin.correlation().edges()) {
      // If origin is at a node - skip any inbound edge (dist = 1)
      if (has_other_edges && edge.end_node()) {
        continue;
      }

      // Disallow any user avoid edges if the avoid location is ahead of the origin along the edge
      GraphId edgeid(edge.graph_id());
      if (costing_->AvoidAsOriginEdge(edgeid, edge.percent_along())) {
        continue;
      }

      // Get the directed edge and the opposing edge Id
      graph_tile_ptr tile = graphreader.GetGraphTile(edgeid);
      graph_tile_ptr opp_tile = tile;
      const DirectedEdge* directededge = tile->directededge(edgeid);
      GraphId oppedgeid = graphreader.GetOpposingEdgeId(edgeid, opp_tile);

      // Get cost. Get distance along the remainder of this edge.
      uint8_t flow_sources;
      Cost edgecost = costing_->EdgeCost(directededge, tile, time_infos[index], flow_sources);
      Cost cost = edgecost * (1.0f - edge.percent_along());
      uint32_t d = std::round(directededge->length() * (1.0f - edge.percent_along()));

      // We need to penalize this location based on its score (distance in meters from input)
      // We assume the slowest speed you could travel to cover that distance to start/end the route
      // TODO: assumes 1m/s which is a maximum penalty this could vary per costing model
      cost.cost += edge.distance();

      // 2 adjustments related only to properly handle trivial routes:
      //   - "transition_cost" is used to store the traversed secs & length
      //   - "path_id" is used to store whether the edge is even allowed (e.g. no oneway)
      Cost ec(std::round(edgecost.secs), static_cast<uint32_t>(directededge->length()));
      BDEdgeLabel edge_label(kInvalidLabel, edgeid, oppedgeid, directededge, cost, mode_, ec, d,
                             !directededge->not_thru(), !(costing_->IsClosed(directededge, tile)),
                             static_cast<bool>(flow_sources & kDefaultFlowMask),
                             InternalTurn::kNoTurn, kInvalidRestriction,
                             static_cast<uint8_t>(costing_->Allowed(directededge, tile)),
                             directededge->destonly() ||
                                 (costing_->is_hgv() && directededge->destonly_hgv()),
                             directededge->forwardaccess() & kTruckAccess);
      auto newsortcost =
          GetAstarHeuristic<MatrixExpansionType::forward>(index, opp_tile->get_node_ll(
                                                                     directededge->endnode()));
      edge_label.SetSortCost(cost.cost + newsortcost);

      // Set the initial not_thru flag to false. There is an issue with not_thru
      // flags on small loops. Set this to false here to override this for now.
      edge_label.set_not_thru(false);

      // Add EdgeLabel to the adjacency list (but do not set its status).
      // Set the predecessor edge index to invalid to indicate the origin
      // of the path.
      uint32_t idx = edgelabel_[MATRIX_FORW][index].size();
      edgelabel_[MATRIX_FORW][index].push_back(std::move(edge_label));
      adjacency_[MATRIX_FORW][index].add(idx);
      edgestatus_[MATRIX_FORW][index].Set(edgeid, EdgeSet::kTemporary, idx, tile);
      if (check_reverse_connections_)
        (*sources_)[edgeid].push_back(index);
    }
    index++;
  }
}

// Set the target/destination locations. Search expands backwards from
// these locations.
void CostMatrix::SetTargets(baldr::GraphReader& graphreader,
                            const google::protobuf::RepeatedPtrField<valhalla::Location>& targets) {
  // Go through each target location
  uint32_t index = 0;
  Cost empty_cost;
  for (const auto& dest : targets) {
    // Only skip outbound edges if we have other options
    bool has_other_edges = false;
    std::for_each(dest.correlation().edges().begin(), dest.correlation().edges().end(),
                  [&has_other_edges](const valhalla::PathEdge& e) {
                    has_other_edges = has_other_edges || !e.begin_node();
                  });

    // Iterate through edges and add to adjacency list
    for (const auto& edge : dest.correlation().edges()) {
      // If the destination is at a node, skip any outbound edges (so any
      // opposing inbound edges are not considered)
      if (has_other_edges && edge.begin_node()) {
        continue;
      }

      // Disallow any user avoided edges if the avoid location is behind the destination along the
      // edge
      GraphId edgeid(edge.graph_id());
      if (costing_->AvoidAsDestinationEdge(edgeid, edge.percent_along())) {
        continue;
      }

      // Get the directed edge
      graph_tile_ptr tile = graphreader.GetGraphTile(edgeid);
      const DirectedEdge* directededge = tile->directededge(edgeid);

      // Get the opposing directed edge, continue if we cannot get it
      graph_tile_ptr opp_tile = tile;
      GraphId opp_edge_id = graphreader.GetOpposingEdgeId(edgeid, opp_tile);
      if (!opp_edge_id.Is_Valid()) {
        continue;
      }
      const DirectedEdge* opp_dir_edge = graphreader.GetOpposingEdge(edgeid, opp_tile);

      // Get cost. Get distance along the remainder of this edge.
      // Use the directed edge for costing, as this is the forward direction
      // along the destination edge.
      uint8_t flow_sources;
      Cost edgecost = costing_->EdgeCost(directededge, tile, TimeInfo::invalid(), flow_sources);
      Cost cost = edgecost * edge.percent_along();
      uint32_t d = std::round(directededge->length() * edge.percent_along());

      // We need to penalize this location based on its score (distance in meters from input)
      // We assume the slowest speed you could travel to cover that distance to start/end the route
      // TODO: assumes 1m/s which is a maximum penalty this could vary per costing model
      cost.cost += edge.distance();

      // 2 adjustments related only to properly handle trivial routes:
      //   - "transition_cost" is used to store the traversed secs & length
      //   - "path_id" is used to store whether the opp edge is even allowed (e.g. no oneway)
      Cost ec(std::round(edgecost.secs), static_cast<uint32_t>(directededge->length()));
      BDEdgeLabel edge_label(kInvalidLabel, opp_edge_id, edgeid, opp_dir_edge, cost, mode_, ec, d,
                             !opp_dir_edge->not_thru(), !(costing_->IsClosed(directededge, tile)),
                             static_cast<bool>(flow_sources & kDefaultFlowMask),
                             InternalTurn::kNoTurn, kInvalidRestriction,
                             static_cast<uint8_t>(costing_->Allowed(opp_dir_edge, opp_tile)),
                             directededge->destonly() ||
                                 (costing_->is_hgv() && directededge->destonly_hgv()),
                             directededge->forwardaccess() & kTruckAccess);

      auto newsortcost =
          GetAstarHeuristic<MatrixExpansionType::reverse>(index,
                                                          tile->get_node_ll(opp_dir_edge->endnode()));
      edge_label.SetSortCost(cost.cost + newsortcost);
      // Set the initial not_thru flag to false. There is an issue with not_thru
      // flags on small loops. Set this to false here to override this for now.
      edge_label.set_not_thru(false);

      // Add EdgeLabel to the adjacency list (but do not set its status).
      // Set the predecessor edge index to invalid to indicate the origin
      // of the path. Set the origin flag
      uint32_t idx = edgelabel_[MATRIX_REV][index].size();
      edgelabel_[MATRIX_REV][index].push_back(std::move(edge_label));
      adjacency_[MATRIX_REV][index].add(idx);
      edgestatus_[MATRIX_REV][index].Set(opp_edge_id, EdgeSet::kTemporary, idx, opp_tile);
      (*targets_)[opp_edge_id].push_back(index);
    }
    index++;
  }
}

// Form the path from the edfge labels and optionally return the shape
std::string CostMatrix::RecostFormPath(GraphReader& graphreader,
                                       BestCandidate& connection,
                                       const valhalla::Location& source,
                                       const valhalla::Location& target,
                                       const uint32_t source_idx,
                                       const uint32_t target_idx,
                                       const baldr::TimeInfo& time_info,
                                       const bool invariant,
                                       const ShapeFormat shape_format) {
  // no need to look at source == target or missing connectivity
  if ((!has_time_ && shape_format == no_shape) || connection.cost.secs == 0.f ||
      connection.distance == kMaxCost) {
    return "";
  }

  // Get the indices where the connection occurs.
  uint32_t connedge_idx1 = edgestatus_[MATRIX_FORW][source_idx].Get(connection.edgeid).index();
  uint32_t connedge_idx2 = edgestatus_[MATRIX_REV][target_idx].Get(connection.opp_edgeid).index();

  // set of edges recovered from shortcuts (excluding shortcut's start edges)
  std::unordered_set<GraphId> recovered_inner_edges;

  // A place to keep the path
  std::vector<GraphId> path_edges;

  // Work backwards on the forward path
  graph_tile_ptr tile;
  for (auto edgelabel_index = connedge_idx1; edgelabel_index != kInvalidLabel;
       edgelabel_index = edgelabel_[MATRIX_FORW][source_idx][edgelabel_index].predecessor()) {
    const BDEdgeLabel& edgelabel = edgelabel_[MATRIX_FORW][source_idx][edgelabel_index];

    const DirectedEdge* edge = graphreader.directededge(edgelabel.edgeid(), tile);
    if (edge == nullptr) {
      throw tile_gone_error_t("CostMatrix::RecostPaths failed", edgelabel.edgeid());
    }

    if (edge->is_shortcut()) {
      auto superseded = graphreader.RecoverShortcut(edgelabel.edgeid());
      recovered_inner_edges.insert(superseded.begin() + 1, superseded.end());
      std::move(superseded.rbegin(), superseded.rend(), std::back_inserter(path_edges));
    } else
      path_edges.push_back(edgelabel.edgeid());
  }

  // Reverse the list
  std::reverse(path_edges.begin(), path_edges.end());

  // Append the reverse path from the destination - use opposing edges
  // The first edge on the reverse path is the same as the last on the forward
  // path, so get the predecessor.
  auto& target_edgelabels = edgelabel_[MATRIX_REV][target_idx];
  for (auto edgelabel_index = target_edgelabels[connedge_idx2].predecessor();
       edgelabel_index != kInvalidLabel;
       edgelabel_index = target_edgelabels[edgelabel_index].predecessor()) {
    const BDEdgeLabel& edgelabel = target_edgelabels[edgelabel_index];
    const DirectedEdge* opp_edge = nullptr;
    GraphId opp_edge_id = graphreader.GetOpposingEdgeId(edgelabel.edgeid(), opp_edge, tile);
    if (opp_edge == nullptr) {
      throw tile_gone_error_t("CostMatrix::RecostPaths failed", edgelabel.edgeid());
    }

    if (opp_edge->is_shortcut()) {
      auto superseded = graphreader.RecoverShortcut(opp_edge_id);
      recovered_inner_edges.insert(superseded.begin() + 1, superseded.end());
      std::move(superseded.begin(), superseded.end(), std::back_inserter(path_edges));
    } else
      path_edges.emplace_back(std::move(opp_edge_id));
  }

  const auto& source_edge = find_correlated_edge(source, path_edges.front());
  const auto& target_edge = find_correlated_edge(target, path_edges.back());
  float source_pct = static_cast<float>(source_edge.percent_along());
  float target_pct = static_cast<float>(target_edge.percent_along());

  // recost the path if this was a time-dependent expansion
  if (has_time_) {
    auto edge_itr = path_edges.begin();
    const auto edge_cb = [&edge_itr, &path_edges]() {
      return (edge_itr == path_edges.end()) ? GraphId{} : (*edge_itr++);
    };

    Cost new_cost{0.f, 0.f};
    const auto label_cb = [&new_cost](const EdgeLabel& label) { new_cost = label.cost(); };

    // recost edges in final path; ignore access restrictions
    try {
      sif::recost_forward(graphreader, *costing_, edge_cb, label_cb, source_pct, target_pct,
                          time_info, invariant, true);
    } catch (const std::exception& e) {
      LOG_ERROR(std::string("CostMatrix failed to recost final paths: ") + e.what());
      return "";
    }

    // update the existing best_connection cost
    connection.cost = new_cost;
  }

  // bail if no shape was requested
  if (shape_format == no_shape)
    return "";

  auto source_vertex = PointLL{source_edge.ll().lng(), source_edge.ll().lat()};
  auto target_vertex = PointLL{target_edge.ll().lng(), target_edge.ll().lat()};
  std::vector<PointLL> points;
  for (uint32_t i = 0; i < path_edges.size(); i++) {
    auto& path_edge = path_edges[i];
    auto is_first_edge = i == 0;
    auto is_last_edge = i == (path_edges.size() - 1);

    const auto* de = graphreader.directededge(path_edge, tile);
    auto edge_shp = tile->edgeinfo(de).shape();

    if (is_first_edge || is_last_edge) {
      if (!de->forward())
        std::reverse(edge_shp.begin(), edge_shp.end());

      float total = static_cast<float>(de->length());
      if (is_first_edge && is_last_edge) {
        trim_shape(source_pct * total, source_vertex, target_pct * total, target_vertex, edge_shp);
      } else if (is_first_edge) {
        trim_shape(source_pct * total, source_vertex, total, edge_shp.back(), edge_shp);
      } // last edge
      else {
        trim_shape(0, edge_shp.front(), target_pct * total, target_vertex, edge_shp);
      }

      points.insert(points.end(), edge_shp.begin() + !is_first_edge, edge_shp.end());
    } else {
      if (de->forward()) {
        points.insert(points.end(), edge_shp.begin() + 1, edge_shp.end());
      } else {
        points.insert(points.end(), edge_shp.rbegin() + 1, edge_shp.rend());
      }
    }
  }

  // encode to 6 precision for geojson as well, which the serializer expects
  return encode<decltype(points)>(points, shape_format != polyline5 ? 1e6 : 1e5);
}

template <const MatrixExpansionType expansion_direction, const bool FORWARD>
float CostMatrix::GetAstarHeuristic(const uint32_t loc_idx, const PointLL& ll) const {
  if (locs_status_[FORWARD][loc_idx].unfound_connections.empty()) {
    return 0.f;
  }

  auto min_cost = std::numeric_limits<float>::max();
  for (const auto other_idx : locs_status_[FORWARD][loc_idx].unfound_connections) {
    const auto cost = astar_heuristics_[FORWARD][other_idx].Get(ll);
    min_cost = std::min(cost, min_cost);
  }

  return min_cost;
};

} // namespace thor
} // namespace valhalla
