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
    : max_reserved_labels_count_(config.get<uint32_t>("max_reserved_labels_count_bidir_dijkstras",
                                                      kInitialEdgeLabelCountBidirDijkstra)),
      clear_reserved_memory_(config.get<bool>("clear_reserved_memory", false)),
      max_reserved_locations_count_(
          config.get<uint32_t>("max_reserved_locations_costmatrix", kMaxLocationReservation)),
      check_reverse_connections_(config.get<bool>("costmatrix_check_reverse_connection", false)),
      access_mode_(kAutoAccess),
      mode_(travel_mode_t::kDrive), locs_count_{0, 0}, locs_remaining_{0, 0},
      current_cost_threshold_(0),
      has_time_(false), targets_{new ReachedMap}, sources_{new ReachedMap} {
}

CostMatrix::~CostMatrix() {
}

float CostMatrix::GetCostThreshold(const float max_matrix_distance) {
  float cost_threshold;
  switch (mode_) {
    case travel_mode_t::kBicycle:
      cost_threshold = max_matrix_distance / kCostThresholdBicycleDivisor;
      break;
    case travel_mode_t::kPedestrian:
    case travel_mode_t::kPublicTransit:
      cost_threshold = max_matrix_distance / kCostThresholdPedestrianDivisor;
      break;
    case travel_mode_t::kDrive:
    default:
      cost_threshold = max_matrix_distance / kCostThresholdAutoDivisor;
  }

  // Increase the cost threshold to make sure requests near the max distance succeed.
  // Some costing models and locations require higher thresholds to succeed.
  return cost_threshold * 2.0f;
}

// Clear the temporary information generated during time + distance matrix
// construction.
void CostMatrix::clear() {
  // Clear the target edge markings
  targets_->clear();
  if (check_reverse_connections_)
    sources_->clear();

  // Clear all source adjacency lists, edge labels, and edge status
  // Resize and shrink_to_fit so all capacity is reduced.
  auto label_reservation = clear_reserved_memory_ ? 0 : max_reserved_labels_count_;
  auto locs_reservation = clear_reserved_memory_ ? 0 : max_reserved_locations_count_;
  for (const auto exp_dir : {MATRIX_FORW, MATRIX_REV}) {
    // resize all relevant structures down to configured amount of locations (25 default)
    if (locs_count_[exp_dir] > locs_reservation) {
      edgelabel_[exp_dir].resize(locs_reservation);
      edgelabel_[exp_dir].shrink_to_fit();
      adjacency_[exp_dir].resize(locs_reservation);
      adjacency_[exp_dir].shrink_to_fit();
      edgestatus_[exp_dir].resize(locs_reservation);
      edgestatus_[exp_dir].shrink_to_fit();
    }
    for (auto& iter : edgelabel_[exp_dir]) {
      if (iter.size() > label_reservation) {
        iter.resize(label_reservation);
        iter.shrink_to_fit();
      }
      iter.clear();
    }
    for (auto& iter : edgestatus_[exp_dir]) {
      iter.clear();
    }
    for (auto& iter : adjacency_[exp_dir]) {
      iter.clear();
    }
    hierarchy_limits_[exp_dir].clear();
    locs_status_[exp_dir].clear();
  }
  best_connection_.clear();
  ignore_hierarchy_limits_ = false;
}

// Form a time distance matrix from the set of source locations
// to the set of target locations.
void CostMatrix::SourceToTarget(Api& request,
                                baldr::GraphReader& graphreader,
                                const sif::mode_costing_t& mode_costing,
                                const sif::travel_mode_t mode,
                                const float max_matrix_distance,
                                const bool has_time,
                                const bool invariant,
                                const ShapeFormat& shape_format) {

  LOG_INFO("matrix::CostMatrix");
  request.mutable_matrix()->set_algorithm(Matrix::CostMatrix);

  // Set the mode and costing
  mode_ = mode;
  costing_ = mode_costing[static_cast<uint32_t>(mode_)];
  access_mode_ = costing_->access_mode();
  has_time_ = has_time;

  auto& source_location_list = *request.mutable_options()->mutable_sources();
  auto& target_location_list = *request.mutable_options()->mutable_targets();

  current_cost_threshold_ = GetCostThreshold(max_matrix_distance);

  auto time_infos = SetOriginTimes(source_location_list, graphreader);

  // Initialize best connections and status. Any locations that are the
  // same get set to 0 time, distance and are not added to the remaining
  // location set.
  Initialize(source_location_list, target_location_list);

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
  while (true) {
    // First iterate over all targets, then over all sources: we only for sure
    // check the connection between both trees on the forward search, so reverse
    // has to come first
    for (uint32_t i = 0; i < locs_count_[MATRIX_REV]; i++) {
      if (locs_status_[MATRIX_REV][i].threshold > 0) {
        locs_status_[MATRIX_REV][i].threshold--;
        Expand<MatrixExpansionType::reverse>(i, n, graphreader);
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
                locs_status_[MATRIX_FORW][i].threshold = -1;
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
        Expand<MatrixExpansionType::forward>(i, n, graphreader, time_infos[i], invariant);
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
                locs_status_[MATRIX_REV][i].threshold = -1;
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
    if (interrupt_ && (n++ % kInterruptIterationsInterval) == 0) {
      (*interrupt_)();
    }
  }

  // Form the matrix PBF output
  graph_tile_ptr tile;
  uint32_t count = 0;
  valhalla::Matrix& matrix = *request.mutable_matrix();
  reserve_pbf_arrays(matrix, best_connection_.size());
  for (auto& connection : best_connection_) {
    uint32_t target_idx = count % target_location_list.size();
    uint32_t source_idx = count / target_location_list.size();

    // first recost and form the path, if desired (either time and/or geometry requested)
    const auto shape = RecostFormPath(graphreader, connection, source_location_list[source_idx],
                                      target_location_list[target_idx], source_idx, target_idx,
                                      time_infos[source_idx], invariant, shape_format);

    float time = connection.cost.secs;
    if (time < kMaxCost) {
      auto date_time =
          DateTime::offset_date(source_location_list[source_idx].date_time(),
                                time_infos[source_idx].timezone_index,
                                graphreader.GetTimezoneFromEdge(edgelabel_[MATRIX_REV][target_idx]
                                                                    .front()
                                                                    .edgeid(),
                                                                tile),
                                time);
      auto* pbf_date_time = matrix.mutable_date_times()->Add();
      *pbf_date_time = date_time;
    }
    matrix.mutable_from_indices()->Set(count, source_idx);
    matrix.mutable_to_indices()->Set(count, target_idx);
    matrix.mutable_distances()->Set(count, connection.distance);
    matrix.mutable_times()->Set(count, time);
    auto* pbf_shape = matrix.mutable_shapes()->Add();
    *pbf_shape = shape;
    count++;
  }
}

// Initialize all time distance to "not found". Any locations that
// are the same get set to 0 time, distance and do not add to the
// remaining locations set.
void CostMatrix::Initialize(
    const google::protobuf::RepeatedPtrField<valhalla::Location>& source_locations,
    const google::protobuf::RepeatedPtrField<valhalla::Location>& target_locations) {

  locs_count_[MATRIX_FORW] = source_locations.size();
  locs_count_[MATRIX_REV] = target_locations.size();

  const auto& hlimits = costing_->GetHierarchyLimits();
  ignore_hierarchy_limits_ =
      std::all_of(hlimits.begin() + 1, hlimits.begin() + TileHierarchy::levels().size(),
                  [](const HierarchyLimits& limits) {
                    return limits.max_up_transitions == kUnlimitedTransitions;
                  });

  // Add initial sources status
  for (const auto exp_dir : {MATRIX_FORW, MATRIX_REV}) {
    const auto count = locs_count_[exp_dir];
    locs_status_[exp_dir].reserve(count);
    hierarchy_limits_[exp_dir].resize(count);
    adjacency_[exp_dir].resize(count);
    edgestatus_[exp_dir].resize(count);
    edgelabel_[exp_dir].resize(count);
    for (uint32_t i = 0; i < count; i++) {
      // Allocate the adjacency list and hierarchy limits for this source.
      // Use the cost threshold to size the adjacency list.
      edgelabel_[exp_dir][i].reserve(max_reserved_labels_count_);
      adjacency_[exp_dir][i].reuse(0, current_cost_threshold_, costing_->UnitSize(),
                                   &edgelabel_[exp_dir][i]);
      locs_status_[exp_dir].emplace_back(kMaxThreshold);
      hierarchy_limits_[exp_dir][i] = hlimits;
    }
  }

  // Initialize best connection
  GraphId empty;
  Cost trivial_cost(0.0f, 0.0f);
  Cost max_cost(kMaxCost, kMaxCost);
  best_connection_.reserve(locs_count_[MATRIX_FORW] * locs_count_[MATRIX_REV]);
  for (uint32_t i = 0; i < locs_count_[MATRIX_FORW]; i++) {
    for (uint32_t j = 0; j < locs_count_[MATRIX_REV]; j++) {
      if (equals(source_locations.Get(i).ll(), target_locations.Get(j).ll())) {
        best_connection_.emplace_back(empty, empty, trivial_cost, 0.0f);
        best_connection_.back().found = true;
      } else {
        best_connection_.emplace_back(empty, empty, max_cost, static_cast<uint32_t>(kMaxCost));
        locs_status_[MATRIX_FORW][i].unfound_connections.insert(j);
        locs_status_[MATRIX_REV][j].unfound_connections.insert(i);
      }
    }
  }

  // Set the remaining number of sources and targets
  locs_remaining_[MATRIX_FORW] = 0;
  for (const auto& s : locs_status_[MATRIX_FORW]) {
    if (!s.unfound_connections.empty()) {
      locs_remaining_[MATRIX_FORW]++;
    }
  }
  locs_remaining_[MATRIX_REV] = 0;
  for (const auto& t : locs_status_[MATRIX_REV]) {
    if (!t.unfound_connections.empty()) {
      locs_remaining_[MATRIX_REV]++;
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
  uint8_t restriction_idx = -1;
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
  // less cost the predecessor is updated along with new cost and distance.
  if (meta.edge_status->set() == EdgeSet::kTemporary) {
    BDEdgeLabel& lab = edgelabel_[FORWARD][index][meta.edge_status->index()];
    if (newcost.cost < lab.cost().cost) {
      adj.decrease(meta.edge_status->index(), newcost.cost);
      lab.Update(pred_idx, newcost, newcost.cost, tc, pred_dist, restriction_idx);
    }
    // Returning true since this means we approved the edge
    return true;
  }

  // Get end node tile (skip if tile is not found) and opposing edge Id
  if (t2 == nullptr && !get_opp_edge_data()) {
    return false;
  }

  // Add edge label, add to the adjacency list and set edge status
  uint32_t idx = edgelabels.size();
  *meta.edge_status = {EdgeSet::kTemporary, idx};
  if (FORWARD) {
    edgelabels.emplace_back(pred_idx, meta.edge_id, opp_edge_id, meta.edge, newcost, mode_, tc,
                            pred_dist, (pred.not_thru_pruning() || !meta.edge->not_thru()),
                            (pred.closure_pruning() || !costing_->IsClosed(meta.edge, tile)),
                            static_cast<bool>(flow_sources & kDefaultFlowMask),
                            costing_->TurnType(pred.opp_local_idx(), nodeinfo, meta.edge),
                            restriction_idx, 0,
                            meta.edge->destonly() ||
                                (costing_->is_hgv() && meta.edge->destonly_hgv()));
  } else {
    edgelabels.emplace_back(pred_idx, meta.edge_id, opp_edge_id, meta.edge, newcost, mode_, tc,
                            pred_dist, (pred.not_thru_pruning() || !meta.edge->not_thru()),
                            (pred.closure_pruning() || !costing_->IsClosed(meta.edge, tile)),
                            static_cast<bool>(flow_sources & kDefaultFlowMask),
                            costing_->TurnType(meta.edge->localedgeidx(), nodeinfo, opp_edge,
                                               opp_pred_edge),
                            restriction_idx, 0,
                            opp_edge->destonly() || (costing_->is_hgv() && opp_edge->destonly_hgv()));
  }
  adj.add(idx);
  // mark the edge as settled for the connection check
  if (!FORWARD) {
    (*targets_)[meta.edge_id].push_back(index);
  } else if (check_reverse_connections_) {
    (*sources_)[meta.edge_id].push_back(index);
  }

  // setting this edge as reached
  if (expansion_callback_) {
    expansion_callback_(graphreader, meta.edge_id, pred.edgeid(), "costmatrix", "r", newcost.secs,
                        pred_dist, newcost.cost);
  }

  return true;
}

template <const MatrixExpansionType expansion_direction, const bool FORWARD>
bool CostMatrix::Expand(const uint32_t index,
                        const uint32_t n,
                        baldr::GraphReader& graphreader,
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
  if (pred.cost().secs > current_cost_threshold_) {
    locs_status_[FORWARD][index].threshold = 0;
    return false;
  }

  // Settle this edge and log it if requested
  auto& edgestatus = edgestatus_[FORWARD][index];
  edgestatus.Update(pred.edgeid(), EdgeSet::kPermanent);
  if (expansion_callback_) {
    auto prev_pred =
        pred.predecessor() == kInvalidLabel ? GraphId{} : edgelabels[pred.predecessor()].edgeid();
    expansion_callback_(graphreader, pred.edgeid(), prev_pred, "costmatrix", "s", pred.cost().secs,
                        pred.path_distance(), pred.cost().cost);
  }

  if (FORWARD) {
    CheckForwardConnections(index, pred, n, graphreader);
  } else if (check_reverse_connections_) {
    CheckReverseConnections(index, pred, n, graphreader);
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
                                         const BDEdgeLabel& pred,
                                         const uint32_t n,
                                         GraphReader& graphreader) {

  // Disallow connections that are part of an uturn on an internal edge
  if (pred.internal_turn() != InternalTurn::kNoTurn) {
    return;
  }
  // Disallow connections that are part of a complex restriction.
  // TODO - validate that we do not need to "walk" the paths forward
  // and backward to see if they match a restriction.
  if (pred.on_complex_rest()) {
    // TODO(nils): bidir a* is digging deeper
    return;
  }

  // Get the opposing edge. Get a list of target locations whose reverse
  // search has reached this edge.
  GraphId oppedge = pred.opp_edgeid();
  auto targets = targets_->find(oppedge);
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
    const auto& opp_edgestate = edgestatus_[MATRIX_REV][target];
    EdgeStatusInfo oppedgestatus = opp_edgestate.Get(oppedge);
    const auto& opp_edgelabels = edgelabel_[MATRIX_REV][target];
    uint32_t opp_predidx = opp_edgelabels[oppedgestatus.index()].predecessor();
    const BDEdgeLabel& opp_el = opp_edgelabels[oppedgestatus.index()];

    // Special case - common edge for source and target are both initial edges
    if (pred.predecessor() == kInvalidLabel && opp_predidx == kInvalidLabel) {
      // TODO: shouldnt this use seconds? why is this using cost!?
      float s = std::abs(pred.cost().secs + opp_el.cost().secs - opp_el.transition_cost().cost);

      // Update best connection and set found = true.
      // distance computation only works with the casts.
      uint32_t d =
          std::abs(static_cast<int>(pred.path_distance()) + static_cast<int>(opp_el.path_distance()) -
                   static_cast<int>(opp_el.transition_cost().secs));
      best_connection_[idx].Update(pred.edgeid(), oppedge, Cost(s, s), d);
      best_connection_[idx].found = true;

      // Update status and update threshold if this is the last location
      // to find for this source or target
      UpdateStatus(source, target);
    } else {
      float oppcost = (opp_predidx == kInvalidLabel) ? 0.f : opp_edgelabels[opp_predidx].cost().cost;
      float c = pred.cost().cost + oppcost + opp_el.transition_cost().cost;

      // Check if best connection
      if (c < best_connection_[idx].cost.cost) {
        float oppsec = (opp_predidx == kInvalidLabel) ? 0.f : opp_edgelabels[opp_predidx].cost().secs;
        uint32_t oppdist =
            (opp_predidx == kInvalidLabel) ? 0U : opp_edgelabels[opp_predidx].path_distance();
        float s = pred.cost().secs + oppsec + opp_el.transition_cost().secs;
        uint32_t d = pred.path_distance() + oppdist;

        // Update best connection and set a threshold
        best_connection_[idx].Update(pred.edgeid(), oppedge, Cost(c, s), d);
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
      auto prev_pred = pred.predecessor() == kInvalidLabel
                           ? GraphId{}
                           : edgelabel_[MATRIX_FORW][source][pred.predecessor()].edgeid();
      expansion_callback_(graphreader, pred.edgeid(), prev_pred, "costmatrix", "c", pred.cost().secs,
                          pred.path_distance(), pred.cost().cost);
    }
  }

  return;
}

void CostMatrix::CheckReverseConnections(const uint32_t target,
                                         const BDEdgeLabel& rev_pred,
                                         const uint32_t n,
                                         GraphReader& graphreader) {

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
    uint32_t idx = source * locs_count_[MATRIX_REV] + target;
    if (best_connection_[idx].found) {
      continue;
    }

    // Update any targets whose threshold has been reached
    if (best_connection_[idx].max_iterations > 0 && n > best_connection_[idx].max_iterations) {
      best_connection_[idx].found = true;
      continue;
    }

    // If this edge has been reached then a shortest path has been found
    // to the end node of this directed edge.
    EdgeStatusInfo oppedgestatus = edgestatus_[MATRIX_FORW][source].Get(fwd_edgeid);
    if (oppedgestatus.set() != EdgeSet::kUnreachedOrReset) {
      const auto& edgelabels = edgelabel_[MATRIX_FORW][source];
      uint32_t predidx = edgelabels[oppedgestatus.index()].predecessor();
      const BDEdgeLabel& opp_el = edgelabels[oppedgestatus.index()];

      // Special case - common edge for source and target are both initial edges
      if (rev_pred.predecessor() == kInvalidLabel && predidx == kInvalidLabel) {
        // TODO: shouldnt this use seconds? why is this using cost!?
        float s = std::abs(rev_pred.cost().secs + opp_el.cost().secs - opp_el.transition_cost().cost);

        // Update best connection and set found = true.
        // distance computation only works with the casts.
        uint32_t d = std::abs(static_cast<int>(rev_pred.path_distance()) +
                              static_cast<int>(opp_el.path_distance()) -
                              static_cast<int>(opp_el.transition_cost().secs));
        best_connection_[idx].Update(fwd_edgeid, rev_pred.edgeid(), Cost(s, s), d);
        best_connection_[idx].found = true;

        // Update status and update threshold if this is the last location
        // to find for this source or target
        UpdateStatus(source, target);
      } else {
        float oppcost = (predidx == kInvalidLabel) ? 0 : edgelabels[predidx].cost().cost;
        float c = rev_pred.cost().cost + oppcost + opp_el.transition_cost().cost;

        // Check if best connection
        if (c < best_connection_[idx].cost.cost) {
          float oppsec = (predidx == kInvalidLabel) ? 0 : edgelabels[predidx].cost().secs;
          uint32_t oppdist = (predidx == kInvalidLabel) ? 0 : edgelabels[predidx].path_distance();
          float s = rev_pred.cost().secs + oppsec + opp_el.transition_cost().secs;
          uint32_t d = rev_pred.path_distance() + oppdist;

          // Update best connection and set a threshold
          best_connection_[idx].Update(fwd_edgeid, rev_pred.edgeid(), Cost(c, s), d);
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
        auto prev_pred = rev_pred.predecessor() == kInvalidLabel
                             ? GraphId{}
                             : edgelabel_[MATRIX_REV][source][rev_pred.predecessor()].edgeid();
        expansion_callback_(graphreader, rev_pred.edgeid(), prev_pred, "costmatrix", "c",
                            rev_pred.cost().secs, rev_pred.path_distance(), rev_pred.cost().cost);
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
      const DirectedEdge* directededge = tile->directededge(edgeid);
      GraphId oppedge = graphreader.GetOpposingEdgeId(edgeid);

      // Get cost. Get distance along the remainder of this edge.
      uint8_t flow_sources;
      Cost edgecost = costing_->EdgeCost(directededge, tile, time_infos[index], flow_sources);
      Cost cost = edgecost * (1.0f - edge.percent_along());
      uint32_t d = std::round(directededge->length() * (1.0f - edge.percent_along()));

      // We need to penalize this location based on its score (distance in meters from input)
      // We assume the slowest speed you could travel to cover that distance to start/end the route
      // TODO: assumes 1m/s which is a maximum penalty this could vary per costing model
      cost.cost += edge.distance();

      // Store the edge cost and length in the transition cost (so we can
      // recover the full length and cost for cases where origin and
      // destination are on the same edge
      Cost ec(std::round(edgecost.secs), static_cast<uint32_t>(directededge->length()));

      // Set the initial not_thru flag to false. There is an issue with not_thru
      // flags on small loops. Set this to false here to override this for now.
      BDEdgeLabel edge_label(kInvalidLabel, edgeid, oppedge, directededge, cost, mode_, ec, d, false,
                             true, static_cast<bool>(flow_sources & kDefaultFlowMask),
                             InternalTurn::kNoTurn, -1, 0,
                             directededge->destonly() ||
                                 (costing_->is_hgv() && directededge->destonly_hgv()));
      edge_label.set_not_thru(false);

      // Add EdgeLabel to the adjacency list (but do not set its status).
      // Set the predecessor edge index to invalid to indicate the origin
      // of the path.
      uint32_t idx = edgelabel_[MATRIX_FORW][index].size();
      edgelabel_[MATRIX_FORW][index].push_back(std::move(edge_label));
      adjacency_[MATRIX_FORW][index].add(idx);
      edgestatus_[MATRIX_FORW][index].Set(edgeid, EdgeSet::kUnreachedOrReset, idx, tile);
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
      GraphId opp_edge_id = graphreader.GetOpposingEdgeId(edgeid);
      if (!opp_edge_id.Is_Valid()) {
        continue;
      }
      const DirectedEdge* opp_dir_edge = graphreader.GetOpposingEdge(edgeid);

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

      // Store the edge cost and length in the transition cost (so we can
      // recover the full length and cost for cases where origin and
      // destination are on the same edge
      Cost ec(std::round(edgecost.secs), static_cast<uint32_t>(directededge->length()));

      // Set the initial not_thru flag to false. There is an issue with not_thru
      // flags on small loops. Set this to false here to override this for now.
      BDEdgeLabel edge_label(kInvalidLabel, opp_edge_id, edgeid, opp_dir_edge, cost, mode_, ec, d,
                             false, true, static_cast<bool>(flow_sources & kDefaultFlowMask),
                             InternalTurn::kNoTurn, -1, 0,
                             opp_dir_edge->destonly() ||
                                 (costing_->is_hgv() && opp_dir_edge->destonly_hgv()));
      edge_label.set_not_thru(false);

      // Add EdgeLabel to the adjacency list (but do not set its status).
      // Set the predecessor edge index to invalid to indicate the origin
      // of the path. Set the origin flag
      uint32_t idx = edgelabel_[MATRIX_REV][index].size();
      edgelabel_[MATRIX_REV][index].push_back(std::move(edge_label));
      adjacency_[MATRIX_REV][index].add(idx);
      edgestatus_[MATRIX_REV][index].Set(opp_edge_id, EdgeSet::kUnreachedOrReset, idx,
                                         graphreader.GetGraphTile(opp_edge_id));
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

  // TODO(nils): bug with trivial routes https://github.com/valhalla/valhalla/issues/4433
  // remove this whole block below once that's fixed
  if (path_edges.size() == 1 && source_pct > target_pct) {
    // it found the wrong direction, so let's turn that around
    auto opp_id = graphreader.GetOpposingEdgeId(path_edges[0]);
    path_edges.clear();
    path_edges.emplace_back(opp_id);
    source_pct = 1.f - source_pct;
    target_pct = 1.f - target_pct;
  }
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
  for (const auto& path_edge : path_edges) {
    auto is_first_edge = path_edge == path_edges.front();
    auto is_last_edge = path_edge == path_edges.back();

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

} // namespace thor
} // namespace valhalla
