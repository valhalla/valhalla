#ifndef VALHALLA_SIF_EDGELABEL_H_
#define VALHALLA_SIF_EDGELABEL_H_

#include <cstdint>
#include <limits>
#include <string.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/sif/costconstants.h>

namespace valhalla {
namespace sif {

constexpr uint32_t kInitialEdgeLabelCountAstar = 2000000;
constexpr uint32_t kInitialEdgeLabelCountBidirAstar = 1000000;
constexpr uint32_t kInitialEdgeLabelCountDijkstras = 4000000;
constexpr uint32_t kInitialEdgeLabelCountBidirDijkstra = 2000000;

/**
 * Labeling information for shortest path algorithm. Contains cost,
 * predecessor, current time, and assorted information required during
 * construction of the shortest path and for reconstructing the path
 * upon completion.
 * The base EdgeLabel class contains all necessary information for costing
 * and for an A* (forward search) algorithm. Derived classes support
 * additional information required other path algorithms.
 */
class EdgeLabel {
public:
  /**
   * Default constructor.
   */
  EdgeLabel()
      : predecessor_(baldr::kInvalidLabel), path_distance_(0), restrictions_(0),
        edgeid_(baldr::kInvalidGraphId), opp_index_(0), opp_local_idx_(0), mode_(0),
        endnode_(baldr::kInvalidGraphId), use_(0), classification_(0), shortcut_(0), dest_only_(0),
        origin_(0), destination_(0), toll_(0), not_thru_(0), deadend_(0), on_complex_rest_(0),
        closure_pruning_(0), has_measured_speed_(0), path_id_(0), restriction_idx_(0),
        internal_turn_(0), unpaved_(0), cost_(0, 0), sortcost_(0), distance_(0),
        transition_cost_(0, 0) {
    assert(path_id_ <= baldr::kMaxMultiPathId);
  }

  /**
   * Constructor with values.
   * @param predecessor         Index into the edge label list for the predecessor
   *                            directed edge in the shortest path.
   * @param edgeid              Directed edge Id.
   * @param edge                Directed edge.
   * @param cost                True cost (cost and time in seconds) to the edge.
   * @param sortcost            Cost for sorting (includes A* heuristic)
   * @param dist                Distance to the destination (meters)
   * @param mode                Mode of travel along this edge.
   * @param path_distance       Accumulated path distance
   * @param transition_cost     Transition cost
   * @param restriction_idx     If this label has restrictions, the index where the restriction is
   * found
   * @param closure_pruning     Should closure pruning be enabled on this path?
   * @param has_measured_speed  Do we have any of the measured speed types set?
   * @param internal_turn       Did we make an turn on a short internal edge.
   * @param path_id             When searching more than one path at a time this denotes which path
   * the this label is tracking
   */
  EdgeLabel(const uint32_t predecessor,
            const baldr::GraphId& edgeid,
            const baldr::DirectedEdge* edge,
            const Cost& cost,
            const float sortcost,
            const float dist,
            const TravelMode mode,
            const uint32_t path_distance,
            const Cost& transition_cost,
            const uint8_t restriction_idx,
            const bool closure_pruning,
            const bool has_measured_speed,
            const InternalTurn internal_turn,
            const uint8_t path_id = 0)
      : predecessor_(predecessor), path_distance_(path_distance), restrictions_(edge->restrictions()),
        edgeid_(edgeid), opp_index_(edge->opp_index()), opp_local_idx_(edge->opp_local_idx()),
        mode_(static_cast<uint32_t>(mode)), endnode_(edge->endnode()),
        use_(static_cast<uint32_t>(edge->use())),
        classification_(static_cast<uint32_t>(edge->classification())), shortcut_(edge->shortcut()),
        dest_only_(edge->destonly()), origin_(0), destination_(0), toll_(edge->toll()),
        not_thru_(edge->not_thru()), deadend_(edge->deadend()),
        on_complex_rest_(edge->part_of_complex_restriction() || edge->start_restriction() ||
                         edge->end_restriction()),
        closure_pruning_(closure_pruning), has_measured_speed_(has_measured_speed), path_id_(path_id),
        restriction_idx_(restriction_idx), internal_turn_(static_cast<uint8_t>(internal_turn)),
        unpaved_(edge->unpaved()), cost_(cost), sortcost_(sortcost), distance_(dist),
        transition_cost_(transition_cost) {
    assert(path_id_ <= baldr::kMaxMultiPathId);
  }

  /**
   * Update an existing edge label with new predecessor and cost information.
   * The mode, edge Id, and end node remain the same.
   * @param predecessor Predecessor directed edge in the shortest path.
   * @param cost        True cost (and elapsed time in seconds) to the edge.
   * @param sortcost    Cost for sorting (includes A* heuristic).
   * @param transition_cost Transition cost
   * @param restriction_idx Does the edge have time dependent restrictions.
   */
  void Update(const uint32_t predecessor,
              const Cost& cost,
              const float sortcost,
              const Cost& transition_cost,
              const uint8_t restriction_idx) {
    predecessor_ = predecessor;
    cost_ = cost;
    sortcost_ = sortcost;
    transition_cost_ = transition_cost;
    restriction_idx_ = restriction_idx;
  }

  /**
   * Update an existing edge label with new predecessor and cost information.
   * Update transit information: prior stop Id will stay the same but trip Id
   * and block Id may change (a new trip at an earlier departure time).
   * The mode, edge Id, and end node remain the same.
   * @param predecessor      Predecessor directed edge in the shortest path.
   * @param cost             True cost (and elapsed time in seconds) to the edge.
   * @param sortcost         Cost for sorting (includes A* heuristic).
   * @param path_distance    Accumulated path distance.
   * @param transition_cost  Transition cost
   * @param restriction_idx  If this label has restrictions, the index where the restriction is found
   */
  void Update(const uint32_t predecessor,
              const Cost& cost,
              const float sortcost,
              const uint32_t path_distance,
              const Cost& transition_cost,
              const uint8_t restriction_idx) {
    predecessor_ = predecessor;
    cost_ = cost;
    sortcost_ = sortcost;
    path_distance_ = path_distance;
    transition_cost_ = transition_cost;
    restriction_idx_ = restriction_idx;
  }

  /**
   * Get the predecessor edge label.
   * @return Predecessor edge label.
   */
  uint32_t predecessor() const {
    return predecessor_;
  }

  /**
   * Get the GraphId of this directed edge.
   * @return  Returns the GraphId of this directed edge.
   */
  baldr::GraphId edgeid() const {
    return baldr::GraphId(edgeid_);
  }

  /**
   * Get the end node of this directed edge. Allows the A* algorithm
   * to expand the search from this end node, without having to read
   * the directed edge again.
   * @return  Returns the GraphId of the end node of this directed edge.
   */
  baldr::GraphId endnode() const {
    return baldr::GraphId(endnode_);
  }

  /**
   * Get the cost from the origin to this directed edge.
   * @return  Returns the cost (units are based on the costing method)
   *          and elapsed time (seconds) to the end of the directed edge.
   */
  const Cost& cost() const {
    return cost_;
  }

  /**
   * Get the sort cost from the origin to this directed edge. The sort
   * cost includes the A* heuristic.
   * @return  Returns the sort cost (units are based on the costing method).
   */
  float sortcost() const {
    return sortcost_;
  }

  /**
   * Set the sort cost from the origin to this directed edge. The sort
   * cost includes the A* heuristic.
   * @param sortcost Sort cost (units are based on the costing method).
   */
  void SetSortCost(float sortcost) {
    sortcost_ = sortcost;
  }

  /**
   * Get the distance to the destination.
   * In case of bidirectional search it may represent the distance to the start point:
   * origin for the forward search and destination for the reverse search.
   * @return  Returns the distance in meters.
   */
  float distance() const {
    return distance_;
  }

  /**
   * Get the use of the directed edge.
   * @return  Returns edge use.
   */
  baldr::Use use() const {
    return static_cast<baldr::Use>(use_);
  }

  /**
   * Get the opposing index - for bidirectional A*.
   * @return  Returns the opposing directed edge index to the incoming edge.
   */
  uint32_t opp_index() const {
    return opp_index_;
  }

  /**
   * Get the opposing local index. This is the index of the incoming edge
   * (on the local hierarchy) at the end node of the predecessor directed
   * edge. This is used for edge transition costs and Uturn detection.
   * @return  Returns the local index of the incoming edge.
   */
  uint32_t opp_local_idx() const {
    return opp_local_idx_;
  }

  /**
   * Get the restriction mask at the end node. Each bit set to 1 indicates a
   * turn restriction onto the directed edge with matching local edge index.
   * @return  Returns the restriction mask.
   */
  uint32_t restrictions() const {
    return restrictions_;
  }

  /**
   * Get the shortcut flag.
   * @return  Returns true if the prior edge was a shortcut, false if not.
   */
  bool shortcut() const {
    return shortcut_;
  }

  /**
   * Get the travel mode along this edge.
   * @return  Returns the travel mode.
   */
  TravelMode mode() const {
    return static_cast<TravelMode>(mode_);
  }

  /**
   * Get the dest only flag.
   * @return  Returns true if the edge is part of a private or no through road that allows access
   *          only if required to get to a destination?
   */
  bool destonly() const {
    return dest_only_;
  }

  /**
   * Is this edge an origin edge?
   * @return  Returns true if this edge is an origin edge.
   */
  bool origin() const {
    return origin_;
  }

  /**
   * Sets this edge as an origin.
   */
  void set_origin() {
    origin_ = true;
  }

  /**
   * Is this edge a destination edge?
   * @return  Returns true if this edge is a destination edge.
   */
  bool destination() const {
    return destination_;
  }

  /**
   * Sets this edge as a destination.
   */
  void set_destination() {
    destination_ = true;
  }
  /**
   * Get the restriction idx, 255 means no restriction
   */
  uint8_t restriction_idx() const {
    return restriction_idx_;
  }
  /**
   * Get the internal_turn.
   * @return  Returns value from InternalTurn that indicates if we made a turn on a short internal
   * edge.
   */
  InternalTurn internal_turn() const {
    return static_cast<InternalTurn>(internal_turn_);
  }
  /**
   * Does this edge have a toll?
   * @return  Returns true if this edge has a toll.
   */
  bool toll() const {
    return toll_;
  }

  /**
   * Get the current path distance in meters.
   * @return  Returns the current path distance.
   */
  uint32_t path_distance() const {
    return path_distance_;
  }

  /**
   * Get the predecessor road classification.
   * @return Predecessor road classification.
   */
  baldr::RoadClass classification() const {
    return static_cast<baldr::RoadClass>(classification_);
  }

  /**
   * Operator < used for sorting.
   */
  bool operator<(const EdgeLabel& other) const {
    return sortcost() < other.sortcost();
  }

  /**
   * Is this edge part of a complex restriction.
   * @return  Returns true if the edge is part of a complex restriction.
   */
  bool on_complex_rest() const {
    return on_complex_rest_;
  }

  /**
   * Is this edge not-through
   * @return  Returns true if the edge is not thru.
   */
  bool not_thru() const {
    return not_thru_;
  }

  /**
   * Set the not-through flag for this edge.
   * @param  not_thru  True if the edge is not thru.
   */
  void set_not_thru(const bool not_thru) {
    not_thru_ = not_thru;
  }

  /**
   * Is this edge a dead end.
   * @return  Returns true if the edge is a dead end.
   */
  bool deadend() const {
    return deadend_;
  }

  void set_deadend(bool is_deadend) {
    deadend_ = is_deadend;
  }

  /**
   * Get the transition cost. This is used in the bidirectional A*
   * to determine the cost at the connection. But is also used for general stats
   * @return  Returns the transition cost (including penalties).
   */
  sif::Cost transition_cost() const {
    return transition_cost_;
  }

  /**
   * Returns the location/path id (index) of the path that this label is tracking. Useful when your
   * algorithm tracks multiple path expansions at the same time
   *
   * @return  the id of the path that this label is tracking
   */
  uint8_t path_id() const {
    return path_id_;
  }

  /**
   * Should closure pruning be enabled on this path?
   * @return Returns true if closure pruning should be enabled.
   */
  bool closure_pruning() const {
    return closure_pruning_;
  }

  /**
   * Do we have any of the measured speed types set?
   * @return Returns true if we have any of the measured speed types set.
   */
  bool has_measured_speed() const {
    return has_measured_speed_;
  }

  /**
   * Get the unpaved flag.
   * @return Returns true if the edge is an unpaved road, otherwise false.
   */
  bool unpaved() const {
    return unpaved_;
  }

protected:
  // predecessor_: Index to the predecessor edge label information.
  // Note: invalid predecessor value uses all 32 bits (so if this needs to
  // be part of a bit field make sure kInvalidLabel is changed.
  uint32_t predecessor_;

  // path_distance_: Accumulated path distance in meters.
  // restriction_:   Bit mask of edges (by local edge index at the end node)
  //                 that are restricted (simple turn restrictions)
  uint32_t path_distance_ : 25;
  uint32_t restrictions_ : 7;

  /**
   * edgeid_:        Graph Id of the edge.
   * opp_index_:     Index at the end node of the opposing directed edge.
   * opp_local_idx_: Index at the end node of the opposing local edge. This
   *                 value can be compared to the directed edge local_edge_idx
   *                 for edge transition costing and Uturn detection.
   * mode_:          Current transport mode.
   */
  uint64_t edgeid_ : 46;
  uint64_t opp_index_ : 7;
  uint64_t opp_local_idx_ : 7;
  uint64_t mode_ : 4;

  /**
   * endnode_:            GraphId of the end node of the edge. This allows the
   *                      expansion to occur by reading the node and not having
   *                      to re-read the directed edge.
   * use_:                Use of the prior edge.
   * classification_:     Road classification
   * shortcut_:           Was the prior edge a shortcut edge?
   * dest_only_:          Was the prior edge destination only?
   * origin_:             True if this is an origin edge.
   * toll_:               Edge is toll.
   * not_thru_:           Flag indicating edge is not_thru.
   * deadend_:            Flag indicating edge is a dead-end.
   * on_complex_rest_:    Part of a complex restriction.
   * closure_pruning_:    Was closure pruning active on prior edge?
   * has_measured_speed_: Do we have any of the measured speed types set?
   */
  uint64_t endnode_ : 46;
  uint64_t use_ : 6;
  uint64_t classification_ : 3;
  uint64_t shortcut_ : 1;
  uint64_t dest_only_ : 1;
  uint64_t origin_ : 1;
  uint64_t destination_ : 1;
  uint64_t toll_ : 1;
  uint64_t not_thru_ : 1;
  uint64_t deadend_ : 1;
  uint64_t on_complex_rest_ : 1;
  uint64_t closure_pruning_ : 1;
  uint64_t has_measured_speed_ : 1;

  // path id can be used to track more than one path at the same time in the same labelset
  // its limited to 7 bits because edgestatus only had 7 and matching made sense to reduce confusion
  uint32_t path_id_ : 7;
  uint32_t restriction_idx_ : 8;
  // internal_turn_ Did we make an turn on a short internal edge.
  uint32_t internal_turn_ : 2;
  // Flag indicating edge is an unpaved road.
  uint32_t unpaved_ : 1;
  uint32_t spare : 13;

  Cost cost_;      // Cost and elapsed time along the path.
  float sortcost_; // Sort cost - includes A* heuristic.
  float distance_; // Distance to the destination.

  // Was originally used for reverse search path to remove extra time where paths intersected
  // but its now used everywhere to measure the difference in time along the edge vs at the node
  Cost transition_cost_;
};

/**
 * EdgeLabel used for bidirectional path algorithms: Bidirectional A*
 * and CostMatrix (which does not use a heuristic based on distance
 * to the destination).
 */
class BDEdgeLabel : public EdgeLabel {
public:
  // Default constructor
  BDEdgeLabel() {
  }

  /**
   * Constructor with values.
   * @param predecessor         Index into the edge label list for the predecessor
   *                            directed edge in the shortest path.
   * @param edgeid              Directed edge Id.
   * @param oppedgeid           Opposing directed edge Id.
   * @param edge                Directed edge.
   * @param cost                True cost (cost and time in seconds) to the edge.
   * @param sortcost            Cost for sorting (includes A* heuristic)n
   * @param dist                Distance to the destination in meters.
   * @param mode                Mode of travel along this edge.
   * @param tc                  Transition cost entering this edge.
   * @param not_thru_pruning    Is not thru pruning enabled.
   * @param closure_pruning     Is closure pruning active.
   * @param has_measured_speed  Do we have any of the measured speed types set?
   * @param internal_turn       Did we make an turn on a short internal edge.
   * @param restriction_idx     If this label has restrictions, the index where the restriction is
   * found
   * @param path_id             When searching more than one path at a time this denotes which path
   * the this label is tracking
   */
  BDEdgeLabel(const uint32_t predecessor,
              const baldr::GraphId& edgeid,
              const baldr::GraphId& oppedgeid,
              const baldr::DirectedEdge* edge,
              const sif::Cost& cost,
              const float sortcost,
              const float dist,
              const sif::TravelMode mode,
              const sif::Cost& transition_cost,
              const bool not_thru_pruning,
              const bool closure_pruning,
              const bool has_measured_speed,
              const sif::InternalTurn internal_turn,
              const uint8_t restriction_idx,
              const uint8_t path_id = 0)
      : EdgeLabel(predecessor,
                  edgeid,
                  edge,
                  cost,
                  sortcost,
                  dist,
                  mode,
                  0,
                  transition_cost,
                  restriction_idx,
                  closure_pruning,
                  has_measured_speed,
                  internal_turn,
                  path_id),
        opp_edgeid_(oppedgeid), not_thru_pruning_(not_thru_pruning) {
  }

  /**
   * Constructor with values. Sets sortcost to the true cost.
   * Used with CostMatrix (no heuristic/distance to destination).
   * @param predecessor         Index into the edge label list for the predecessor
   *                            directed edge in the shortest path.
   * @param edgeid              Directed edge.
   * @param oppedgeid           Opposing directed edge Id.
   * @param edge                End node of the directed edge.
   * @param cost                True cost (cost and time in seconds) to the edge.
   * @param mode                Mode of travel along this edge.
   * @param tc                  Transition cost entering this edge.
   * @param path_distance       Accumulated path distance.
   * @param not_thru_pruning    Is not thru pruning enabled.
   * @param closure_pruning     Is closure pruning active.
   * @param has_measured_speed  Do we have any of the measured speed types set?
   * @param internal_turn       Did we make an turn on a short internal edge.
   * @param restriction_idx     If this label has restrictions, the index where the restriction is
   * found
   * @param path_id             When searching more than one path at a time this denotes which path
   * the this label is tracking
   */
  BDEdgeLabel(const uint32_t predecessor,
              const baldr::GraphId& edgeid,
              const baldr::GraphId& oppedgeid,
              const baldr::DirectedEdge* edge,
              const sif::Cost& cost,
              const sif::TravelMode mode,
              const sif::Cost& transition_cost,
              const uint32_t path_distance,
              const bool not_thru_pruning,
              const bool closure_pruning,
              const bool has_measured_speed,
              const sif::InternalTurn internal_turn,
              const uint8_t restriction_idx,
              const uint8_t path_id = 0)
      : EdgeLabel(predecessor,
                  edgeid,
                  edge,
                  cost,
                  cost.cost,
                  0,
                  mode,
                  path_distance,
                  transition_cost,
                  restriction_idx,
                  closure_pruning,
                  has_measured_speed,
                  internal_turn,
                  path_id),
        opp_edgeid_(oppedgeid), not_thru_pruning_(not_thru_pruning) {
  }

  /**
   * Constructor with values. Used in SetOrigin.
   * @param predecessor         Index into the edge label list for the predecessor
   *                            directed edge in the shortest path.
   * @param edgeid              Directed edge Id.
   * @param edge                Directed edge.
   * @param cost                True cost (cost and time in seconds) to the edge.
   * @param sortcost            Cost for sorting (includes A* heuristic).
   * @param dist                Distance to the destination in meters.
   * @param mode                Mode of travel along this edge.
   * @param restriction_idx     If this label has restrictions, the index where the restriction is
   * found
   * @param closure_pruning     Is closure pruning active.
   * @param has_measured_speed  Do we have any of the measured speed types set?
   * @param internal_turn       Did we make an turn on a short internal edge.
   * @param path_id             When searching more than one path at a time this denotes which path
   * the this label is tracking
   */
  BDEdgeLabel(const uint32_t predecessor,
              const baldr::GraphId& edgeid,
              const baldr::DirectedEdge* edge,
              const sif::Cost& cost,
              const float sortcost,
              const float dist,
              const sif::TravelMode mode,
              const uint8_t restriction_idx,
              const bool closure_pruning,
              const bool has_measured_speed,
              const sif::InternalTurn internal_turn,
              const uint8_t path_id = 0)
      : EdgeLabel(predecessor,
                  edgeid,
                  edge,
                  cost,
                  sortcost,
                  dist,
                  mode,
                  0,
                  Cost{},
                  restriction_idx,
                  closure_pruning,
                  has_measured_speed,
                  internal_turn,
                  path_id),
        not_thru_pruning_(!edge->not_thru()) {
    opp_edgeid_ = {};
  }

  /**
   * Update an existing edge label with new predecessor and cost information.
   * The mode, edge Id, and end node remain the same.
   * @param predecessor      Predecessor directed edge in the shortest path.
   * @param cost             True cost (and elapsed time in seconds) to the edge.
   * @param sortcost         Cost for sorting (includes A* heuristic).
   * @param tc               Transition cost onto the edge.
   * @param restriction_idx  If this label has restrictions, the index where the restriction is found
   */
  void Update(const uint32_t predecessor,
              const sif::Cost& cost,
              const float sortcost,
              const sif::Cost& tc,
              const uint8_t restriction_idx) {
    predecessor_ = predecessor;
    cost_ = cost;
    sortcost_ = sortcost;
    transition_cost_ = tc;
    restriction_idx_ = restriction_idx;
  }

  /**
   * Update an existing edge label with new predecessor and cost information.
   * Update distance as well (used in time distance matrix)
   * @param predecessor      Predecessor directed edge in the shortest path.
   * @param cost             True cost (and elapsed time in seconds) to the edge.
   * @param sortcost         Cost for sorting (includes A* heuristic).
   * @param tc               Transition cost onto the edge.
   * @param path_distance    Accumulated path distance.
   * @param restriction_idx  If this label has restrictions, the index where the restriction is found
   */
  void Update(const uint32_t predecessor,
              const sif::Cost& cost,
              const float sortcost,
              const sif::Cost& tc,
              const uint32_t path_distance,
              const uint8_t restriction_idx) {
    predecessor_ = predecessor;
    cost_ = cost;
    sortcost_ = sortcost;
    transition_cost_ = tc;
    path_distance_ = path_distance;
    restriction_idx_ = restriction_idx;
  }

  /**
   * Get the GraphId of the opposing directed edge.
   * @return  Returns the GraphId of the opposing directed edge.
   */
  baldr::GraphId opp_edgeid() const {
    return baldr::GraphId(opp_edgeid_);
  }

  /**
   * Should not thru pruning be enabled on this path?
   * @return Returns true if not thru pruning should be enabled.
   */
  bool not_thru_pruning() const {
    return not_thru_pruning_;
  }

protected:
  // Graph Id of the opposing edge.
  // not_thru_pruning_: Is not thru pruning enabled?
  uint64_t opp_edgeid_ : 63; // Could be 46 (to provide more spare)
  // TODO: Move not_thru_prunin to the base class - EdgeLabel so that we can
  // consolidate all pruning related properties at 1 place
  uint64_t not_thru_pruning_ : 1;
};

/**
 * EdgeLabel used for multi-modal A* path algorithm.
 */
class MMEdgeLabel : public EdgeLabel {
public:
  // Default constructor
  MMEdgeLabel() {
  }

  /**
   * Constructor with values.  Used for multi-modal path.
   * @param predecessor      Index into the edge label list for the predecessor
   *                         directed edge in the shortest path.
   * @param edgeid           Directed edge.
   * @param edge             End node of the directed edge.
   * @param cost             True cost (cost and time in seconds) to the edge.
   * @param sortcost         Cost for sorting (includes A* heuristic)
   * @param dist             Distance meters to the destination
   * @param mode             Mode of travel along this edge.
   * @param path_distance    Accumulated distance.
   * @param walking_distance Accumulated walking distance. Used to prune the expansion.
   * @param tripid           Trip Id for a transit edge.
   * @param prior_stopid     Prior transit stop Id.
   * @param blockid          Transit trip block Id.
   * @param transit_operator Transit operator - index into an internal map
   * @param has_transit      Does the path to this edge have any transit.
   * @param transition_cost  Transition cost
   * @param restriction_idx  If this label has restrictions, the index where the restriction is found
   * @param path_id          When searching more than one path at a time this denotes which path the
   *                         this label is tracking
   */
  MMEdgeLabel(const uint32_t predecessor,
              const baldr::GraphId& edgeid,
              const baldr::DirectedEdge* edge,
              const sif::Cost& cost,
              const float sortcost,
              const float dist,
              const sif::TravelMode mode,
              const uint32_t path_distance,
              const uint32_t walking_distance,
              const uint32_t tripid,
              const baldr::GraphId& prior_stopid,
              const uint32_t blockid,
              const uint32_t transit_operator,
              const bool has_transit,
              const Cost& transition_cost,
              const uint8_t restriction_idx,
              const uint8_t path_id = 0)
      : EdgeLabel(predecessor,
                  edgeid,
                  edge,
                  cost,
                  sortcost,
                  dist,
                  mode,
                  path_distance,
                  transition_cost,
                  restriction_idx,
                  true,
                  false,
                  InternalTurn::kNoTurn,
                  path_id),
        prior_stopid_(prior_stopid), tripid_(tripid), blockid_(blockid),
        transit_operator_(transit_operator), has_transit_(has_transit),
        walking_distance_(walking_distance) {
  }

  /**
   * Update an existing edge label with new predecessor and cost information.
   * Update transit information: prior stop Id will stay the same but trip Id
   * and block Id may change (a new trip at an earlier departure time).
   * The mode, edge Id, and end node remain the same.
   * @param predecessor      Predecessor directed edge in the shortest path.
   * @param cost             True cost (and elapsed time in seconds) to the edge.
   * @param sortcost         Cost for sorting (includes A* heuristic).
   * @param path_distance    Accumulated path distance.
   * @param walking_distance Accumulated walking distance. Used to prune the expansion.
   * @param tripid           Trip Id for a transit edge.
   * @param blockid          Transit trip block Id.
   * @param transition_cost  Transition cost
   * @param restriction_idx  If this label has restrictions, the index where the restriction is found
   */
  void Update(const uint32_t predecessor,
              const sif::Cost& cost,
              const float sortcost,
              const uint32_t path_distance,
              const uint32_t walking_distance,
              const uint32_t tripid,
              const uint32_t blockid,
              const Cost& transition_cost,
              const uint8_t restriction_idx) {
    predecessor_ = predecessor;
    cost_ = cost;
    sortcost_ = sortcost;
    path_distance_ = path_distance;
    walking_distance_ = walking_distance;
    tripid_ = tripid;
    blockid_ = blockid;
    transition_cost_ = transition_cost;
    restriction_idx_ = restriction_idx;
  }

  /**
   * Get the prior transit stop Id.
   * @return  Returns the prior transit stop Id.
   */
  const baldr::GraphId& prior_stopid() const {
    return prior_stopid_;
  }

  /**
   * Get the transit trip Id.
   * @return   Returns the transit trip Id of the prior edge.
   */
  uint32_t tripid() const {
    return tripid_;
  }

  /**
   * Return the transit block Id of the prior trip.
   * @return  Returns the block Id.
   */
  uint32_t blockid() const {
    return blockid_;
  }

  /**
   * Get the index of the transit operator.
   * @return  Returns the transit operator index (0 if none).
   */
  uint32_t transit_operator() const {
    return transit_operator_;
  }

  /**
   * Has any transit been taken up to this point on the path.
   * @return  Returns true if any transit has been taken, false if not.
   */
  bool has_transit() const {
    return has_transit_;
  }

  /**
   * Return the current walking distance in meters.
   */
  uint32_t walking_distance() const {
    return walking_distance_;
  }

protected:
  // GraphId of the predecessor transit stop.
  baldr::GraphId prior_stopid_;

  // tripid_: Transit trip Id.
  uint32_t tripid_;

  // blockid_:          Block Id (0 indicates no prior).
  // transit_operator_: Prior operator. Index to an internal mapping).
  // has_transit_:      True if any transit taken along the path to this edge.
  uint32_t blockid_ : 21; // Really only needs 20 bits
  uint32_t transit_operator_ : 10;
  uint32_t has_transit_ : 1;

  // Accumulated walking distance to prune the expansion
  uint32_t walking_distance_;
};

} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_EDGELABEL_H_
