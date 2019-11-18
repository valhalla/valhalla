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
  EdgeLabel() {
  }

  /**
   * Constructor with values.
   * @param predecessor   Index into the edge label list for the predecessor
   *                      directed edge in the shortest path.
   * @param edgeid        Directed edge Id.
   * @param edge          Directed edge.
   * @param cost          True cost (cost and time in seconds) to the edge.
   * @param sortcost      Cost for sorting (includes A* heuristic)
   * @param dist          Distance to the destination (meters)
   * @param mode          Mode of travel along this edge.
   * @param path_distance Accumulated path distance
   */
  EdgeLabel(const uint32_t predecessor,
            const baldr::GraphId& edgeid,
            const baldr::DirectedEdge* edge,
            const Cost& cost,
            const float sortcost,
            const float dist,
            const TravelMode mode,
            const uint32_t path_distance,
            bool has_time_restrictions = false)
      : predecessor_(predecessor), path_distance_(path_distance), restrictions_(edge->restrictions()),
        edgeid_(edgeid), opp_index_(edge->opp_index()), opp_local_idx_(edge->opp_local_idx()),
        mode_(static_cast<uint32_t>(mode)), endnode_(edge->endnode()),
        has_time_restrictions_(has_time_restrictions), use_(static_cast<uint32_t>(edge->use())),
        classification_(static_cast<uint32_t>(edge->classification())), shortcut_(edge->shortcut()),
        dest_only_(edge->destonly()), origin_(0), toll_(edge->toll()), not_thru_(edge->not_thru()),
        deadend_(edge->deadend()), on_complex_rest_(edge->part_of_complex_restriction()), cost_(cost),
        sortcost_(sortcost), distance_(dist) {
  }

  /**
   * Update an existing edge label with new predecessor and cost information.
   * The mode, edge Id, and end node remain the same.
   * @param predecessor Predecessor directed edge in the shortest path.
   * @param cost        True cost (and elapsed time in seconds) to the edge.
   * @param sortcost    Cost for sorting (includes A* heuristic).
   */
  void Update(const uint32_t predecessor,
              const Cost& cost,
              const float sortcost,
              const bool has_time_restrictions) {
    predecessor_ = predecessor;
    cost_ = cost;
    sortcost_ = sortcost;
    has_time_restrictions_ = has_time_restrictions;
  }

  /**
   * Update an existing edge label with new predecessor and cost information.
   * Update transit information: prior stop Id will stay the same but trip Id
   * and block Id may change (a new trip at an earlier departure time).
   * The mode, edge Id, and end node remain the same.
   * @param predecessor    Predecessor directed edge in the shortest path.
   * @param cost           True cost (and elapsed time in seconds) to the edge.
   * @param sortcost       Cost for sorting (includes A* heuristic).
   * @param path_distance  Accumulated path distance.
   */
  void Update(const uint32_t predecessor,
              const Cost& cost,
              const float sortcost,
              const uint32_t path_distance,
              const bool has_time_restrictions) {
    predecessor_ = predecessor;
    cost_ = cost;
    sortcost_ = sortcost;
    path_distance_ = path_distance;
    has_time_restrictions_ = has_time_restrictions;
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
   * Does this edge have any time restrictions?
   */
  bool has_time_restriction() const {
    return has_time_restrictions_;
  }
  /**
   * Sets whether this edge has any time restrictions or not
   */
  void set_has_time_restriction(bool has_time_restrictions) {
    has_time_restrictions_ = has_time_restrictions;
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
   * edgeid_:         Graph Id of the edge.
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
   * endnode_:        GraphId of the end node of the edge. This allows the
   *                  expansion to occur by reading the node and not having
   *                  to re-read the directed edge.
   * use_:            Use of the prior edge.
   * classification_: Road classification
   * shortcut_:       Was the prior edge a shortcut edge?
   * dest_only_:      Was the prior edge destination only?
   * origin_:         True if this is an origin edge.
   * toll_"           Edge is toll.
   * not_thru_:       Flag indicating edge is not_thru.
   * deadend_:        Flag indicating edge is a dead-end.
   * on_complex_rest: Part of a complex restriction.
   */
  uint64_t endnode_ : 46;
  uint64_t spare_ : 1; // Unused  bit
  uint64_t has_time_restrictions_ : 1;
  uint64_t use_ : 6;
  uint64_t classification_ : 3;
  uint64_t shortcut_ : 1;
  uint64_t dest_only_ : 1;
  uint64_t origin_ : 1;
  uint64_t toll_ : 1;
  uint64_t not_thru_ : 1;
  uint64_t deadend_ : 1;
  uint64_t on_complex_rest_ : 1;

  Cost cost_;      // Cost and elapsed time along the path.
  float sortcost_; // Sort cost - includes A* heuristic.
  float distance_; // Distance to the destination.
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
   * @param predecessor  Index into the edge label list for the predecessor
   *                     directed edge in the shortest path.
   * @param edgeid       Directed edge Id.
   * @param oppedgeid    Opposing directed edge Id.
   * @param edge         Directed edge.
   * @param cost         True cost (cost and time in seconds) to the edge.
   * @param sortcost     Cost for sorting (includes A* heuristic)n
   * @param dist         Distance to the destination in meters.
   * @param mode         Mode of travel along this edge.
   * @param tc           Transition cost entering this edge.
   * @param not_thru_pruning  Is not thru pruning enabled.
   */
  BDEdgeLabel(const uint32_t predecessor,
              const baldr::GraphId& edgeid,
              const baldr::GraphId& oppedgeid,
              const baldr::DirectedEdge* edge,
              const sif::Cost& cost,
              const float sortcost,
              const float dist,
              const sif::TravelMode mode,
              const sif::Cost& tc,
              const bool not_thru_pruning,
              const bool has_time_restrictions)
      : EdgeLabel(predecessor, edgeid, edge, cost, sortcost, dist, mode, 0, has_time_restrictions),
        opp_edgeid_(oppedgeid), not_thru_pruning_(not_thru_pruning), transition_cost_(tc) {
  }

  /**
   * Constructor with values. Sets sortcost to the true cost.
   * Used with CostMatrix (no heuristic/distance to destination).
   * @param predecessor  Index into the edge label list for the predecessor
   *                       directed edge in the shortest path.
   * @param edgeid        Directed edge.
   * @param oppedgeid     Opposing directed edge Id.
   * @param edge          End node of the directed edge.
   * @param cost          True cost (cost and time in seconds) to the edge.
   * @param mode          Mode of travel along this edge.
   * @param tc            Transition cost entering this edge.
   * @param path_distance Accumulated path distance.
   * @param not_thru_pruning  Is not thru pruning enabled.
   */
  BDEdgeLabel(const uint32_t predecessor,
              const baldr::GraphId& edgeid,
              const baldr::GraphId& oppedgeid,
              const baldr::DirectedEdge* edge,
              const sif::Cost& cost,
              const sif::TravelMode mode,
              const sif::Cost& tc,
              const uint32_t path_distance,
              const bool not_thru_pruning,
              const bool has_time_restrictions)
      : EdgeLabel(predecessor,
                  edgeid,
                  edge,
                  cost,
                  cost.cost,
                  0,
                  mode,
                  path_distance,
                  has_time_restrictions),
        opp_edgeid_(oppedgeid), not_thru_pruning_(not_thru_pruning), transition_cost_(tc) {
  }

  /**
   * Constructor with values.
   * @param predecessor  Index into the edge label list for the predecessor
   *                     directed edge in the shortest path.
   * @param edgeid       Directed edge Id.
   * @param edge         Directed edge.
   * @param cost         True cost (cost and time in seconds) to the edge.
   * @param sortcost     Cost for sorting (includes A* heuristic).
   * @param dist         Distance to the destination in meters.
   * @param mode         Mode of travel along this edge.
   */
  BDEdgeLabel(const uint32_t predecessor,
              const baldr::GraphId& edgeid,
              const baldr::DirectedEdge* edge,
              const sif::Cost& cost,
              const float sortcost,
              const float dist,
              const sif::TravelMode mode,
              const bool has_time_restrictions)
      : EdgeLabel(predecessor, edgeid, edge, cost, sortcost, dist, mode, 0, has_time_restrictions),
        not_thru_pruning_(false) {
    opp_edgeid_ = {};
    transition_cost_ = {};
  }

  /**
   * Update an existing edge label with new predecessor and cost information.
   * The mode, edge Id, and end node remain the same.
   * @param predecessor Predecessor directed edge in the shortest path.
   * @param cost        True cost (and elapsed time in seconds) to the edge.
   * @param sortcost    Cost for sorting (includes A* heuristic).
   * @param tc          Transition cost onto the edge.
   */
  void Update(const uint32_t predecessor,
              const sif::Cost& cost,
              const float sortcost,
              const sif::Cost& tc,
              const bool has_time_restrictions) {
    predecessor_ = predecessor;
    cost_ = cost;
    sortcost_ = sortcost;
    transition_cost_ = tc;
    has_time_restrictions_ = has_time_restrictions;
  }

  /**
   * Update an existing edge label with new predecessor and cost information.
   * Update distance as well (used in time distance matrix)
   * @param predecessor   Predecessor directed edge in the shortest path.
   * @param cost          True cost (and elapsed time in seconds) to the edge.
   * @param sortcost      Cost for sorting (includes A* heuristic).
   * @param tc            Transition cost onto the edge.
   * @param path_distance  Accumulated path distance.
   */
  void Update(const uint32_t predecessor,
              const sif::Cost& cost,
              const float sortcost,
              const sif::Cost& tc,
              const uint32_t path_distance,
              const bool has_time_restrictions) {
    predecessor_ = predecessor;
    cost_ = cost;
    sortcost_ = sortcost;
    transition_cost_ = tc;
    path_distance_ = path_distance;
    has_time_restrictions_ = has_time_restrictions;
  }

  /**
   * Get the GraphId of the opposing directed edge.
   * @return  Returns the GraphId of the opposing directed edge.
   */
  baldr::GraphId opp_edgeid() const {
    return baldr::GraphId(opp_edgeid_);
  }

  /**
   * Get the transition cost in seconds. This is used in the bidirectional A*
   * to determine the cost at the connection.
   * @return  Returns the transition cost (including penalties) in seconds.
   */
  float transition_cost() const {
    return transition_cost_.cost;
  }

  /**
   * Get the transition cost in seconds. This is used in the bidirectional A*
   * reverse path search to allow the recovery of the true elapsed time along
   * the path. This is needed since the transition cost is applied at a
   * different node than the forward search.
   * @return  Returns the transition cost (without penalties) in seconds.
   */
  float transition_secs() const {
    return transition_cost_.secs;
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
  uint64_t not_thru_pruning_ : 1;

  // Transition cost (for recovering elapsed time on reverse path)
  sif::Cost transition_cost_;
};

/**
 * EdgeLabel used for multi-modal A* path algorithm.
 */
class MMEdgeLabel : public EdgeLabel {
public:
  /**
   * Constructor with values.  Used for multi-modal path.
   * @param predecessor   Index into the edge label list for the predecessor
   *                      directed edge in the shortest path.
   * @param edgeid        Directed edge.
   * @param edge          End node of the directed edge.
   * @param cost          True cost (cost and time in seconds) to the edge.
   * @param sortcost      Cost for sorting (includes A* heuristic)
   * @param dist          Distance meters to the destination
   * @param mode          Mode of travel along this edge.
   * @param path_distance Accumulated distance.
   * @param tripid        Trip Id for a transit edge.
   * @param prior_stopid  Prior transit stop Id.
   * @param blockid       Transit trip block Id.
   * @param transit_operator Transit operator - index into an internal map
   * @param has_transit   Does the path to this edge have any transit.
   */
  MMEdgeLabel(const uint32_t predecessor,
              const baldr::GraphId& edgeid,
              const baldr::DirectedEdge* edge,
              const sif::Cost& cost,
              const float sortcost,
              const float dist,
              const sif::TravelMode mode,
              const uint32_t path_distance,
              const uint32_t tripid,
              const baldr::GraphId& prior_stopid,
              const uint32_t blockid,
              const uint32_t transit_operator,
              const bool has_transit,
              const bool has_time_restrictions = false)
      : EdgeLabel(predecessor,
                  edgeid,
                  edge,
                  cost,
                  sortcost,
                  dist,
                  mode,
                  path_distance,
                  has_time_restrictions),
        prior_stopid_(prior_stopid), tripid_(tripid), blockid_(blockid),
        transit_operator_(transit_operator), has_transit_(has_transit) {
  }

  /**
   * Update an existing edge label with new predecessor and cost information.
   * Update transit information: prior stop Id will stay the same but trip Id
   * and block Id may change (a new trip at an earlier departure time).
   * The mode, edge Id, and end node remain the same.
   * @param predecessor    Predecessor directed edge in the shortest path.
   * @param cost           True cost (and elapsed time in seconds) to the edge.
   * @param sortcost       Cost for sorting (includes A* heuristic).
   * @param path_distance  Accumulated path distance.
   * @param tripid         Trip Id for a transit edge.
   * @param blockid        Transit trip block Id.
   */
  void Update(const uint32_t predecessor,
              const sif::Cost& cost,
              const float sortcost,
              const uint32_t path_distance,
              const uint32_t tripid,
              const uint32_t blockid,
              const bool has_time_restrictions) {
    predecessor_ = predecessor;
    cost_ = cost;
    sortcost_ = sortcost;
    path_distance_ = path_distance;
    tripid_ = tripid;
    blockid_ = blockid;
    has_time_restrictions_ = has_time_restrictions;
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
};

} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_EDGELABEL_H_
