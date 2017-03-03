#ifndef VALHALLA_SIF_EDGELABEL_H_
#define VALHALLA_SIF_EDGELABEL_H_

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/sif/costconstants.h>
#include <limits>

namespace valhalla {
namespace sif {

/**
 * Labeling information for shortest path algorithm. Contains cost,
 * predecessor, current time, and assorted information required during
 * construction of the shortest path (stored in AdjacencyList) and for
 * reconstructing the path upon completion.
 */
class EdgeLabel {
 public:
  /**
   * Default constructor.
   */
  EdgeLabel();

  /**
   * Constructor with values.
   * @param predecessor   Index into the edge label list for the predecessor
   *                      directed edge in the shortest path.
   * @param edgeid        Directed edge Id.
   * @param edge          Directed edge.
   * @param cost          True cost (cost and time in seconds) to the edge.
   * @param sortcost      Cost for sorting (includes A* heuristic)
   * @param dist          Distance meters to the destination
   * @param mode          Mode of travel along this edge.
   * @param path_distance Accumulated path distance
   */
  EdgeLabel(const uint32_t predecessor, const baldr::GraphId& edgeid,
            const baldr::DirectedEdge* edge, const Cost& cost,
            const float sortcost, const float dist,
            const TravelMode mode, const uint32_t path_distance);

  /**
   * Constructor with values - used in bidirectional A*.
   * @param predecessor  Index into the edge label list for the predecessor
   *                     directed edge in the shortest path.
   * @param edgeid       Directed edge.
   * @param oppedgeid    Opposing directed edge Id.
   * @param endnode      End node of the directed edge.
   * @param cost         True cost (cost and time in seconds) to the edge.
   * @param sortcost     Cost for sorting (includes A* heuristic)
   * @param dist         Distance meters to the destination
   * @param mode         Mode of travel along this edge.
   * @param tc           Transition cost entering this edge.
   * @param not_thru_pruning  Is not thru pruning enabled.
   */
  EdgeLabel(const uint32_t predecessor, const baldr::GraphId& edgeid,
            const baldr::GraphId& oppedgeid,
            const baldr::DirectedEdge* edge, const Cost& cost,
            const float sortcost, const float dist,
            const TravelMode mode, const Cost& tc,
            bool not_thru_pruning);

  /**
   * Constructor with values.  Used for multi-modal path.
   * @param predecessor   Index into the edge label list for the predecessor
   *                      directed edge in the shortest path.
   * @param edgeid        Directed edge.
   * @param endnode       End node of the directed edge.
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
  EdgeLabel(const uint32_t predecessor, const baldr::GraphId& edgeid,
            const baldr::DirectedEdge* edge, const Cost& cost,
            const float sortcost, const float dist,
            const TravelMode mode, const uint32_t path_distance,
            const uint32_t tripid, const baldr::GraphId& prior_stopid,
            const uint32_t blockid, const uint32_t transit_operator,
            const bool has_transit);

  /**
   * Constructor with values - used in time distance matrix (needs the
   * accumulated distance as well as opposing edge information). Sets
   * sortcost to the true cost.
   * @param predecessor  Index into the edge label list for the predecessor
   *                       directed edge in the shortest path.
   * @param edgeid        Directed edge.
   * @param oppedgeid     Opposing directed edge Id.
   * @param endnode       End node of the directed edge.
   * @param cost          True cost (cost and time in seconds) to the edge.
   * @param mode          Mode of travel along this edge.
   * @param tc            Transition cost entering this edge.
   * @param path_distance Accumulated path distance.
   * @param not_thru_pruning  Is not thru pruning enabled.
   */
  EdgeLabel(const uint32_t predecessor, const baldr::GraphId& edgeid,
            const baldr::GraphId& oppedgeid,
            const baldr::DirectedEdge* edge, const Cost& cost,
            const TravelMode mode, const Cost& tc,
            const uint32_t path_distance, bool not_thru_pruning);

  /**
   * Constructor given a predecessor edge label. This is used for hierarchy
   * transitions where the attributes at the predecessor are needed (rather
   * than attributes from the directed edge).
   * @param predecessor  Index into the edge label list for the predecessor
   *                     directed edge in the shortest path.
   * @param edgeid       Directed edge id.
   * @param endnode      End node of the transition edge.
   * @param pred         Predecessor edge label (to copy attributes from)
   */
  EdgeLabel(const uint32_t predecessor, const baldr::GraphId& edgeid,
            const baldr::GraphId& endnode, const EdgeLabel& pred);

  /**
   * Update an existing edge label with new predecessor and cost information.
   * The mode, edge Id, and end node remain the same.
   * @param predecessor Predecessor directed edge in the shortest path.
   * @param cost        True cost (and elapsed time in seconds) to the edge.
   * @param sortcost    Cost for sorting (includes A* heuristic).
   */
  void Update(const uint32_t predecessor, const Cost& cost,
              const float sortcost);

  /**
   * Update an existing edge label with new predecessor and cost information.
   * Update distance as well (used in time distance matrix)
   * @param predecessor   Predecessor directed edge in the shortest path.
   * @param cost          True cost (and elapsed time in seconds) to the edge.
   * @param sortcost      Cost for sorting (includes A* heuristic).
   * @param tc            Transition cost onto the edge.
   * @param path_distance  Accumulated path distance.
   */
  void Update(const uint32_t predecessor, const Cost& cost,
              const float sortcost, const Cost& tc,
              const uint32_t path_distance);

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
  void Update(const uint32_t predecessor, const Cost& cost,
              const float sortcost, const uint32_t path_distance,
              const uint32_t tripid, const uint32_t blockid);

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
  const baldr::GraphId& edgeid() const {
    return edgeid_;
  }

  /**
   * Get the GraphId of the opposing directed edge.
   * @return  Returns the GraphId of the opposing directed edge.
   */
  const baldr::GraphId& opp_edgeid() const {
    return opp_edgeid_;
  }

  /**
   * Get the prior transit stop Id.
   * @return  Returns the prior transit stop Id.
   */
  const baldr::GraphId& prior_stopid() const {
    return opp_edgeid_;
  }

  /**
   * Get the end node of this directed edge. Allows the A* algorithm
   * to expand the search from this end node, without having to read
   * the directed edge again.
   * @return  Returns the GraphId of the end node of this directed edge.
   */
  const baldr::GraphId& endnode() const {
    return endnode_;
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
   * Has any transit been taken up to this point on the path.
   * @return  Returns true if any transit has been taken, false if not.
   */
  bool has_transit() const {
    return has_transit_;
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
   * Should not thru pruning be enabled on this path?
   * @return Returns true if not thru pruning should be enabled.
   */
  bool not_thru_pruning() const {
    return not_thru_pruning_;
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
   * Operator < used for sorting.
   */
  bool operator < (const EdgeLabel& other) const;

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
   * Set the transition cost.
   * @param  tc  Transition cost.
   */
  void set_transition_cost(const Cost& tc) {
    transition_cost_ = tc;
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

 private:
  // Graph Id of the edge.
  baldr::GraphId edgeid_;

  // GraphId of the end node of the edge. This allows the expansion to occur
  // by reading the node and not having to re-read the directed edge.
  baldr::GraphId endnode_;

  // Graph Id of the opposing edge (for bidirectional A*). Stores the
  // predecessor transit stop graph Id for multi-modal routes.
  baldr::GraphId opp_edgeid_;

  // Cost and elapsed time along the path.
  Cost cost_;

  // Transition cost (for recovering elapsed time on reverse path)
  Cost transition_cost_;

  // Sort cost - includes A* heuristic.
  float sortcost_;

  // Distance to the destination.
  float distance_;

  /**
   * Attributes to carry along with the edge label.
   * path_distance_: Accumulated path distance in meters.
   * use_:           Use of the prior edge.
   * opp_index_:     Index at the end node of the opposing directed edge.
   * opp_local_idx_: Index at the end node of the opposing local edge. This
   *                 value can be compared to the directed edge local_edge_idx
   *                 for edge transition costing and Uturn detection.
   * restriction_:   Bit mask of edges (by local edge index at the end node)
   *                 that are restricted (simple turn restrictions)
   * shortcut_:      Was the prior edge a shortcut edge?
   * mode_:          Current transport mode.
   * dest_only_:     Was the prior edge destination only?
   * has_transit_:   True if any transit taken along the path to this edge.
   * origin_:        True if this is an origin edge.
   * toll_"          Edge is toll.
   * not_thru_:      Flag indicating edge is not_thru.
   * deadend_:       Flag indicating edge is a dead-end.
   */
  uint64_t path_distance_    : 26;
  uint64_t use_              : 6;
  uint64_t opp_index_        : 7;
  uint64_t opp_local_idx_    : 7;
  uint64_t restrictions_     : 7;
  uint64_t shortcut_         : 1;
  uint64_t mode_             : 4;
  uint64_t dest_only_        : 1;
  uint64_t has_transit_      : 1;
  uint64_t origin_           : 1;
  uint64_t toll_             : 1;
  uint32_t not_thru_         : 1;
  uint32_t deadend_          : 1;

  // predecessor_: Index to the predecessor edge label information.
  // Note: invalid predecessor value uses all 32 bits (so if this needs to
  // be part of a bit field make sure kInvalidLabel is changed.
  uint32_t predecessor_;

  // tripid_:           Transit trip Id.
  // classification_:   Road classification
  // not_thru_pruning_: Is not thru pruning enabled?
  uint32_t tripid_           : 28;
  uint32_t classification_   : 3;
  uint32_t not_thru_pruning_ : 1;

  // Block Id and prior operator (index to an internal mapping).
  //          0 indicates no prior.
  // on_complex_rest_: Edge is part of a complex restriction.
  uint32_t blockid_          : 21; // Really only needs 20 bits
  uint32_t transit_operator_ : 10;
  uint32_t on_complex_rest_  : 1;
};

}
}

#endif  // VALHALLA_SIF_EDGELABEL_H_
