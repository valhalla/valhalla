#ifndef VALHALLA_SIF_EDGELABEL_H_
#define VALHALLA_SIF_EDGELABEL_H_

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/sif/costconstants.h>
#include <limits>

namespace valhalla {
namespace sif {

constexpr uint32_t kInvalidLabel = std::numeric_limits<uint32_t>::max();

/**
 * Labeling information for shortest path algorithm. Contains cost,
 * predecessor, current time, and assorted information required during
 * construction of the shortest path (stored in AdjacencyList) and for
 * reconstructing the path upon completion (the EdgeLabel list forms
 * the list of permanently labeled elements - aka the Done set).
 */
class EdgeLabel {
 public:
  /**
   * Default constructor.
   */
  EdgeLabel();

  /**
   * Constructor with values.
   * @param predecessor Index into the edge label list for the predecessor
   *                       directed edge in the shortest path.
   * @param edgeid    Directed edge.
   * @param endnode   End node of the directed edge.
   * @param cost      True cost (cost and elapsed time in seconds) to the edge.
   * @param sortcost  Cost for sorting (includes A* heuristic)
   * @param dist      Distance meters to the destination
   * @param restrictions Restriction mask from prior edge - this is used
   *                  if edge is a transition edge. This allows restrictions
   *                  to be carried across different hierarchy levels.
   * @param mode      Mode of travel along this edge.
   */
  EdgeLabel(const uint32_t predecessor, const baldr::GraphId& edgeid,
            const baldr::DirectedEdge* edge, const Cost& cost,
            const float sortcost, const float dist,
            const uint32_t restrictions, const uint32_t opp_local_idx,
            const TravelMode mode);

  /**
   * Destructor.
   */
  virtual ~EdgeLabel();

  /**
   * Update an existing edge label with new predecessor and cost information.
   * The edge Id and end node remain the same.
   * @param predecessor Predecessor directed edge in the shortest path.
   * @param cost        True cost (and elapsed time in seconds) to the edge.
   * @param sortcost    Cost for sorting (includes A* heuristic)
   * @param mode        Mode of travel along the edge
   */
  void Update(const uint32_t predecessor, const Cost& cost,
            const float sortcost, const TravelMode mode);

  /**
   * Get the predecessor edge label.
   * @return Predecessor edge label.
   */
  uint32_t predecessor() const;

  /**
   * Get the GraphId of this directed edge.
   * @return  Returns the GraphId of this directed edge.
   */
  const baldr::GraphId& edgeid() const;

  /**
   * Get the end node of this directed edge. Allows the A* algorithm
   * to expand the search from this end node, without having to read
   * the directed edge again.
   * @return  Returns the GraphId of the end node of this directed edge.
   */
  const baldr::GraphId& endnode() const;

  /**
   * Get the cost from the origin to this directed edge.
   * @return  Returns the cost (units are based on the costing method)
   *          and elapsed time (seconds) to the end of the directed edge.
   */
  const Cost& cost() const;

  /**
   * Get the sort cost from the origin to this directed edge. The sort
   * cost includes the A* heuristic.
   * @return  Returns the sort cost (units are based on the costing method).
   */
  float sortcost() const;

  /**
   * Set the sort cost from the origin to this directed edge. The sort
   * cost includes the A* heuristic.
   * @param sortcost Sort cost (units are based on the costing method).
   */
  void SetSortCost(float sortcost);

  /**
   * Get the distance to the destination.
   * @return  Returns the distance in meters.
   */
  float distance() const;

  /**
   * Get the opposing local index. This is the index of the incoming edge
   * (on the local hierarchy) at the end node of the predecessor directed
   * edge. This is used for edge transition costs and Uturn detection.
   * @return  Returns the local index of the incoming edge.
   */
  uint32_t opp_local_idx() const;

  /**
   * Get the restriction mask at the end node. Each bit set to 1 indicates a
   * turn restriction onto the directed edge with matching local edge index.
   * @return  Returns the restriction mask.
   */
  uint32_t restrictions() const;

  /**
   * Get the transition up flag.
   * @return  Returns true if the prior edge was a transition up, false if not.
   */
  bool trans_up() const;

  /**
   * Get the transition down flag.
   * @return  Returns true if the prior edge was a transition down,
   *          false if not.
   */
  bool trans_down() const;

  /**
   * Get the shortcut flag.
   * @return  Returns true if the prior edge was a shortcut, false if not.
   */
  bool shortcut() const;

  /**
   * Get the travel mode along this edge.
   * @return  Returns the travel mode.
   */
  TravelMode mode() const;

  /**
   * Operator < used for sorting.
   */
  bool operator < (const EdgeLabel& other) const;

 private:
  // Index to the predecessor edge label information.
  uint32_t predecessor_;

  // Graph Id of the edge.
  baldr::GraphId edgeid_;

  // Elapsed time
  uint32_t elapsedtime_;

  // GraphId of the end node of the edge. This allows the expansion
  // to occur by reading the node and not having to re-read the directed
  // edge
  baldr::GraphId endnode_;

  // Cost and elapsed time along the path
  Cost cost_;

  // Sort cost - includes A* heuristic
  float sortcost_;

  // Distance to the destination
  float distance_;

  /**
   * Attributes to carry along with the edge label.
   * opp_local_idx: Index at the end node of the opposing local edge. This
   *                value can be compared to the directed edge local_edge_idx
   *                for edge transition costing and Uturn detection.
   * trans_up:      Was the prior edge a transition up to a higher level?
   * trans_down:    Was the prior edge a transition down to a lower level?
   * shortcut:      Was the prior edge a shortcut edge?
   * restrictions:  Bit mask of edges (by local edge index at the end node)
   *                that are restricted (simple turn restrictions)

   */
  struct Attributes {
    uint32_t opp_local_idx : 7;
    uint32_t restrictions  : 7;
    uint32_t trans_up      : 1;
    uint32_t trans_down    : 1;
    uint32_t shortcut      : 1;
    uint32_t mode          : 4;
    uint32_t spare         : 11;
  };
  Attributes attributes_;

  // Transit information...trip ID , prior stop ID, time at stop? This
  // could be be part of a derived class used only in multimodal routes?
};

}
}

#endif  // VALHALLA_SIF_EDGELABEL_H_
