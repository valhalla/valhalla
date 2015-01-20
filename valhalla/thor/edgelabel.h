#ifndef VALHALLA_THOR_EDGELABEL_H_
#define VALHALLA_THOR_EDGELABEL_H_

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/directededge.h>

namespace valhalla {
namespace thor {

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
   * @param cost      True cost to the edge.
   * @param sortcost  Cost for sorting (includes A* heuristic)
   * @param dist      Distance meters to the destination
   */
  EdgeLabel(const uint32_t predecessor, const baldr::GraphId& edgeid,
            const baldr::DirectedEdge* edge, const float cost,
            const float sortcost, const float dist);

  /**
   * Destructor.
   */
  virtual ~EdgeLabel();

  /**
   * Update an existing edge label with new predecessor and cost information.
   * The edge Id and end node remain the same.
   * @param predecessor Predecessor directeded edge in the shortest path.
   * @param cost      True cost to the edge.
   * @param sortcost  Cost for sorting (includes A* heuristic)
   */
  void Update(const uint32_t predecessor, const float cost,
            const float sortcost);

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
   * @return  Returns the cost (units are based on the costing method).
   */
  float truecost() const;

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
   * Get the index that represents a Uturn.
   * @return  Returns the Uturn index.
   */
  uint32_t uturn_index() const;

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
   * Operator < used for sorting.
   */
  bool operator < (const EdgeLabel& other) const;

 private:
  // Index to the predecessor edge label information.
  uint32_t predecessor_;

  // Graph Id of the edge.
  baldr::GraphId edgeid_;

  // GraphId of the end node of the edge. This allows the expansion
  // to occur by reading the node and not having to re-read the directed
  // edge
  baldr::GraphId endnode_;

  // Cost (time, distance, etc.)
  float truecost_;

  // Sort cost - includes A* heuristic
  float sortcost_;

  // Distance to the destination
  float distance_;

  /**
   * Attributes to carry along with the edge label.
   * uturn_index: Index at the end node of the edge that constitutes a U-turn
   * trans_up:    Was the prior edge a transition up to a higher level
   * trans_down:  Was the prior edge a transition down to a lower level
   */
  struct Attributes {
    uint32_t uturn_index  : 5;
    uint32_t trans_up     : 1;
    uint32_t trans_down   : 1;
    uint32_t spare        : 25;
  };
  Attributes attributes_;

  // Transit information...trip ID , prior stop ID, time at stop? This
  // could be be part of a derived class used only in multimodal routes?
};

}
}

#endif  // VALHALLA_THOR_EDGELABEL_H_
