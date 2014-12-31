#ifndef VALHALLA_THOR_EDGELABEL_H_
#define VALHALLA_THOR_EDGELABEL_H_

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/directededge.h>

namespace valhalla{
namespace thor{

/**
 * Labeling information for shortest path algorithm. Contains cost,
 * predecessor, current time, and assorted information required during
 * construction of the shortest path (stored in AdjacencyList) and for
 * reconstructing the path upon completion (from the list of permanently
 * labeled elements).
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
   * @param edgeid    GraphId of this directed edge.
   * @param endnode   End node of the directed edge.
   * @param cost      True cost to the edge.
   * @param sortcost  Cost for sorting (includes A* heuristic)
   * @param uturn     Index at the end node of the directed edge that
   *                  represents at u-turn
   */
  EdgeLabel(const uint32_t predecessor, const baldr::GraphId& edgeid,
            const baldr::GraphId& endnode, const float cost,
            const float sortcost, const uint32_t uturn);

  /**
   * Destructor.
   */
  virtual ~EdgeLabel();

  /**
   * Set values.
   * @param predecessor Index into the edge label list for the predecessor
   *                       directed edge in the shortest path.
   * @param edgeid    GraphId of this directed edge.
   * @param endnode   End node of the directed edge.
   * @param cost      True cost to the edge.
   * @param sortcost  Cost for sorting (includes A* heuristic)
   * @param uturn     Index at the end node of the directed edge that
   *                  represents at u-turn
   */
  void Set(const uint32_t predecessor, const baldr::GraphId& edgeid,
           const baldr::GraphId& endnode, const float cost,
           const float sortcost, const uint32_t uturn);

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
   * Set the predecessor edge label.
   * @param  predecessor  Index of the predecessor edge label.
   */
  void SetPredecessor(const uint32_t predecessor);

  /**
   * Get the GraphId of this directed edge.
   * @return  Returns the GraphId of this directed edge.
   */
  const baldr::GraphId& edgeid() const;

  /**
   * Set the GraphId of this directed edge.
   * @param  edgeid  GraphId of this directed edge.
   */
  void SetEdgeId(const baldr::GraphId& edgeid);

  /**
   * Get the end node of this directed edge. Allows the A* algorithm
   * to expand the search from this end node, without having to read
   * the directed edge again.
   * @return  Returns the GraphId of the end node of this directed edge.
   */
  const baldr::GraphId& endnode() const;

  /**
   * Set the end node of this directed edge. Allows the A* algorithm
   * to expand the search from this end node, without having to read
   * the directed edge again.
   * @param endnode  GraphId of the end node of this directed edge.
   */
  void SetEndNode(const baldr::GraphId& endnode);

  /**
   * Get the cost from the origin to this directed edge.
   * @return  Returns the cost (units are based on the costing method).
   */
  float truecost() const;

  /**
   * Set the cost from the origin to this directed edge.
   * @param  truecost  Cost (units are based on the costing method).
   */
  void SetTrueCost(float truecost);

  /**
   * Get the sort cost from the origin to this directed edge. The sort
   * cost includes the A* heuristic.
   * @return  Returns the sort cost (units are based on the costing method).
   */
  float sortcost() const;

  /**
   * Set the sort cost from the origin to this directed edge. The sort
   * cost includes the A* heuristic.
   * @param sortcost  Sort cost (units are based on the costing method).
   */
  void SetSortCost(float sortcost);

  /**
   * Get the index that represents a Uturn.
   * @return  Returns the Uturn index.
   */
  uint32_t uturn_index() const;

  /**
   * Set the index that represents a Uturn.
   * @param  idx  Uturn index.
   */
  void SetUturnIndex(const uint32_t idx);

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

  // Index at the end node of the edge that constitutes a U-turn
  // TODO - combne with other attributes to save space where we can
  uint32_t uturn_index_;

  // Transit information...trip ID , prior stop ID, time at stop? This
  // should probably be part of a derived class
};

}
}

#endif  // VALHALLA_THOR_EDGELABEL_H_
