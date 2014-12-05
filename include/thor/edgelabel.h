#ifndef VALHALLA_THOR_EDGELABEL_H_
#define VALHALLA_THOR_EDGELABEL_H_

#include "baldr/graphid.h"
#include "baldr/directededge.h"

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
  EdgeLabel();

  // TODO - add methods to construct the label

  // TODO - add methods to update an existing label


  virtual ~EdgeLabel() {
  }

  const baldr::GraphId& edgeid() const;

  void SetEdgeId(const baldr::GraphId& edgeid);

  const baldr::GraphId& endnode() const;

  void SetEndNode(const baldr::GraphId& endnode);

  /**
  const EdgeLabel*& predecessor() const;

  void SetPredecessor(const EdgeLabel*& predecessor);
**/
  float sortcost() const;

  void SetSortCost(float sortcost);

  float truecost() const;

  void SetTrueCost(float truecost);

 private:
  // Pointer to the predecessor edge and its label information.
  // TODO - need setter and getter. Should this be const?
  EdgeLabel* predecessor_;

  // Graph Id of the edge. TODO - do we need the edge ID or can
  // it be recovered easily?
  baldr::GraphId edgeid_;

  // GraphId of the end node of the edge. This would allow the expansion
  // to occur by reading the node and not having to re-read the directed
  // edge
  baldr::GraphId endnode_;

  // Cost (time, distance, etc.)
  float truecost_;

  // Sort cost - includes A* heuristic
  float sortcost_;

  // Transit information...trip ID , prior stop ID, time at stop?

};

}
}

#endif  // VALHALLA_THOR_EDGELABEL_H_
