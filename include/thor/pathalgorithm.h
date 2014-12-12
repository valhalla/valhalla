#ifndef VALHALLA_THOR_PATHALGORITHM_H_
#define VALHALLA_THOR_PATHALGORITHM_H_

#include "thor/adjacencylist.h"
#include "thor/astarheuristic.h"
#include "thor/edgecost.h"
#include "thor/edgelabel.h"
#include "thor/edgestatus.h"

namespace valhalla {
namespace thor {

/**
 * Algorithm to create shortest path.
 */
class PathAlgorithm {
 public:
  /**
   * Constructor.
   */
  PathAlgorithm();

  /**
   * Destructor
   */
  virtual ~PathAlgorithm();

 protected:

  // A* heuristic
  AStarHeuristic astarheuristic_;

  // Adjacency list
  AdjacencyList<EdgeLabel>* adjacencylist_;

  // Done set
  std::vector<EdgeLabel*> doneset_;

  // Edge costing
  EdgeCost* edgecost_;

  // Edge status
  EdgeStatus* edgestatus_;

};

}
}

#endif  // VALHALLA_THOR_PATHALGORITHM_H_
