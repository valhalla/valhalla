#ifndef VALHALLA_THOR_PATHALGORITHM_H_
#define VALHALLA_THOR_PATHALGORITHM_H_

#include "baldr/graphreader.h"
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

  /**
   * Form path.
   * TODO - define inputs and outputs!
   */
  bool GetBestPath(baldr::GraphReader& graphreader, EdgeCost* edgecost);

 protected:

  // A* heuristic
  AStarHeuristic astarheuristic_;

  // Adjacency list
  AdjacencyList* adjacencylist_;

  // Done set
  std::vector<EdgeLabel*> doneset_;

  // Edge status
  EdgeStatus* edgestatus_;

  /**
   * Initialize
   */
  void Init(const PointLL& destll, EdgeCost* edgecost);

  /**
   * Add an edge at the origin to the adjacency list
   */
  void AddOriginEdge(const baldr::DirectedEdge& edge, EdgeCost* edgecost);

  /**
   * Clear the temporary information generated during path construction.
   */
  void Clear();

  /**
   * Test if the shortest path is found.
   */
  bool IsComplete();
};

}
}

#endif  // VALHALLA_THOR_PATHALGORITHM_H_
