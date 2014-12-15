#ifndef VALHALLA_THOR_PATHALGORITHM_H_
#define VALHALLA_THOR_PATHALGORITHM_H_

#include <vector>
#include <map>

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
  std::vector<baldr::GraphId> GetBestPath(baldr::GraphReader& graphreader,
                  EdgeCost* edgecost);

 protected:

  // A* heuristic
  AStarHeuristic astarheuristic_;

  // Adjacency list
  AdjacencyList* adjacencylist_;

  // Done set
  std::vector<EdgeLabel*> doneset_;

  // Edge status
  EdgeStatus* edgestatus_;

  // Map of edges in the adjacency list. Keep this map so we do not have
  // to search to find an entry that is already in the adjacency list
  std::map<baldr::GraphId, EdgeLabel*> adjlistedges_;

  // Destinations
  std::map<baldr::GraphId, float> destinations_;

  /**
   * Initialize
   */
  void Init(const PointLL& origll, const PointLL& destll, EdgeCost* edgecost);

  /**
   * Add an edge at the origin to the adjacency list
   */
  void AddOriginEdge(baldr::GraphReader& graphreader,
        const baldr::GraphId& edgeid, EdgeCost* edgecost);

  /**
   * Add a destination edge.
   */
  void AddDestinationEdge(const baldr::GraphId& edgeid);

  /**
   * Test if the shortest path is found.
   */
  bool IsComplete(const baldr::GraphId& edgeid);

  /**
   * Clear the temporary information generated during path construction.
   */
  void Clear();

  /**
   * Form the path from the adjacency list.
   * TODO - support partial distances at origin/destination
   */
  std::vector<baldr::GraphId> FormPath(const EdgeLabel* dest);

  /**
   * Gets the edge label for an edge that is in the adjacency list.
   */
  EdgeLabel* GetPriorEdgeLabel(const baldr::GraphId& edgeid) const;

  /*
   * Remove the edge label from the map of edges in the adjacency list
   */
  void RemoveFromAdjMap(const baldr::GraphId& edgeid);
};

}
}

#endif  // VALHALLA_THOR_PATHALGORITHM_H_
