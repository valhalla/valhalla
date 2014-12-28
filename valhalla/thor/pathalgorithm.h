#ifndef VALHALLA_THOR_PATHALGORITHM_H_
#define VALHALLA_THOR_PATHALGORITHM_H_

#include <vector>
#include <map>

#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/pathlocation.h>
#include "thor/adjacencylist.h"
#include "thor/astarheuristic.h"
#include "thor/dynamiccost.h"
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
  std::vector<baldr::GraphId> GetBestPath(const baldr::PathLocation& origin,
          const baldr::PathLocation& dest, baldr::GraphReader& graphreader,
           DynamicCost* costing);

  /**
   * Clear the temporary information generated during path construction.
   */
  void Clear();

 protected:

  // A* heuristic
  AStarHeuristic astarheuristic_;

  // List of edge labels
  uint64_t edgelabel_index_;
  std::vector<EdgeLabel> edgelabels_;

  // Adjacency list
  AdjacencyList* adjacencylist_;

  // Edge status
  EdgeStatus* edgestatus_;

  // Map of edges in the adjacency list. Keep this map so we do not have
  // to search to find an entry that is already in the adjacency list
  std::map<uint64_t, uint32_t> adjlistedges_;

  // Destinations
  std::map<uint64_t, float> destinations_;

  /**
   * Initialize
   */
  void Init(const PointLL& origll, const PointLL& destll,
            DynamicCost* costing);

  /**
   * Add edges at the origin to the adjacency list
   */
  void SetOrigin(baldr::GraphReader& graphreader,
        const baldr::PathLocation& origin, DynamicCost* costing);

  /**
   * Set the destination edge(s).
   */
  void SetDestination(const baldr::PathLocation& dest);

  /**
   * Test if the shortest path is found.
   */
  bool IsComplete(const baldr::GraphId& edgeid);

  /**
   * Form the path from the adjacency list.
   * TODO - support partial distances at origin/destination
   */
  std::vector<baldr::GraphId> FormPath(const uint32_t dest);

  /**
   * Gets the edge label for an edge that is in the adjacency list.
   */
  uint32_t GetPriorEdgeLabel(const baldr::GraphId& edgeid) const;

  /*
   * Remove the edge label from the map of edges in the adjacency list
   */
  void RemoveFromAdjMap(const baldr::GraphId& edgeid);
};

}
}

#endif  // VALHALLA_THOR_PATHALGORITHM_H_
