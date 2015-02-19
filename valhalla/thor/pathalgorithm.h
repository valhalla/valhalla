#ifndef VALHALLA_THOR_PATHALGORITHM_H_
#define VALHALLA_THOR_PATHALGORITHM_H_

#include <vector>
#include <map>
#include <unordered_map>
#include <utility>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/thor/adjacencylist.h>
#include <valhalla/thor/astarheuristic.h>
#include <valhalla/thor/dynamiccost.h>
#include <valhalla/thor/edgelabel.h>
#include <valhalla/thor/edgestatus.h>

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
          const std::shared_ptr<DynamicCost>& costing);

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
  std::unordered_map<baldr::GraphId, uint32_t> adjlistedges_;

  // Destinations, id and cost
  std::unordered_map<baldr::GraphId, float> destinations_;

  // Destination that was last found with its true cost + partial cost
  std::pair<uint32_t, float> best_destination_;

  /**
   * Initialize
   */
  void Init(const PointLL& origll, const PointLL& destll,
      const std::shared_ptr<DynamicCost>& costing);

  /**
   * Add edges at the origin to the adjacency list
   */
  void SetOrigin(baldr::GraphReader& graphreader, const baldr::PathLocation& origin,
      const std::shared_ptr<DynamicCost>& costing, const baldr::GraphId& loop_edge);

  /**
   * Set the destination edge(s).
   */
  void SetDestination(baldr::GraphReader& graphreader, const baldr::PathLocation& dest,
     const std::shared_ptr<DynamicCost>& costing);

  /**
   * Return a valid edge id if we've found the destination edge
   *
   * @param edge_label_index     edge label to be tested for destination
   * @return bool                true if we've found the destination
   */
  bool IsComplete(const uint32_t edge_label_index);

  /**
   * Form the path from the adjacency list.
   * TODO - support partial distances at origin/destination
   */
  std::vector<baldr::GraphId> FormPath(const uint32_t dest,
                                       baldr::GraphReader& graphreader, const baldr::GraphId& loop);

  /**
   * TODO - are we keeping these?
   */
  baldr::GraphId GetStartNode(baldr::GraphReader& graphreader,
                              const baldr::DirectedEdge* directededge);
  std::vector<baldr::GraphId> FormLocalPath(const uint32_t dest,
                              baldr::GraphReader& graphreader);
  baldr::GraphId RecoverShortcut(baldr::GraphReader& graphreader,
                                 const baldr::GraphId& startnode,
                                 const baldr::GraphId& endnode,
                                 const baldr::DirectedEdge* shortcutedge,
                                 std::vector<baldr::GraphId>& edgesonpath);

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
