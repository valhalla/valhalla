#ifndef VALHALLA_BALDR_NODEINFO_H_
#define VALHALLA_BALDR_NODEINFO_H_

#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/util.h>
#include <valhalla/baldr/graphid.h>

using namespace valhalla::midgard;

namespace valhalla {
namespace baldr {

/**
 * Information held for each node within the graph. The graph uses a forward
 * star structure: nodes point to the first outbound directed edge and each
 * directed edge points to the other end node of the edge.
 * @author  David W. Nesbitt
 */
class NodeInfo {
 public:
  /**
   * Constructor
   */
  NodeInfo();

  /**
   * Get the latitude, longitude of the node.
   * @return  Returns the latitude and longitude of the node.
   */
  const PointLL& latlng() const;

  /**
   * Get the GraphId of the node on the upward hierarchy that this
   * node shares. Is an empty GraphId if no connection occurs.
   * TODO - set a flag?
   * @return  Returns the GraphId of the upward connection.
   */
//  const GraphId& upnode() const;

  /**
   * Get the index of the first outbound edge from this node. Since
   * all outbound edges are in the same tile/level as the node we
   * only need an index within the tile.
   * @return  Returns the GraphId of the first outbound edge.
   */
  uint32_t edge_index() const;

  /**
   * Get the number of outbound directed edges.
   * @return  Returns the number of outbound directed edges.
   */
  uint32_t edge_count() const;

  /**
   * Get the best road class of the outbound directed edges.
   * TODO - should use enum?
   * @return   Returns road class.
   */
  uint32_t bestrc() const;

 protected:
  // Latitude, longitude position of the node.
  PointLL latlng_;

  // Upward transition node at the next hierarchy level
  // TODO - evaluate adding a directed edge instead. Makes the hierarchy
  // builder more complex (and maybe PathAlgorithm) but would use less
  // memory.
//  baldr::GraphId upnode_;

  // Node attributes.
  // TODO - what is the max. number of edges within the tile?
  // TODO - do we want/need number of driveable? We can probably sort by
  // driveability to optimize for driving routes - when the first non
  // driveable edge is encountered the successive edges can be skipped
  struct NodeAttributes {
    uint32_t edge_index_  : 18; // Index within the node's tile of its first
                                // outbound directed edge
    uint32_t edge_count_   : 5; // Number of outbound edges
    uint32_t bestrc_       : 1; // Best directed edge road class
    uint32_t spare         : 8;
  };
  NodeAttributes attributes_;
};

}
}

#endif  // VALHALLA_BALDR_NODEINFO_H_
