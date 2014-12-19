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
   * Get the index of the first outbound edge from this node. Since
   * all outbound edges are in the same tile/level as the node we
   * only need an index within the tile.
   * @return  Returns the GraphId of the first outbound edge.
   */
  unsigned int edge_index() const;

  /**
   * Get the number of outbound directed edges.
   * @return  Returns the number of outbound directed edges.
   */
  unsigned int edge_count() const;

 protected:
  // Latitude, longitude position of the node.
  PointLL latlng_;

  // TODO - add attribution and compress

  // Index within the node's tile of its first directed edge outbound
  unsigned int edge_index_;

  // Number of outbound edges
  // TODO - add this to a bit field to compress with other node data.
  // Rather than number of driveable, we can probably sort by driveability
  // to optimized for drving routes - when the first non driveable edge is
  // encountered the successive edges can be skipped
  unsigned int edge_count_;
};

}
}

#endif  // VALHALLA_BALDR_NODEINFO_H_
