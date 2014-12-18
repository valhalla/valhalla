#ifndef VALHALLA_MJOLNIR_NODEINFOBUILDER_H_
#define VALHALLA_MJOLNIR_NODEINFOBUILDER_H_

#include "../midgard/pointll.h"
#include "baldr/graphid.h"
#include "baldr/nodeinfo.h"

using namespace valhalla::midgard;

namespace valhalla{
namespace mjolnir{

/**
 * Information held for each node within the graph. The graph uses a forward
 * star structure: nodes point to the first outbound directed edge and each
 * directed edge points to the other end node of the edge.
 */
class NodeInfoBuilder : public baldr::NodeInfo {
 public:
  /**
   * Constructor
   */
  NodeInfoBuilder();

  /**
   * Sets the latitude and longitude.
   * @param  ll  Lat,lng position of the node.
   */
  void set_latlng(const PointLL& ll);

  /**
   * Set the index within the node's tile of its first outbound edge.
   * @param  edge_index  the GraphId of the first outbound edge.
   */
  void set_edge_index(const unsigned int edge_index);

  /**
   * Set the number of outbound directed edges.
   * @param  edge_count  the number of outbound directed edges.
   */
  void set_edge_count(const unsigned int edge_count);

};

}
}

#endif  // VALHALLA_MJOLNIR_NODEINFOBUILDER_H_
