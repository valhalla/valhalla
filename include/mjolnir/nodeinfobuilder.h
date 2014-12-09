#ifndef VALHALLA_MJOLNIR_NODEINFOBUILDER_H_
#define VALHALLA_MJOLNIR_NODEINFOBUILDER_H_

#include "geo/pointll.h"
#include "baldr/graphid.h"
#include "baldr/nodeinfo.h"

using namespace valhalla::geo;

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
   * Set the GraphId of the first outbound edge from this node.
   * @param  edge_id  the GraphId of the first outbound edge.
   */
  void set_edge_id(const baldr::GraphId& edge_id);

  /**
   * Set the number of outbound directed edges.
   * @param  edge_count  the number of outbound directed edges.
   */
  void set_edge_count(const unsigned int edge_count);

};

}
}

#endif  // VALHALLA_MJOLNIR_NODEINFOBUILDER_H_
