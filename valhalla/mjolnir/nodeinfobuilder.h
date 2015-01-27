#ifndef VALHALLA_MJOLNIR_NODEINFOBUILDER_H_
#define VALHALLA_MJOLNIR_NODEINFOBUILDER_H_

#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/nodeinfo.h>

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
   * Constructor with arguments
   * @param  ll  Lat,lng position of the node.
   * @param  edge_index  the GraphId of the first outbound edge.
   * @param  edge_count  the number of outbound directed edges.
   */
  NodeInfoBuilder(const PointLL& ll, const uint32_t edge_index,
                  const uint32_t edge_count, const baldr::RoadClass rc);

  /**
   * Sets the latitude and longitude.
   * @param  ll  Lat,lng position of the node.
   */
  void set_latlng(const PointLL& ll);

  /**
   * Set the index within the node's tile of its first outbound edge.
   * @param  edge_index  the GraphId of the first outbound edge.
   */
  void set_edge_index(const uint32_t edge_index);

  /**
   * Set the number of outbound directed edges.
   * @param  edge_count  the number of outbound directed edges.
   */
  void set_edge_count(const uint32_t edge_count);

  /**
   * Sets the best road class of the outbound directed edges.
   * @param  bestrc  Best road class (lowest value).
   */
  void set_bestrc(const baldr::RoadClass bestrc);
};

}
}

#endif  // VALHALLA_MJOLNIR_NODEINFOBUILDER_H_
