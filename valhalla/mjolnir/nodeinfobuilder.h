#ifndef VALHALLA_MJOLNIR_NODEINFOBUILDER_H_
#define VALHALLA_MJOLNIR_NODEINFOBUILDER_H_

#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/nodeinfo.h>

using namespace valhalla::midgard;

namespace valhalla {
namespace mjolnir {

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
   * @param  edge_index     The GraphId of the first outbound edge.
   * @param  edge_count     The number of outbound directed edges.
   * @param  rc             Best road class / importance of outbound edges.
   * @param  access         Access mask at this node.
   * @param  type           The type of node.
   * @param  end            Is a dead-end node?
   * @param  traffic_signal Has a traffic signal at this node?
   *
   */
  NodeInfoBuilder(const std::pair<float, float>& ll, const uint32_t edge_index,
                  const uint32_t edge_count, const baldr::RoadClass rc,
                  const uint32_t access, const baldr::NodeType type, const bool end,
                  const bool traffic_signal);

  /**
   * Sets the latitude and longitude.
   * @param  ll  Lat,lng position of the node.
   */
  void set_latlng(const std::pair<float, float>& ll);

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
   * @param  bestrc  Best road class / importance (lowest value).
   */
  void set_bestrc(const baldr::RoadClass bestrc);

  /**
   * Set the access modes (bit mask) allowed to pass through the node.
   * @param  access  Access mask.
   */
  void set_access(const uint32_t access);

  /**
   * Set the index of the administrative information within this tile.
   * @param  admin_index  admin index.
   */
  void set_admin_index(const uint16_t admin_index);

  /**
   * Set the timezone index.
   * @param  timezone  timezone index.
   */
  void set_timezone(const uint16_t timezone);

  /**
   * Set the daylight saving time flag
   * @param  dst  dst flag.
   */
  void set_dst(const bool dst);

  /**
   * Set the relative density
   * @param  density  density.
   */
  void set_density(const uint32_t density);

  /**
   * Set the node type.
   * @param  type  node type.
   */
  void set_type(const baldr::NodeType type);

  /**
   * Set the dead-end node flag.
   * @param  end  dead-end flag.
   */
  void set_end(const bool end);

  /**
   * Set the parent node flag (e.g. a parent transit stop).
   * @param  parent  parent node flag.
   */
  void set_parent(const bool parent);

  /**
   * Set the child node flag (e.g. a child transit stop).
   * @param  child  child node flag.
   */
  void set_child(const bool child);

  /**
   * Set the traffic signal flag.
   * @param  traffic_signal  taffic signal flag.
   */
  void set_traffic_signal(const bool traffic_signal);

  /**
   * Set the transit stop Id.
   * @param  stop_id  transit stop id.
   */
  void set_stop_id(const uint32_t stop_id);

};

}
}

#endif  // VALHALLA_MJOLNIR_NODEINFOBUILDER_H_
