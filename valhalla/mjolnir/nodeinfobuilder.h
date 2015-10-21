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
   * @param  rc             Best road class / importance of outbound edges.
   * @param  access         Access mask at this node.
   * @param  type           The type of node.
   * @param  traffic_signal Has a traffic signal at this node?
   */
  NodeInfoBuilder(const std::pair<float, float>& ll, const baldr::RoadClass rc,
                  const uint32_t access, const baldr::NodeType type,
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
   * Set the intersection type.
   * @param  type   Intersection type (see baldr/graphconstants.h)
   */
  void set_intersection(const baldr::IntersectionType type);

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
   * Set the auto driveability of the local directed edge given a local
   * edge index.
   * @param  localidx  Local edge index.
   * @param  t         Traversability (see graphconstants.h)
   */
  void set_local_driveability(const uint32_t localidx,
                              const baldr::Traversability t);

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
   * Set the number of edges on the local level (up to kMaxLocalEdgeInfo+1).
   * @param  n  Number of edges on the local level.
   */
  void set_local_edge_count(const uint32_t n);

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
   * Sets the flag indicating a mode change is allowed at this node.
   * The access data tells which modes are allowed at the node. Examples
   * include transit stops, bike share locations, and parking locations.
   * @param  mc  True if a mode change is allowed at the node.
   */
  void set_mode_change(const bool mc);

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

  /**
   * Set the name consistency between a pair of local edges. This is limited
   * to the first 8 local edge indexes.
   * @param  from  Local index of the from edge.
   * @param  to    Local index of the to edge.
   * @param  c     Are names consistent between the 2 edges?
   */
  void set_name_consistency(const uint32_t from, const uint32_t to,
                            const bool c);

  /**
   * Set the heading of the local edge given its local index. Supports
   * up to 8 local edges. Headings are reduced to 8 bits.
   * @param  localidx  Local edge index.
   * @param  heading   Heading relative to N (0-359 degrees).
   */
  void set_heading(uint32_t localidx, uint32_t heading);
};

}
}

#endif  // VALHALLA_MJOLNIR_NODEINFOBUILDER_H_
