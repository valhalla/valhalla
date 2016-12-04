#ifndef VALHALLA_BALDR_NODEINFO_H_
#define VALHALLA_BALDR_NODEINFO_H_

#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/util.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/json.h>

using namespace valhalla::midgard;

namespace valhalla {
namespace baldr {

class GraphTile;

constexpr uint32_t kMaxEdgesPerNode     = 127;      // Maximum edges per node
constexpr uint32_t kMaxAdminsPerTile    = 63;       // Maximum Admins per tile
constexpr uint32_t kMaxTimeZonesPerTile = 511;      // Maximum TimeZones index
constexpr uint32_t kMaxLocalEdgeIndex   = 7;        // Max. index of edges on
                                                    // local level

// Heading shrink factor to reduce max heading of 359 to 255
constexpr float kHeadingShrinkFactor = (255.f/359.f);

// Heading expand factor to increase max heading of 255 to 359
constexpr float kHeadingExpandFactor = (359.f/255.f);

/**
 * Information held for each node within the graph. The graph uses a forward
 * star structure: nodes point to the first outbound directed edge and each
 * directed edge points to the other end node of the edge.
 */
class NodeInfo {
 public:
  /**
   * Constructor
   */
  NodeInfo();

  /**
   * Constructor with arguments
   * @param  ll  Lat,lng position of the node.
   * @param  rc             Best road class / importance of outbound edges.
   * @param  access         Access mask at this node.
   * @param  type           The type of node.
   * @param  traffic_signal Has a traffic signal at this node?
   */
  NodeInfo(const std::pair<float, float>& ll, const baldr::RoadClass rc,
                  const uint32_t access, const baldr::NodeType type,
                  const bool traffic_signal);

  /**
   * Get the latitude, longitude of the node.
   * @return  Returns the latitude and longitude of the node.
   */
  const PointLL& latlng() const {
    return static_cast<const PointLL&>(latlng_);
  }

  /**
   * Sets the latitude and longitude.
   * @param  ll  Lat,lng position of the node.
   */
  void set_latlng(const std::pair<float, float>& ll);

  /**
   * Get the index of the first outbound edge from this node. Since
   * all outbound edges are in the same tile/level as the node we
   * only need an index within the tile.
   * @return  Returns the GraphId of the first outbound edge.
   */
  uint32_t edge_index() const {
    return edge_index_;
  }

  /**
   * Set the index within the node's tile of its first outbound edge.
   * @param  edge_index  the GraphId of the first outbound edge.
   */
  void set_edge_index(const uint32_t edge_index);

  /**
   * Get the number of outbound directed edges. This includes all edge
   * present on the current hierarchy level.
   * @return  Returns the number of outbound directed edges.
   */
  uint32_t edge_count() const {
    return edge_count_;
  }

  /**
   * Set the number of outbound directed edges.
   * @param  edge_count  the number of outbound directed edges.
   */
  void set_edge_count(const uint32_t edge_count);

  /**
   * Get the access modes (bit mask) allowed to pass through the node.
   * See graphconstants.h for access constants.
   * @return  Returns the access bit mask indicating allowable modes.
   */
  uint16_t access() const {
    return access_;
  }

  /**
   * Set the access modes (bit mask) allowed to pass through the node.
   * * See graphconstants.h for access constants.
   * @param  access  Access mask.
   */
  void set_access(const uint32_t access);

  /**
   * Get the intersection type.
   * @return  Returns the intersection type.
   */
  IntersectionType intersection() const {
    return static_cast<IntersectionType>(intersection_);
  }

  /**
   * Set the intersection type.
   * @param  type   Intersection type (see baldr/graphconstants.h)
   */
  void set_intersection(const baldr::IntersectionType type);

  /**
   * Get the index of the administrative information within this tile.
   * @return  Returns an index within the tile's administrative information.
   */
  uint32_t admin_index() const {
    return admin_index_;
  }

  /**
   * Set the index of the administrative information within this tile.
   * @param  admin_index  admin index.
   */
  void set_admin_index(const uint16_t admin_index);

  /**
   * Returns the timezone index. TODO - describe the timezone information.
   * @return  Returns the timezone index.
   */
  uint32_t timezone() const {
    return timezone_;
  }

  /**
   * Set the timezone index.
   * @param  timezone  timezone index.
   */
  void set_timezone(const uint32_t timezone);

  /**
   * Get the driveability of the local directed edge given a local
   * edge index.
   * @param  localidx  Local edge index.
   * @return Returns traversability (see graphconstants.h)
   */
  Traversability local_driveability(const uint32_t localidx) const {
    uint32_t s = localidx * 2;     // 2 bits per index
    return static_cast<Traversability>((local_driveability_ & (3 << s)) >> s);
  }

  /**
   * Set the auto driveability of the local directed edge given a local
   * edge index.
   * @param  localidx  Local edge index.
   * @param  t         Traversability (see graphconstants.h)
   */
  void set_local_driveability(const uint32_t localidx,
                              const baldr::Traversability t);

  /**
   * Get the relative road density at the node.
   * @return  Returns relative density (0-15).
   */
  uint32_t density() const {
    return density_;
  }

  /**
   * Set the relative road density
   * @param  density  density.
   */
  void set_density(const uint32_t density);

  /**
   * Gets the node type. See graphconstants.h for the list of types.
   * @return  Returns the node type.
   */
  NodeType type() const {
    return static_cast<NodeType>(type_);
  }

  /**
   * Set the node type.
   * @param  type  node type.
   */
  void set_type(const baldr::NodeType type);

  /**
   * Checks if this node is a transit node.
   * @return  Returns true if this node is a transit node.
   */
  bool is_transit() const {
    return type() == NodeType::kMultiUseTransitStop;
  }

  /**
   * Get the number of regular edges across all levels (up to
   * kMaxLocalEdgeIndex+1). Does not include shortcut edges,
   * transit edges and transit connections, and transition edges.
   * @return  Returns the number of edges on the local level.
   */
  uint32_t local_edge_count() const {
    return local_edge_count_ + 1;
  }

  /**
   * Set the number of edges on the local level (up to kMaxLocalEdgeInfo+1).
   * @param  n  Number of edges on the local level.
   */
  void set_local_edge_count(const uint32_t n);

  /**
   * Is a mode change allowed at this node? The access data tells which
   * modes are allowed at the node. Examples include transit stops, bike
   * share locations, and parking locations.
   * @return  Returns true if mode changes are allowed.
   */
  bool mode_change() const {
    return mode_change_;
  }

  /**
   * Sets the flag indicating a mode change is allowed at this node.
   * The access data tells which modes are allowed at the node. Examples
   * include transit stops, bike share locations, and parking locations.
   * @param  mc  True if a mode change is allowed at the node.
   */
  void set_mode_change(const bool mc);

  /**
   * Is there a traffic signal at this node?
   * @return  Returns true if there is a traffic signal at the node.
   */
  bool traffic_signal() const {
    return traffic_signal_;
  }

  /**
   * Set the traffic signal flag.
   * @param  traffic_signal  taffic signal flag.
   */
  void set_traffic_signal(const bool traffic_signal);

  /**
   * Gets the transit stop index. This is used for schedule lookups
   * and possibly queries to a transit service.
   * @return  Returns the transit stop index.
   */
  uint32_t stop_index() const {
    return stop_.stop_index;
  }

  /**
   * Set the transit stop index.
   * @param  stop_index  transit stop index.
   */
  void set_stop_index(const uint32_t stop_index);

  /**
   * Get the name consistency between a pair of local edges. This is limited
   * to the first 8 local edge indexes.
   * @param  from  Local index of the from edge.
   * @param  to    Local index of the to edge.
   * @return  Returns true if names are consistent, false if not (or if from
   *          or to index exceeds max).
   */
  bool name_consistency(const uint32_t from, const uint32_t to) const;

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
   * Get the heading of the local edge given its local index. Supports
   * up to 8 local edges. Headings are stored rounded off to 2 degree
   * values.
   * @param  localidx  Local edge index.
   * @return Returns heading relative to N (0-360 degrees).
   */
  uint32_t heading(const uint32_t localidx) const;

  /**
   * Set the heading of the local edge given its local index. Supports
   * up to 8 local edges. Headings are reduced to 8 bits.
   * @param  localidx  Local edge index.
   * @param  heading   Heading relative to N (0-359 degrees).
   */
  void set_heading(uint32_t localidx, uint32_t heading);

  /**
   * Returns the json representation of the object
   * @param   the tile required to get admin information
   * @return  json object
   */
  json::MapPtr json(const GraphTile* tile) const;

 protected:
  // Latitude, longitude position of the node.
  std::pair<float, float> latlng_;

  // Node attributes and admin information
  uint64_t edge_index_       : 21;  // Index within the node's tile of its
                                    // first outbound directed edge
  uint64_t access_           : 12;  // Access through the node - bit field
  uint64_t edge_count_       : 7;   // Number of outbound edges (on this level)
  uint32_t local_edge_count_ : 3;   // # of regular edges across all levels
                                    // (up to kMaxLocalEdgeIndex+1)
  uint64_t admin_index_      : 6;   // Index into this tile's admin data list
  uint64_t timezone_         : 9;   // Time zone
  uint64_t intersection_     : 5;   // Intersection type
  uint64_t spare_0           : 1;   //maybe add to admin_index?

  // Node type and additional node attributes
  uint32_t local_driveability_ : 16; // Driveability for regular edges (up to
                                     // kMaxLocalEdgeIndex+1 edges)
  uint32_t density_            : 4;  // Relative road density
  uint32_t type_               : 4;  // NodeType, see graphconstants
  uint32_t mode_change_        : 1;  // Mode change allowed?
  uint32_t traffic_signal_     : 1;  // Traffic signal
  uint32_t spare_1             : 6;

  // Transit stop index
  union NodeStop {
    uint32_t stop_index;
    uint32_t name_consistency;
  };
  NodeStop stop_;

  // Headings of up to kMaxLocalEdgeIndex+1 local edges (rounded to
  // nearest 2 degrees)
  uint64_t headings_;
};

}
}

#endif  // VALHALLA_BALDR_NODEINFO_H_
