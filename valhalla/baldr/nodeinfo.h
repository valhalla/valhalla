#ifndef VALHALLA_BALDR_NODEINFO_H_
#define VALHALLA_BALDR_NODEINFO_H_

#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/util.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphconstants.h>

using namespace valhalla::midgard;

namespace valhalla {
namespace baldr {

constexpr uint32_t kMaxTileEdgeCount  = 4194303;  // 2^22 directed edges
constexpr uint32_t kMaxEdgesPerNode   = 127;      // Maximum edges per node
constexpr uint32_t kMaxLocalDriveable = 7;        // Max. index of driveable
                                                  // edges on local level
constexpr uint32_t kMaxDensity = 15;              // Max. relative node density

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
  uint32_t edge_index() const;

  /**
   * Get the number of outbound directed edges. This includes all edge
   * present on the current hierarchy level.
   * @return  Returns the number of outbound directed edges.
   */
  uint32_t edge_count() const;

  /**
   * Get the best road class of the outbound directed edges.
   * @return   Returns road class.
   */
  RoadClass bestrc() const;

  /**
   * Get the access modes (bit mask) allowed to pass through the node.
   * See graphconstants.h
   * @return  Returns the access bit mask indicating allowable modes.
   */
  uint8_t access() const;

  /**
   * Get the intersection type.
   * @return  Returns the intersection type.
   */
  IntersectionType intersection() const;

  /**
   * Get the index of the administrative information within this tile.
   * @return  Returns an index within the tile's administrative information.
   */
  uint32_t admin_index() const;

  /**
   * Returns the timezone index. TODO - describe the timezone information.
   * @return  Returns the timezone index.
   */
  uint32_t timezone() const;

  /**
   * Is daylight saving time observed at the node's location.
   * @return  Returns true if daylight savings time is observed.
   */
  bool dst() const;

  /**
   * Get the driveability of the local directed edge given a local
   * edge index.
   * @param  localidx  Local edge index.
   * @return Returns driveability (see graphconstants.h)
   */
  Driveability local_driveability(const uint32_t localidx) const;

  /**
   * Get the relative density (TODO - define) at the node.
   * @return  Returns relative density (0-15).
   */
  uint32_t density() const;

  /**
   * Gets the node type. See graphconstants.h for the list of types.
   * @return  Returns the node type.
   */
  NodeType type() const;

  /**
   * Get the number of driveable edges on the local level.
   * @return  Returns the number of driveable edges on the local level.
   */
  uint32_t local_driveable() const;

  /**
   * Is this a dead-end node that connects to only one edge?
   * @return  Returns true if this is a dead-end node.
   */
  bool end() const;

  /**
   * Is this a parent node (e.g. a parent transit stop).
   * @return  Returns true if this is a parent node.
   */
  bool parent() const;

  /**
   * Is this a child node (e.g. a child transit stop).
   * @return  Returns true if this is a child node.
   */
  bool child() const;

  /**
   * Is a mode change allowed at this node? The access data tells which
   * modes are allowed at the node. Examples include transit stops, bike
   * share locations, and parking locations.
   * @return  Returns true if mode changes are allowed.
   */
  bool mode_change() const;

  /**
   * Is there a traffic signal at this node?
   * @return  Returns true if there is a traffic signal at the node.
   */
  bool traffic_signal() const;

  /**
   * Gets the transit stop Id. This is used for schedule lookups
   * and possibly queries to a transit service.
   * @return  Returns the transit stop Id.
   */
  uint32_t stop_id() const;

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
   * Get the heading of the local edge given its local index. Supports
   * up to 8 local edges. Headings are stored rounded off to 2 degree
   * values.
   * @param  localidx  Local edge index.
   * @return Returns heading relative to N (0-360 degrees).
   */
  uint32_t heading(const uint32_t localidx) const;

  /**
   * Get the computed version of NodeInfo attributes.
   * @return   Returns internal version.
   */
  static const uint64_t internal_version();

 protected:
  // Latitude, longitude position of the node.
  std::pair<float, float> latlng_;

  // Node attributes.
  struct NodeAttributes {
    uint32_t edge_index_  : 22; // Index within the node's tile of its first
                                // outbound directed edge
    uint32_t edge_count_   : 7; // Number of outbound edges (on this level)
    uint32_t bestrc_       : 3; // Best directed edge road class
  };
  NodeAttributes attributes_;

  // Node access (see graphconstants.h)
  Access access_;

  // Intersection type. Classification of the intersection.
  // (see graphconstants.h)
  IntersectionType intersection_;

  // Administrative information
  struct NodeAdmin {
    uint16_t admin_index  : 6; // Index into this tile's list of admin data
    uint16_t timezone     : 6; // Time zone
    uint16_t dst          : 1; // Is Daylight Saving Time used?
    uint16_t spare        : 3;
  };
  NodeAdmin admin_;

  // Node type
  struct NodeTypeInfo {
    uint32_t local_driveability : 16; // Driveability of up to 8 local edges
    uint32_t density            : 4;  // Density (population? edges?)
    uint32_t type               : 4;  // Node type
    uint32_t local_driveable    : 3;  // # of driveable edges on local level
    uint32_t end                : 1;  // End node (only connects to 1 edge)
    uint32_t parent             : 1;  // Is this a parent node
    uint32_t child              : 1;  // Is this a child node
    uint32_t mode_change        : 1;  // Mode change allowed?
    uint32_t traffic_signal     : 1;  // Traffic signal

  };
  NodeTypeInfo type_;

  // Transit stop Id
  union NodeStop {
    uint32_t stop_id;
    uint32_t name_consistency;
  };
  NodeStop stop_;

  // Headings of up to 8 local edges (rounded to nearest 2 degrees)
  uint64_t headings_;
};

}
}

#endif  // VALHALLA_BALDR_NODEINFO_H_
