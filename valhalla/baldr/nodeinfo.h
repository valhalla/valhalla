#ifndef VALHALLA_BALDR_NODEINFO_H_
#define VALHALLA_BALDR_NODEINFO_H_

#include <cstdint>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphtileptr.h>
#include <valhalla/baldr/json.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/util.h>

namespace valhalla {
namespace baldr {

constexpr uint32_t kMaxEdgesPerNode = 127;     // Maximum edges per node
constexpr uint32_t kMaxAdminsPerTile = 4095;   // Maximum Admins per tile
constexpr uint32_t kMaxTimeZonesPerTile = 511; // Maximum TimeZones index
constexpr uint32_t kMaxLocalEdgeIndex = 7;     // Max. index of edges on local level

// Heading shrink factor to reduce max heading of 359 to 255
constexpr float kHeadingShrinkFactor = (255.0f / 359.0f);

// Heading expand factor to increase max heading of 255 to 359
constexpr float kHeadingExpandFactor = (359.0f / 255.0f);

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
   * @param  tile_corner    Lower left (SW) corner of the tile that contains the node.
   * @param  ll             Lat,lng position of the node.
   * @param  access         Access mask at this node.
   * @param  type           The type of node.
   * @param  traffic_signal Has a traffic signal at this node?
   */
  NodeInfo(const midgard::PointLL& tile_corner,
           const midgard::PointLL& ll,
           const uint32_t access,
           const baldr::NodeType type,
           const bool traffic_signal,
           const bool tagged_access,
           const bool private_access,
           const bool cash_only_toll);

  /**
   * Get the latitude, longitude of the node.
   * @param tile_corner Lower left (SW) corner of the tile.
   * @return  Returns the latitude and longitude of the node.
   */
  midgard::PointLL latlng(const midgard::PointLL& tile_corner) const {
    return midgard::PointLL(tile_corner.lng() + (lon_offset_ * 1e-6 + lon_offset7_ * 1e-7),
                            tile_corner.lat() + (lat_offset_ * 1e-6 + lat_offset7_ * 1e-7));
  }

  /**
   * Sets the latitude and longitude.
   * @param  tile_corner Lower left (SW) corner of the tile.
   * @param  ll  Lat,lng position of the node.
   */
  void set_latlng(const midgard::PointLL& tile_corner, const midgard::PointLL& ll);

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
    uint32_t s = localidx * 2; // 2 bits per index
    return static_cast<Traversability>((local_driveability_ & (3 << s)) >> s);
  }

  /**
   * Set the auto driveability of the local directed edge given a local
   * edge index.
   * @param  localidx  Local edge index.
   * @param  t         Traversability (see graphconstants.h)
   */
  void set_local_driveability(const uint32_t localidx, const baldr::Traversability t);

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
   * Evaluates a basic set of conditions to determine if this node is eligible for contraction.
   * @return true if the node has at least 2 edges and does not represent a fork, gate or toll booth.
   */
  bool can_contract() const {
    return edge_count() >= 2 && intersection() != IntersectionType::kFork &&
           type() != NodeType::kGate && type() != NodeType::kTollBooth &&
           type() != NodeType::kTollGantry && type() != NodeType::kSumpBuster;
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
    return (type() == NodeType::kMultiUseTransitPlatform);
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
   * Is driving on the right hand side of the road along edges originating at this node?
   * @return  Returns true outbound edges use right-side driving, false if
   *          left-side driving.
   */
  bool drive_on_right() const {
    return drive_on_right_;
  }

  /**
   * Set the flag indicating driving is on the right hand side of the road
   * for outbound edges from this node.
   * @param rsd  True if outbound edges use right-side driving, false if
   *             left-side driving.
   */
  void set_drive_on_right(const bool rsd);

  /**
   * Was the access information originally set in the data?
   * True if any tags like "access", "auto", "truck", "foot", etc were specified.
   * @return  Returns true if access was specified.
   */
  bool tagged_access() const {
    return tagged_access_;
  }

  /**
   * Sets the flag indicating if the access information was specified.
   * True if any tags like "access", "auto", "truck", "foot", etc were specified.
   * @param  tagged_access True if any access was set for the node.
   */
  void set_tagged_access(const bool tagged_access);

  /**
   * Is access set as private?
   * @return  Returns true if node access is private.
   */
  bool private_access() const {
    return private_access_;
  }

  /**
   * Sets private_access flag. It is true when access is private for all travel modes.
   * @param  private_access  True if node access is private.
   */
  void set_private_access(const bool private_access) {
    private_access_ = private_access;
  }

  /**
   * Is this node a cash only toll (booth/barrier)?
   * @return  Returns true if node is a cash only toll (booth/barrier).
   */
  bool cash_only_toll() const {
    return cash_only_toll_;
  }

  /**
   * Sets cash_only_toll flag. It is true when the node is a cash only toll (booth/barrier).
   * @param  cash_only_toll  True if node is a cash only toll (booth/barrier).
   */
  void set_cash_only_toll(const bool cash_only_toll) {
    cash_only_toll_ = cash_only_toll;
  }

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
   * Is this a named intersection?
   * @return  Returns true if this is a named intersection.
   */
  bool named_intersection() const {
    return named_;
  }

  /**
   * Sets the flag indicating if this is a named intersection.
   * @param  named  named flag.
   */
  void set_named_intersection(const bool named);

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
   * Gets the transit stop index. This is used for schedule lookups.
   * NOTE - this uses the transition_index_ field which is not used for transit level
   * data. Transit stops connect to the road network with Transit Connection edges.
   * @return  Returns the transit stop index.
   */
  uint32_t stop_index() const {
    return transition_index_;
  }

  /**
   * Set the transit stop index.
   * @param  stop_index  transit stop index.
   */
  void set_stop_index(const uint32_t stop_index);

  /**
   * Get the connecting way id for a transit stop (stored in headings_ while transit data
   * is connected to the road network). Returns 0 if unset or if used for lon lat
   * @return Returns the connecting way id for a transit stop.
   */
  uint64_t connecting_wayid() const {
    // if the last bit is unset this is a wayid (or unset or headings) for transit in/egress. we
    // return 0 for the way id if a connection point (lon, lat) was encoded here instead
    return headings_ >> 63 ? 0 : headings_;
  }

  /**
   * Set the connecting way id for a transit stop.
   * @param  wayid  Connecting wayid.
   */
  void set_connecting_wayid(const uint64_t wayid);

  /**
   * Get the connection point location to be used for associating this transit station to the road
   * network or an invalid point if it is unset
   * @return the connection point or an invalid lon lat
   */
  midgard::PointLL connecting_point() const {
    // if the last bit is set this is a connection point for transit in/egress
    return headings_ >> 63 ? midgard::PointLL(headings_) : midgard::PointLL();
  }

  /**
   * Sets the connection point location to be used for associating this transit in/egress to the road
   * network
   * @param p the location where the in/egress should connect to the road network
   */
  void set_connecting_point(const midgard::PointLL& p);

  /**
   * Get the heading of the local edge given its local index. Supports
   * up to 8 local edges. Headings are stored rounded off to 2 degree
   * values.
   * @param  localidx  Local edge index.
   * @return Returns heading relative to N (0-360 degrees).
   */
  inline uint32_t heading(const uint32_t localidx) const {
    if (localidx > kMaxLocalEdgeIndex) {
      LOG_WARN("Local index " + std::to_string(localidx) + " exceeds max value of " +
               std::to_string(kMaxLocalEdgeIndex) + ", returning heading of 0");
      return 0;
    }
    // Make sure everything is 64 bit!
    uint64_t shift = localidx * 8; // 8 bits per index
    return static_cast<uint32_t>(std::round(
        ((headings_ & (static_cast<uint64_t>(255) << shift)) >> shift) * kHeadingExpandFactor));
  }

  /**
   * Set the heading of the local edge given its local index. Supports
   * up to 8 local edges. Headings are reduced to 8 bits.
   * @param  localidx  Local edge index.
   * @param  heading   Heading relative to N (0-359 degrees).
   */
  void set_heading(uint32_t localidx, uint32_t heading);

  /**
   * Return the index of the first transition from this node.
   * @return  Returns the transition index.
   */
  uint32_t transition_index() const {
    return transition_index_;
  }

  /**
   * Return the index of the first transition from this node.
   * @return  Returns the transition index.
   */
  void set_transition_index(const uint32_t index) {
    transition_index_ = index;
  }

  /**
   * Return the number of transitions from this node.
   * @return  Returns the transition count.
   */
  uint32_t transition_count() const {
    return transition_count_;
  }

  /**
   * Return the number of transitions from this node.
   * @return  Returns the transition count.
   */
  void set_transition_count(const uint32_t count) {
    transition_count_ = count;
  }

  /**
   * Returns the json representation of the object
   * @param tile the tile required to get admin information
   * @return  json object
   */
  json::MapPtr json(const graph_tile_ptr& tile) const;

protected:
  // Organized into 8-byte words so structure will align to 8 byte boundaries.

  // 26 bits for lat,lon offset allows 7 digits of precision even in 4 degree tiles
  // to stay backwards compatible we have to break 6 digits and the 7th digit into two parts
  uint64_t lat_offset_ : 22; // Latitude offset from tile base latitude in int 6 digit precision
  uint64_t lat_offset7_ : 4; // Latitude offset 7th digit of precision
  uint64_t lon_offset_ : 22; // Longitude offset from tile base longitude in int 6 digit precision
  uint64_t lon_offset7_ : 4; // Longitude offset 7th digit of precision
  uint64_t access_ : 12;     // Access through the node - bit field

  uint64_t edge_index_ : 21;    // Index within the node's tile of its first outbound directed edge
  uint64_t edge_count_ : 7;     // Number of outbound edges (on this level)
  uint64_t admin_index_ : 12;   // Index into this tile's administrative information list
  uint64_t timezone_ : 9;       // Time zone
  uint64_t intersection_ : 4;   // Intersection type (see graphconstants.h)
  uint64_t type_ : 4;           // NodeType (see graphconstants.h)
  uint64_t density_ : 4;        // Relative road density
  uint64_t traffic_signal_ : 1; // Traffic signal
  uint64_t mode_change_ : 1;    // Mode change allowed?
  uint64_t named_ : 1;          // Is this a named intersection?

  uint64_t transition_index_ : 21;   // Index into the node transitions to the first transition
                                     // (used to store transit stop index for transit level)
  uint64_t transition_count_ : 3;    // Number of transitions from this node
  uint64_t local_driveability_ : 16; // Driveability for regular edges (up to
                                     // kMaxLocalEdgeIndex+1 edges)
  uint64_t local_edge_count_ : 3;    // # of regular edges across all levels
                                     // (up to kMaxLocalEdgeIndex+1)
  uint64_t drive_on_right_ : 1;      // Driving side. Right if true (false=left)
  uint64_t tagged_access_ : 1;       // Was access initially tagged?
  uint64_t private_access_ : 1;      // Is the access private?
  uint64_t cash_only_toll_ : 1;      // Is this toll cash only?
  uint64_t spare2_ : 17;

  // For not transit levels its the headings of up to kMaxLocalEdgeIndex+1 local edges (rounded to
  // nearest 2 degrees)for all other levels.
  // Sadly we need to keep this for now because its used in map matching, otherwise we could remove it
  // Also for transit levels (while building data only) it can be used for either the connecting way
  // id for matching the connection point of the station to the edge or an encoded lon lat pair for
  // the exact connection point. If the highest bit is set its a lon lat otherwise its a way id
  uint64_t headings_;
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_NODEINFO_H_
