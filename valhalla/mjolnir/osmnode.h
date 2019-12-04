#ifndef VALHALLA_MJOLNIR_OSMNODE_H
#define VALHALLA_MJOLNIR_OSMNODE_H

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <valhalla/baldr/graphconstants.h>

namespace valhalla {
namespace mjolnir {

constexpr uint64_t kMaxOSMNodeId = 68719476735;
constexpr uint32_t kMaxNodeNameIndex = 2097151;

/**
 * OSM node information. Result of parsing an OSM node.
 */
struct OSMNode {
  // The osm id of the node
  uint64_t osmid_ : 36; // Allows up to 64B Ids
  uint64_t access_ : 12;
  uint64_t type_ : 4;
  uint64_t intersection_ : 1;
  uint64_t traffic_signal_ : 1;
  uint64_t forward_signal_ : 1;
  uint64_t backward_signal_ : 1;
  uint64_t non_link_edge_ : 1;
  uint64_t link_edge_ : 1;
  uint64_t shortlink_ : 1; // Link edge < kMaxInternalLength
  uint64_t non_ferry_edge_ : 1;
  uint64_t ferry_edge_ : 1;
  uint64_t spare_ : 3;

  // Lat,lng of the node
  float lng_;
  float lat_;

  // Store node names in a separate list (so they don't require as many indexes)
  uint64_t name_index_ : 21;
  uint64_t ref_index_ : 21;
  uint64_t exit_to_index_ : 21;
  uint64_t spare1_ : 1;

  OSMNode() {
    memset(this, 0, sizeof(OSMNode));
  }

  /**
   * Constructor with OSM node Id
   */
  OSMNode(const uint64_t id) {
    memset(this, 0, sizeof(OSMNode));
    set_id(id);
  }

  /**
   * Sets the OSM node Id. Ensures the Id does not exceed the maximum allowed based on
   * ths OSMNode structure.
   * @param id Node Id.
   */
  void set_id(const uint64_t id) {
    // Check for overflow
    if (id > kMaxOSMNodeId) {
      throw std::runtime_error("OSMNode: exceeded maximum OSM node Id: " + std::to_string(id));
    }
    osmid_ = id;
  }

  /**
   * Sets the lat,lng.
   * @param  lng  Longitude of the node.
   * @param  lat  Latitude of the node.
   *
   */
  void set_latlng(const float lng, const float lat) {
    lng_ = lng;
    lat_ = lat;
  }

  /**
   * Gets the lat,lng.
   * @return   Returns the lat,lng of the node.
   */
  std::pair<float, float> latlng() const {
    return std::make_pair(lng_, lat_);
  }

  /**
   * Set the name index into the unique node names list.
   * @param index Index into OSMData node_names.
   */
  void set_name_index(const uint32_t index) {
    if (index > kMaxNodeNameIndex) {
      throw std::runtime_error("OSMNode: exceeded maximum name index");
    }
    name_index_ = index;
  }

  /**
   * Get the name index into the unique node names list.
   * @return Returns the index into OSMData node_names.
   */
  uint32_t name_index() const {
    return name_index_;
  }

  /**
   * Does the node have a name. Check if name_index is non-zero
   */
  bool has_name() const {
    return name_index_ > 0;
  }

  /**
   * Set the ref index into the unique node names list.
   * @param index Index into OSMData node_names
   */
  void set_ref_index(const uint32_t index) {
    if (index > kMaxNodeNameIndex) {
      throw std::runtime_error("OSMNode: exceeded maximum name index");
    }
    ref_index_ = index;
  }

  /**
   * Get the ref index into the unique node names list.
   * @return Returns the index into OSMData node_names.
   */
  uint32_t ref_index() const {
    return ref_index_;
  }

  /**
   * Does the node have ref information. Checks if exit_ref_index is non-zero
   */
  bool has_ref() const {
    return ref_index_ > 0;
  }

  /**
   * Set the exit_to index into the unique node names list.
   * @param index Index into OSMData node_names
   */
  void set_exit_to_index(const uint32_t index) {
    if (index > kMaxNodeNameIndex) {
      throw std::runtime_error("OSMNode: exceeded maximum name index");
    }
    exit_to_index_ = index;
  }

  /**
   * Get the exit_to index into the unique node names list.
   * @return Returns the index into OSMData node_names.
   */
  uint32_t exit_to_index() const {
    return exit_to_index_;
  }

  /**
   * Does the node have exit_to information. Checks if exit_to_index is non-zero
   */
  bool has_exit_to() const {
    return exit_to_index_ > 0;
  }

  /**
   * Set access mask.
   */
  void set_access(const uint32_t mask) {
    access_ = mask;
  }

  /**
   * Get the access mask.
   */
  uint32_t access() const {
    return access_;
  }

  /**
   * Sets the type.
   * @param  type
   */
  void set_type(const baldr::NodeType type) {
    type_ = static_cast<uint8_t>(type);
  }

  /**
   * Get the type.
   * @return  Returns the type of node.
   */
  baldr::NodeType type() const {
    return static_cast<baldr::NodeType>(type_);
  }

  /**
   * Set the intersection flag.
   * @param  intersection  Is this node part of an intersection? True if
   *                       this node is an end node of more than 1 way.
   */
  void set_intersection(const bool intersection) {
    intersection_ = intersection;
  }

  /**
   * Get the intersection flag
   * @return  Returns true if the node is part of an intersection (end
   *          node of more than 1 way), false if not.
   */
  bool intersection() const {
    return intersection_;
  }

  /**
   * Set traffic_signal flag.
   */
  void set_traffic_signal(const bool traffic_signal) {
    traffic_signal_ = traffic_signal;
  }

  /**
   * Get the traffic_signal flag.
   */
  bool traffic_signal() const {
    return traffic_signal_;
  }

  /**
   * Set forward_signal flag.
   */
  void set_forward_signal(const bool forward_signal) {
    forward_signal_ = forward_signal;
  }

  /**
   * Get the forward_signal flag.
   */
  bool forward_signal() const {
    return forward_signal_;
  }

  /**
   * Set backward_signal flag.
   */
  void set_backward_signal(const bool backward_signal) {
    backward_signal_ = backward_signal;
  }

  /**
   * Get the backward_signal flag.
   */
  bool backward_signal() const {
    return backward_signal_;
  }
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_OSMNODE_H
