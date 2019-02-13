#ifndef VALHALLA_MJOLNIR_OSMNODE_H
#define VALHALLA_MJOLNIR_OSMNODE_H

#include <cstdint>
#include <string>
#include <vector>

#include <valhalla/baldr/graphconstants.h>

namespace valhalla {
namespace mjolnir {

/**
 * OSM node information. Result of parsing an OSM node.
 */
struct OSMNode {

  // The osm id of the node
  uint64_t osmid;

  // Lat,lng of the node
  float lng, lat;

  // Index to the node name (if it exists)
  uint32_t name_index_;

  // Index to the node ref (if it exists)
  uint32_t ref_index_;

  // Index to exit_to (if it exists)
  uint32_t exit_to_index_;

  // Node attributes. Shared by OSMNode and GraphBuilder Node.
  uint32_t access_ : 12;
  uint32_t type_ : 4;
  uint32_t intersection_ : 1;
  uint32_t traffic_signal_ : 1;
  uint32_t forward_signal_ : 1;
  uint32_t backward_signal_ : 1;
  uint32_t non_link_edge_ : 1;
  uint32_t link_edge_ : 1;
  uint32_t shortlink_ : 1; // Link edge < kMaxInternalLength
  uint32_t non_ferry_edge_ : 1;
  uint32_t ferry_edge_ : 1;
  uint32_t spare_ : 7;

  /**
   * Sets the lat,lng.
   * @param  ll  Lat,lng of the node.
   */
  void set_latlng(const std::pair<float, float>& ll) {
    lng = ll.first;
    lat = ll.second;
  }

  /**
   * Gets the lat,lng.
   * @return   Returns the lat,lng of the node.
   */
  std::pair<float, float> latlng() const {
    return std::make_pair(lng, lat);
  }

  /**
   * Set the name index.
   */
  void set_name_index(const uint32_t index) {
    name_index_ = index;
  }

  /**
   * Get the name index.
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
   * Set the ref index.
   */
  void set_ref_index(const uint32_t index) {
    ref_index_ = index;
  }

  /**
   * Get the ref index.
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
   * Set the exit_to index.
   */
  void set_exit_to_index(const uint32_t index) {
    exit_to_index_ = index;
  }

  /**
   * Get the exit_to index.
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
