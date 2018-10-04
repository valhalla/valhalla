#ifndef VALHALLA_MJOLNIR_OSMNODE_H
#define VALHALLA_MJOLNIR_OSMNODE_H

#include <cstdint>
#include <string>
#include <vector>

#include <valhalla/baldr/graphconstants.h>

namespace valhalla {
namespace mjolnir {

struct NodeAttributes {
  uint32_t access_mask : 12;
  uint32_t type : 4;
  uint32_t exit_to : 1;
  uint32_t ref : 1;
  uint32_t name : 1;
  uint32_t intersection : 1;
  uint32_t traffic_signal : 1;
  uint32_t forward_signal : 1;
  uint32_t backward_signal : 1;
  uint32_t non_link_edge : 1;
  uint32_t link_edge : 1;
  uint32_t shortlink : 1; // Link edge < kMaxInternalLength
  uint32_t non_ferry_edge : 1;
  uint32_t ferry_edge : 1;
  uint32_t spare : 4;
};

/**
 * OSM node information. Result of parsing an OSM node.
 */
struct OSMNode {

  // The osm id of the node
  uint64_t osmid;

  // Lat,lng of the node
  float lng, lat;

  // Node attributes. Shared by OSMNode and GraphBuilder Node.
  NodeAttributes attributes_;

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
   * Set access mask.
   */
  void set_access_mask(const uint32_t access_mask) {
    attributes_.access_mask = access_mask;
  }

  /**
   * Get the access mask.
   */
  uint32_t access_mask() const {
    return attributes_.access_mask;
  }

  /**
   * Sets the type.
   * @param  type
   */
  void set_type(const baldr::NodeType type) {
    attributes_.type = static_cast<uint8_t>(type);
  }

  /**
   * Get the type.
   * @return  Returns the type of node.
   */
  baldr::NodeType type() const {
    return static_cast<baldr::NodeType>(attributes_.type);
  }

  /**
   * Set the exit to flag
   */
  void set_exit_to(const bool exit_to) {
    attributes_.exit_to = exit_to;
  }

  /**
   * Get the exit to flag
   */
  bool exit_to() const {
    return attributes_.exit_to;
  }

  /**
   * Set the ref flag
   */
  void set_ref(const bool ref) {
    attributes_.ref = ref;
  }

  /**
   * Get the ref flag
   */
  bool ref() const {
    return attributes_.ref;
  }

  /**
   * Set the name flag (for some exit information)
   */
  void set_name(const bool name) {
    attributes_.name = name;
  }

  /**
   * Get the name flag
   */
  bool name() const {
    return attributes_.name;
  }

  /**
   * Set the intersection flag.
   * @param  intersection  Is this node part of an intersection? True if
   *                       this node is an end node of more than 1 way.
   */
  void set_intersection(const bool intersection) {
    attributes_.intersection = intersection;
  }

  /**
   * Get the intersection flag
   * @return  Returns true if the node is part of an intersection (end
   *          node of more than 1 way), false if not.
   */
  bool intersection() const {
    return attributes_.intersection;
  }

  /**
   * Set traffic_signal flag.
   */
  void set_traffic_signal(const bool traffic_signal) {
    attributes_.traffic_signal = traffic_signal;
  }

  /**
   * Get the traffic_signal flag.
   */
  bool traffic_signal() const {
    return attributes_.traffic_signal;
  }

  /**
   * Set forward_signal flag.
   */
  void set_forward_signal(const bool forward_signal) {
    attributes_.forward_signal = forward_signal;
  }

  /**
   * Get the forward_signal flag.
   */
  bool forward_signal() const {
    return attributes_.forward_signal;
  }

  /**
   * Set backward_signal flag.
   */
  void set_backward_signal(const bool backward_signal) {
    attributes_.backward_signal = backward_signal;
  }

  /**
   * Get the backward_signal flag.
   */
  bool backward_signal() const {
    return attributes_.backward_signal;
  }
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_OSMNODE_H
