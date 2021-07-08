#ifndef VALHALLA_MJOLNIR_OSMNODE_H
#define VALHALLA_MJOLNIR_OSMNODE_H

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <valhalla/baldr/graphconstants.h>
#include <valhalla/midgard/pointll.h>

namespace valhalla {
namespace mjolnir {

constexpr uint32_t kMaxNodeNameIndex = 2097151;

/**
 * OSM node information. Result of parsing an OSM node.
 */
struct OSMNode {
  // The osm id of the node
  uint64_t osmid_;

  // Store node names in a separate list (so they don't require as many indexes)
  uint64_t name_index_ : 21;
  uint64_t ref_index_ : 21;
  uint64_t exit_to_index_ : 21;
  uint64_t named_intersection_ : 1;

  uint64_t country_iso_index_ : 21;
  uint64_t state_iso_index_ : 21;
  uint64_t spare_ : 22;

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
  uint32_t flat_loop_ : 1; // A node which on a section of a way that is doubled back on itself
  uint32_t urban_ : 1;
  uint32_t tagged_access_ : 1; // Was access originally tagged?
  uint64_t private_access_ : 1;
  uint32_t spare1_ : 3;

  // Lat,lng of the node at fixed 7digit precision
  uint32_t lng7_;
  uint32_t lat7_;

  OSMNode() {
    memset(this, 0, sizeof(OSMNode));
  }

  /**
   * Constructor with OSM node Id
   */
  OSMNode(const uint64_t id,
          const double lat = std::numeric_limits<double>::max(),
          const double lng = std::numeric_limits<double>::max()) {
    memset(this, 0, sizeof(OSMNode));
    set_id(id);
    set_latlng(lat, lng);
  }

  /**
   * Sets the OSM node Id. Ensures the Id does not exceed the maximum allowed based on
   * ths OSMNode structure.
   * @param id Node Id.
   */
  void set_id(const uint64_t id) {
    osmid_ = id;
  }

  /**
   * Sets the lat,lng.
   * @param  lng  Longitude of the node.
   * @param  lat  Latitude of the node.
   *
   */
  void set_latlng(const double lng, double lat) {
    lng7_ = lat7_ = std::numeric_limits<uint32_t>::max();
    if (lng >= -180 && lng <= 180)
      lng7_ = std::round((lng + 180) * 1e7);
    if (lat >= -90 && lat <= 90)
      lat7_ = std::round((lat + 90) * 1e7);
  }

  /**
   * Gets the lat,lng.
   * @return   Returns the lat,lng of the node.
   */
  midgard::PointLL latlng() const {
    // if either coord is borked we return invalid pointll
    if (lng7_ == std::numeric_limits<uint32_t>::max() ||
        lat7_ == std::numeric_limits<uint32_t>::max())
      return {};
    return {lng7_ * 1e-7 - 180, lat7_ * 1e-7 - 90};
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

  /**
   * Set the named intersection flag.
   * @param  named  Is this a named intersection?
   */
  void set_named_intersection(const bool named) {
    named_intersection_ = named;
  }

  /**
   * Get the named intersection flag
   * @return  Returns true if the node is a named intersection.
   */
  bool named_intersection() const {
    return named_intersection_;
  }

  /**
   * Sets the urban flag.
   * @param  urban       Urban.
   */
  void set_urban(const bool urban) {
    urban_ = urban;
  }

  /**
   * Get the urban flag.
   * @return  Returns urban flag.
   */
  bool urban() const {
    return urban_;
  }

  /**
   * Set the country iso code index
   * @param country iso code Index into the 2 char Country ISO Code.
   */
  void set_country_iso_index(const uint32_t index) {
    if (index > kMaxNodeNameIndex) {
      throw std::runtime_error("OSMNode: exceeded maximum country iso index");
    }
    country_iso_index_ = index;
  }

  /**
   * Get the country iso code.
   * @return Returns the index into the 2 char Country ISO Code.
   */
  uint32_t country_iso_index() const {
    return country_iso_index_;
  }

  /**
   * Does the node have a 2 char code. Check if country_iso_index is non-zero
   */
  bool has_country_iso() const {
    return country_iso_index_ > 0;
  }

  /**
   * Set the country iso code index
   * @param country iso code Index into the 2 char Country ISO Code.
   */
  void set_state_iso_index(const uint32_t index) {
    if (index > kMaxNodeNameIndex) {
      throw std::runtime_error("OSMNode: exceeded maximum state iso index");
    }
    state_iso_index_ = index;
  }

  /**
   * Get the state iso code.
   * @return Returns the index into the 2 char State ISO Code.
   */
  uint32_t state_iso_index() const {
    return state_iso_index_;
  }

  /**
   * Does the node have a 2 char code. Check if state_iso_index is non-zero
   */
  bool has_state_iso_index() const {
    return state_iso_index_ > 0;
  }

  /**
   * Set the tagged_access flag.
   * @param  tagged_access   Was the access originally tagged? True if
   *         any tags like "access", "auto", "truck", "foot", etc were specified.
   */
  void set_tagged_access(const bool tagged_access) {
    tagged_access_ = tagged_access;
  }

  /**
   * Get the tagged_access flag
   * @return  Returns true if any tags like "access", "auto", "truck", "foot", etc
   *          were specified. False if not.
   */
  bool tagged_access() const {
    return tagged_access_;
  }

  /**
   * Sets the private_access flag.
   * @param  private_access bool.
   */
  void set_private_access(const bool private_access) {
    private_access_ = private_access;
  }

  /**
   * Get the private_access flag.
   * @return  Returns private_access flag.
   */
  bool private_access() const {
    return private_access_;
  }
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_OSMNODE_H
