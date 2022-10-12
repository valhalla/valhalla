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
  uint64_t traffic_signal_ : 1;
  uint64_t forward_signal_ : 1;
  uint64_t backward_signal_ : 1;
  uint64_t stop_sign_ : 1;
  uint64_t forward_stop_ : 1;
  uint64_t backward_stop_ : 1;
  uint64_t yield_sign_ : 1;
  uint64_t forward_yield_ : 1;
  uint64_t backward_yield_ : 1;
  uint64_t minor_ : 1;
  uint64_t direction_ : 1;
  uint64_t spare_ : 11;

  uint32_t access_ : 12;
  uint32_t type_ : 4;
  uint32_t intersection_ : 1;
  uint32_t non_link_edge_ : 1;
  uint32_t link_edge_ : 1;
  uint32_t shortlink_ : 1; // Link edge < kMaxInternalLength
  uint32_t non_ferry_edge_ : 1;
  uint32_t ferry_edge_ : 1;
  uint32_t flat_loop_ : 1; // A node which on a section of a way that is doubled back on itself
  uint32_t urban_ : 1;
  uint32_t tagged_access_ : 1; // Was access originally tagged?
  uint32_t private_access_ : 1;
  uint32_t cash_only_toll_ : 1;
  uint32_t spare1_ : 5;

  // pronunciations
  uint32_t name_pronunciation_ipa_index_;
  uint32_t name_pronunciation_nt_sampa_index_;
  uint32_t name_pronunciation_katakana_index_;
  uint32_t name_pronunciation_jeita_index_;
  uint32_t ref_pronunciation_ipa_index_;
  uint32_t ref_pronunciation_nt_sampa_index_;
  uint32_t ref_pronunciation_katakana_index_;
  uint32_t ref_pronunciation_jeita_index_;

  // bss information
  uint32_t bss_info_;

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
  void set_latlng(double lng, double lat) {
    lng = std::round((lng + 180) * 1e7);
    lng7_ = (lng >= 0 && lng <= 360 * 1e7) ? lng : std::numeric_limits<uint32_t>::max();

    lat = std::round((lat + 90) * 1e7);
    lat7_ = (lat >= 0 && lat <= 180 * 1e7) ? lat : std::numeric_limits<uint32_t>::max();
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
   * Set stop sign flag.
   */
  void set_stop_sign(const bool sign) {
    stop_sign_ = sign;
  }

  /**
   * Get the stop sign flag.
   */
  bool stop_sign() const {
    return stop_sign_;
  }

  /**
   * Set forward_stop flag.
   */
  void set_forward_stop(const bool forward_stop) {
    forward_stop_ = forward_stop;
  }

  /**
   * Get the forward_stop flag.
   */
  bool forward_stop() const {
    return forward_stop_;
  }

  /**
   * Set backward_stop flag.
   */
  void set_backward_stop(const bool backward_stop) {
    backward_stop_ = backward_stop;
  }

  /**
   * Get the backward_stop flag.
   */
  bool backward_stop() const {
    return backward_stop_;
  }

  /**
   * Set yield sign flag.
   */
  void set_yield_sign(const bool sign) {
    yield_sign_ = sign;
  }

  /**
   * Get the yield sign flag.
   */
  bool yield_sign() const {
    return yield_sign_;
  }

  /**
   * Set forward_yield flag.
   */
  void set_forward_yield(const bool forward_yield) {
    forward_yield_ = forward_yield;
  }

  /**
   * Get the forward_yield flag.
   */
  bool forward_yield() const {
    return forward_yield_;
  }

  /**
   * Set backward_yield flag.
   */
  void set_backward_yield(const bool backward_yield) {
    backward_yield_ = backward_yield;
  }

  /**
   * Get the backward_yield flag.
   */
  bool backward_yield() const {
    return backward_yield_;
  }

  /**
   * Set minor flag.
   */
  void set_minor(const bool minor) {
    minor_ = minor;
  }

  /**
   * Get the minor flag.
   */
  bool minor() const {
    return minor_;
  }

  /**
   * Set direction flag.
   */
  void set_direction(const bool direction) {
    direction_ = direction;
  }

  /**
   * Get the direction flag.
   */
  bool direction() const {
    return direction_;
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
   * Sets the index for the ref ipa pronunciation
   * @param  idx  Index for the reference ipa pronunciation.
   */
  void set_ref_pronunciation_ipa_index(const uint32_t idx) {
    ref_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the ref ipa pronunciation index.
   * @return  Returns the index for the ref ipa pronunciation.
   */
  uint32_t ref_pronunciation_ipa_index() const {
    return ref_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for the ref nt-sampa pronunciation
   * @param  idx  Index for the reference nt-sampa pronunciation.
   */
  void set_ref_pronunciation_nt_sampa_index(const uint32_t idx) {
    ref_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the ref nt-sampa pronunciation index.
   * @return  Returns the index for the ref nt-sampa pronunciation.
   */
  uint32_t ref_pronunciation_nt_sampa_index() const {
    return ref_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for the ref katakana pronunciation
   * @param  idx  Index for the reference katakana pronunciation.
   */
  void set_ref_pronunciation_katakana_index(const uint32_t idx) {
    ref_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the ref katakana pronunciation index.
   * @return  Returns the index for the ref katakana pronunciation.
   */
  uint32_t ref_pronunciation_katakana_index() const {
    return ref_pronunciation_katakana_index_;
  }

  /**
   * Sets the index for the ref jeita pronunciation
   * @param  idx  Index for the reference jeita pronunciation.
   */
  void set_ref_pronunciation_jeita_index(const uint32_t idx) {
    ref_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the ref jeita pronunciation index.
   * @return  Returns the index for the ref jeita pronunciation.
   */
  uint32_t ref_pronunciation_jeita_index() const {
    return ref_pronunciation_jeita_index_;
  }

  /**
   * Sets the index for name ipa pronunciation
   * @param  idx  Index for the name ipa pronunciation.
   */
  void set_name_pronunciation_ipa_index(const uint32_t idx) {
    name_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the name ipa pronunciation index.
   * @return  Returns the index for the name ipa pronunciation.
   */
  uint32_t name_pronunciation_ipa_index() const {
    return name_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for name nt-sampa pronunciation
   * @param  idx  Index for the name nt-sampa pronunciation.
   */
  void set_name_pronunciation_nt_sampa_index(const uint32_t idx) {
    name_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the name nt-sampa pronunciation index.
   * @return  Returns the index for the name nt-sampa pronunciation.
   */
  uint32_t name_pronunciation_nt_sampa_index() const {
    return name_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for name katakana pronunciation
   * @param  idx  Index for the name katakana pronunciation.
   */
  void set_name_pronunciation_katakana_index(const uint32_t idx) {
    name_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the name katakana pronunciation index.
   * @return  Returns the index for the name katakana pronunciation.
   */
  uint32_t name_pronunciation_katakana_index() const {
    return name_pronunciation_katakana_index_;
  }

  /**
   * Sets the index for name jeita pronunciation
   * @param  idx  Index for the name jeita pronunciation.
   */
  void set_name_pronunciation_jeita_index(const uint32_t idx) {
    name_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the name jeita pronunciation index.
   * @return  Returns the index for the name jeita pronunciation.
   */
  uint32_t name_pronunciation_jeita_index() const {
    return name_pronunciation_jeita_index_;
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

  /**
   * Set the cash_only_toll flag.
   * @param  cash_only_toll bool.
   */
  void set_cash_only_toll(const bool cash_only_toll) {
    cash_only_toll_ = cash_only_toll;
  }

  /**
   * Get the cash_only_toll flag.
   * @return  Returns cash_only_toll flag.
   */
  bool cash_only_toll() const {
    return cash_only_toll_;
  }

  /**
   * Sets the index for bss informations.
   * @param  idx  Index for the bss informations.
   */
  void set_bss_info_index(const uint32_t index) {
    if (index > kMaxNodeNameIndex) {
      throw std::runtime_error("OSMNode: exceeded maximum bss informations index");
    }
    bss_info_ = index;
  }

  /**
   * Get the bss informations index.
   * @return  Returns the index for the bss informations.
   */
  uint32_t bss_info_index() const {
    return bss_info_;
  }
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_OSMNODE_H
