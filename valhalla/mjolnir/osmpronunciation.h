#ifndef VALHALLA_MJOLNIR_PBFGRAPHBUILDER_OSMPRONUNCIATION_H
#define VALHALLA_MJOLNIR_PBFGRAPHBUILDER_OSMPRONUNCIATION_H

#include <cstdint>
#include <string>
#include <vector>

#include <valhalla/baldr/graphconstants.h>

namespace valhalla {
namespace mjolnir {

// OSM Pronunciation.  IPA/X-Sampa/Katakana phonems/pronunciations
struct OSMPronunciation {

  /**
   * Constructor
   */
  OSMPronunciation() {
    memset(this, 0, sizeof(OSMPronunciation));
  }

  /**
   * Constructor with way id arg.
   * @param   id  way id
   */
  OSMPronunciation(const uint64_t id) {
    memset(this, 0, sizeof(OSMPronunciation));
    set_way_id(id);
  }

  /**
   * Set way id.
   * @param   id  way id
   */
  void set_way_id(const uint64_t id) {
    osmwayid_ = id;
  }

  /**
   * Get the way id
   * @return  Returns way id.
   */
  uint64_t way_id() const {
    return osmwayid_;
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
   * Sets the index for int ret ipa pronunciation
   * @param  idx  Index for the international reference ipa pronunciation.
   */
  void set_int_ref_pronunciation_ipa_index(const uint32_t idx) {
    int_ref_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the int ref ipa pronunciation index.
   * @return  Returns the index for the int ref ipa pronunciation.
   */
  uint32_t int_ref_pronunciation_ipa_index() const {
    return int_ref_pronunciation_ipa_index_;
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
   * Sets the index for name:en ipa pronunciation
   * @param  idx  Index for the English name ipa pronunciation.
   */
  void set_name_en_pronunciation_ipa_index(const uint32_t idx) {
    name_en_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the name:en ipa pronunciation index.
   * @return  Returns the index for the English name ipa pronunciation.
   */
  uint32_t name_en_pronunciation_ipa_index() const {
    return name_en_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for alt name ipa pronunciation
   * @param  idx  Index for the alt name ipa pronunciation.
   */
  void set_alt_name_pronunciation_ipa_index(const uint32_t idx) {
    alt_name_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the alt name ipa pronunciation index.
   * @return  Returns the index for the alt name ipa pronunciation.
   */
  uint32_t alt_name_pronunciation_ipa_index() const {
    return alt_name_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for official name ipa pronunciation
   * @param  idx  Index for the official name ipa pronunciation.
   */
  void set_official_name_pronunciation_ipa_index(const uint32_t idx) {
    official_name_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the official name ipa pronunciation index.
   * @return  Returns the index for the official name ipa pronunciation.
   */
  uint32_t official_name_pronunciation_ipa_index() const {
    return official_name_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for tunnel name ipa pronunciation
   * @param  idx  Index for the tunnel name ipa pronunciation.
   */
  void set_tunnel_name_pronunciation_ipa_index(const uint32_t idx) {
    tunnel_name_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the tunnel name ipa pronunciation index.
   * @return  Returns the index for the tunnel name ipa pronunciation.
   */
  uint32_t tunnel_name_pronunciation_ipa_index() const {
    return tunnel_name_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for destination ipa pronunciation.
   * @param  idx  Index for the destination ipa pronunciation.
   */
  void set_destination_pronunciation_ipa_index(const uint32_t idx) {
    destination_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the get_destination ipa pronunciation index.
   * @return  Returns the index for the destination ipa pronunciation.
   */
  uint32_t destination_pronunciation_ipa_index() const {
    return destination_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for destination in forward direction ipa pronunciation.
   * @param  idx  Index for the destination ipa pronunciation.
   */
  void set_destination_forward_pronunciation_ipa_index(const uint32_t idx) {
    destination_forward_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the forward direction ipa pronunciation index.
   * @return  Returns the index for the forward direction ipa pronunciation.
   */
  uint32_t destination_forward_pronunciation_ipa_index() const {
    return destination_forward_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for destination in backward direction ipa pronunciation.
   * @param  idx  Index for the backward direction ipa pronunciation.
   */
  void set_destination_backward_pronunciation_ipa_index(const uint32_t idx) {
    destination_backward_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the backward direction ipa pronunciation index.
   * @return  Returns the index for the backward direction ipa pronunciation.
   */
  uint32_t destination_backward_pronunciation_ipa_index() const {
    return destination_backward_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for destination ref ipa pronunciation.
   * @param  idx  Index for the destination ref ipa pronunciation.
   */
  void set_destination_ref_pronunciation_ipa_index(const uint32_t idx) {
    destination_ref_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the destination ref ipa pronunciation index.
   * @return  Returns the index for the destination ref ipa pronunciation.
   */
  uint32_t destination_ref_pronunciation_ipa_index() const {
    return destination_ref_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for destination ref to ipa pronunciation.
   * @param  idx  Index for the destination ref to ipa pronunciation.
   */
  void set_destination_ref_to_pronunciation_ipa_index(const uint32_t idx) {
    destination_ref_to_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the destination ref to ipa pronunciation index.
   * @return  Returns the index for the destination ref to ipa pronunciation.
   */
  uint32_t destination_ref_to_pronunciation_ipa_index() const {
    return destination_ref_to_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for destination street ipa pronunciation.
   * @param  idx  Index for the destination street ipa pronunciation.
   */
  void set_destination_street_pronunciation_ipa_index(const uint32_t idx) {
    destination_street_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the destination_street ipa pronunciation index.
   * @return  Returns the index for the destination street ipa pronunciation.
   */
  uint32_t destination_street_pronunciation_ipa_index() const {
    return destination_street_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for destination street to ipa pronunciation.
   * @param  idx  Index for the destination street to ipa pronunciation.
   */
  void set_destination_street_to_pronunciation_ipa_index(const uint32_t idx) {
    destination_street_to_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the destination street to ipa pronunciation index.
   * @return  Returns the index for the destination street to ipa pronunciation.
   */
  uint32_t destination_street_to_pronunciation_ipa_index() const {
    return destination_street_to_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for junction ref ipa pronunciation.
   * @param  idx  Index for the junction ref ipa pronunciation.
   */
  void set_junction_ref_pronunciation_ipa_index(const uint32_t idx) {
    junction_ref_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the junction ref ipa pronunciation index.
   * @return  Returns the ipa pronunciation index for the junction ref.
   */
  uint32_t junction_ref_pronunciation_ipa_index() const {
    return junction_ref_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for the ref x-sampa pronunciation
   * @param  idx  Index for the reference x-sampa pronunciation.
   */
  void set_ref_pronunciation_x_sampa_index(const uint32_t idx) {
    ref_pronunciation_x_sampa_index_ = idx;
  }

  /**
   * Get the ref x-sampa pronunciation index.
   * @return  Returns the index for the ref x-sampa pronunciation.
   */
  uint32_t ref_pronunciation_x_sampa_index() const {
    return ref_pronunciation_x_sampa_index_;
  }

  /**
   * Sets the index for int ret x-sampa pronunciation
   * @param  idx  Index for the international reference x-sampa pronunciation.
   */
  void set_int_ref_pronunciation_x_sampa_index(const uint32_t idx) {
    int_ref_pronunciation_x_sampa_index_ = idx;
  }

  /**
   * Get the int ref x-sampa pronunciation index.
   * @return  Returns the index for the int ref x-sampa pronunciation.
   */
  uint32_t int_ref_pronunciation_x_sampa_index() const {
    return int_ref_pronunciation_x_sampa_index_;
  }

  /**
   * Sets the index for name x-sampa pronunciation
   * @param  idx  Index for the name x-sampa pronunciation.
   */
  void set_name_pronunciation_x_sampa_index(const uint32_t idx) {
    name_pronunciation_x_sampa_index_ = idx;
  }

  /**
   * Get the name x-sampa pronunciation index.
   * @return  Returns the index for the name x-sampa pronunciation.
   */
  uint32_t name_pronunciation_x_sampa_index() const {
    return name_pronunciation_x_sampa_index_;
  }

  /**
   * Sets the index for name:en x-sampa pronunciation
   * @param  idx  Index for the English name x-sampa pronunciation.
   */
  void set_name_en_pronunciation_x_sampa_index(const uint32_t idx) {
    name_en_pronunciation_x_sampa_index_ = idx;
  }

  /**
   * Get the name:en x-sampa pronunciation index.
   * @return  Returns the index for the English name x-sampa pronunciation.
   */
  uint32_t name_en_pronunciation_x_sampa_index() const {
    return name_en_pronunciation_x_sampa_index_;
  }

  /**
   * Sets the index for alt name x-sampa pronunciation
   * @param  idx  Index for the alt name x-sampa pronunciation.
   */
  void set_alt_name_pronunciation_x_sampa_index(const uint32_t idx) {
    alt_name_pronunciation_x_sampa_index_ = idx;
  }

  /**
   * Get the alt name x-sampa pronunciation index.
   * @return  Returns the index for the alt name x-sampa pronunciation.
   */
  uint32_t alt_name_pronunciation_x_sampa_index() const {
    return alt_name_pronunciation_x_sampa_index_;
  }

  /**
   * Sets the index for official name x-sampa pronunciation
   * @param  idx  Index for the official name x-sampa pronunciation.
   */
  void set_official_name_pronunciation_x_sampa_index(const uint32_t idx) {
    official_name_pronunciation_x_sampa_index_ = idx;
  }

  /**
   * Get the official name x-sampa pronunciation index.
   * @return  Returns the index for the official name x-sampa pronunciation.
   */
  uint32_t official_name_pronunciation_x_sampa_index() const {
    return official_name_pronunciation_x_sampa_index_;
  }

  /**
   * Sets the index for tunnel name x-sampa pronunciation
   * @param  idx  Index for the tunnel name x-sampa pronunciation.
   */
  void set_tunnel_name_pronunciation_x_sampa_index(const uint32_t idx) {
    tunnel_name_pronunciation_x_sampa_index_ = idx;
  }

  /**
   * Get the tunnel name x-sampa pronunciation index.
   * @return  Returns the index for the tunnel name x-sampa pronunciation.
   */
  uint32_t tunnel_name_pronunciation_x_sampa_index() const {
    return tunnel_name_pronunciation_x_sampa_index_;
  }

  /**
   * Sets the index for destination x-sampa pronunciation.
   * @param  idx  Index for the destination x-sampa pronunciation.
   */
  void set_destination_pronunciation_x_sampa_index(const uint32_t idx) {
    destination_pronunciation_x_sampa_index_ = idx;
  }

  /**
   * Get the get_destination x-sampa pronunciation index.
   * @return  Returns the index for the destination x-sampa pronunciation.
   */
  uint32_t destination_pronunciation_x_sampa_index() const {
    return destination_pronunciation_x_sampa_index_;
  }

  /**
   * Sets the index for destination in forward direction x-sampa pronunciation.
   * @param  idx  Index for the destination x-sampa pronunciation.
   */
  void set_destination_forward_pronunciation_x_sampa_index(const uint32_t idx) {
    destination_forward_pronunciation_x_sampa_index_ = idx;
  }

  /**
   * Get the forward direction x-sampa pronunciation index.
   * @return  Returns the index for the forward direction x-sampa pronunciation.
   */
  uint32_t destination_forward_pronunciation_x_sampa_index() const {
    return destination_forward_pronunciation_x_sampa_index_;
  }

  /**
   * Sets the index for destination in backward direction x-sampa pronunciation.
   * @param  idx  Index for the backward direction x-sampa pronunciation.
   */
  void set_destination_backward_pronunciation_x_sampa_index(const uint32_t idx) {
    destination_backward_pronunciation_x_sampa_index_ = idx;
  }

  /**
   * Get the backward direction x-sampa pronunciation index.
   * @return  Returns the index for the backward direction x-sampa pronunciation.
   */
  uint32_t destination_backward_pronunciation_x_sampa_index() const {
    return destination_backward_pronunciation_x_sampa_index_;
  }

  /**
   * Sets the index for destination ref x-sampa pronunciation.
   * @param  idx  Index for the destination ref x-sampa pronunciation.
   */
  void set_destination_ref_pronunciation_x_sampa_index(const uint32_t idx) {
    destination_ref_pronunciation_x_sampa_index_ = idx;
  }

  /**
   * Get the destination ref x-sampa pronunciation index.
   * @return  Returns the index for the destination ref x-sampa pronunciation.
   */
  uint32_t destination_ref_pronunciation_x_sampa_index() const {
    return destination_ref_pronunciation_x_sampa_index_;
  }

  /**
   * Sets the index for destination ref to x-sampa pronunciation.
   * @param  idx  Index for the destination ref to x-sampa pronunciation.
   */
  void set_destination_ref_to_pronunciation_x_sampa_index(const uint32_t idx) {
    destination_ref_to_pronunciation_x_sampa_index_ = idx;
  }

  /**
   * Get the destination ref to x-sampa pronunciation index.
   * @return  Returns the index for the destination ref to x-sampa pronunciation.
   */
  uint32_t destination_ref_to_pronunciation_x_sampa_index() const {
    return destination_ref_to_pronunciation_x_sampa_index_;
  }

  /**
   * Sets the index for destination street x-sampa pronunciation.
   * @param  idx  Index for the destination street x-sampa pronunciation.
   */
  void set_destination_street_pronunciation_x_sampa_index(const uint32_t idx) {
    destination_street_pronunciation_x_sampa_index_ = idx;
  }

  /**
   * Get the destination_street x-sampa pronunciation index.
   * @return  Returns the index for the destination street x-sampa pronunciation.
   */
  uint32_t destination_street_pronunciation_x_sampa_index() const {
    return destination_street_pronunciation_x_sampa_index_;
  }

  /**
   * Sets the index for destination street to x-sampa pronunciation.
   * @param  idx  Index for the destination street to x-sampa pronunciation.
   */
  void set_destination_street_to_pronunciation_x_sampa_index(const uint32_t idx) {
    destination_street_to_pronunciation_x_sampa_index_ = idx;
  }

  /**
   * Get the destination street to x-sampa pronunciation index.
   * @return  Returns the index for the destination street to x-sampa pronunciation.
   */
  uint32_t destination_street_to_pronunciation_x_sampa_index() const {
    return destination_street_to_pronunciation_x_sampa_index_;
  }

  /**
   * Sets the index for junction ref x-sampa pronunciation.
   * @param  idx  Index for the junction ref x-sampa pronunciation.
   */
  void set_junction_ref_pronunciation_x_sampa_index(const uint32_t idx) {
    junction_ref_pronunciation_x_sampa_index_ = idx;
  }

  /**
   * Get the junction ref x-sampa pronunciation index.
   * @return  Returns the x-sampa pronunciation index for the junction ref.
   */
  uint32_t junction_ref_pronunciation_x_sampa_index() const {
    return junction_ref_pronunciation_x_sampa_index_;
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
   * Sets the index for int ret katakana pronunciation
   * @param  idx  Index for the international reference katakana pronunciation.
   */
  void set_int_ref_pronunciation_katakana_index(const uint32_t idx) {
    int_ref_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the int ref katakana pronunciation index.
   * @return  Returns the index for the int ref katakana pronunciation.
   */
  uint32_t int_ref_pronunciation_katakana_index() const {
    return int_ref_pronunciation_katakana_index_;
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
   * Sets the index for name:en katakana pronunciation
   * @param  idx  Index for the English name katakana pronunciation.
   */
  void set_name_en_pronunciation_katakana_index(const uint32_t idx) {
    name_en_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the name:en katakana pronunciation index.
   * @return  Returns the index for the English name katakana pronunciation.
   */
  uint32_t name_en_pronunciation_katakana_index() const {
    return name_en_pronunciation_katakana_index_;
  }

  /**
   * Sets the index for alt name katakana pronunciation
   * @param  idx  Index for the alt name katakana pronunciation.
   */
  void set_alt_name_pronunciation_katakana_index(const uint32_t idx) {
    alt_name_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the alt name katakana pronunciation index.
   * @return  Returns the index for the alt name katakana pronunciation.
   */
  uint32_t alt_name_pronunciation_katakana_index() const {
    return alt_name_pronunciation_katakana_index_;
  }

  /**
   * Sets the index for official name katakana pronunciation
   * @param  idx  Index for the official name katakana pronunciation.
   */
  void set_official_name_pronunciation_katakana_index(const uint32_t idx) {
    official_name_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the official name katakana pronunciation index.
   * @return  Returns the index for the official name katakana pronunciation.
   */
  uint32_t official_name_pronunciation_katakana_index() const {
    return official_name_pronunciation_katakana_index_;
  }

  /**
   * Sets the index for tunnel name katakana pronunciation
   * @param  idx  Index for the tunnel name katakana pronunciation.
   */
  void set_tunnel_name_pronunciation_katakana_index(const uint32_t idx) {
    tunnel_name_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the tunnel name katakana pronunciation index.
   * @return  Returns the index for the tunnel name katakana pronunciation.
   */
  uint32_t tunnel_name_pronunciation_katakana_index() const {
    return tunnel_name_pronunciation_katakana_index_;
  }

  /**
   * Sets the index for destination katakana pronunciation.
   * @param  idx  Index for the destination katakana pronunciation.
   */
  void set_destination_pronunciation_katakana_index(const uint32_t idx) {
    destination_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the get_destination katakana pronunciation index.
   * @return  Returns the index for the destination katakana pronunciation.
   */
  uint32_t destination_pronunciation_katakana_index() const {
    return destination_pronunciation_katakana_index_;
  }

  /**
   * Sets the index for destination in forward direction katakana pronunciation.
   * @param  idx  Index for the destination katakana pronunciation.
   */
  void set_destination_forward_pronunciation_katakana_index(const uint32_t idx) {
    destination_forward_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the forward direction katakana pronunciation index.
   * @return  Returns the index for the forward direction katakana pronunciation.
   */
  uint32_t destination_forward_pronunciation_katakana_index() const {
    return destination_forward_pronunciation_katakana_index_;
  }

  /**
   * Sets the index for destination in backward direction katakana pronunciation.
   * @param  idx  Index for the backward direction katakana pronunciation.
   */
  void set_destination_backward_pronunciation_katakana_index(const uint32_t idx) {
    destination_backward_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the backward direction katakana pronunciation index.
   * @return  Returns the index for the backward direction katakana pronunciation.
   */
  uint32_t destination_backward_pronunciation_katakana_index() const {
    return destination_backward_pronunciation_katakana_index_;
  }

  /**
   * Sets the index for destination ref katakana pronunciation.
   * @param  idx  Index for the destination ref katakana pronunciation.
   */
  void set_destination_ref_pronunciation_katakana_index(const uint32_t idx) {
    destination_ref_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the destination ref katakana pronunciation index.
   * @return  Returns the index for the destination ref katakana pronunciation.
   */
  uint32_t destination_ref_pronunciation_katakana_index() const {
    return destination_ref_pronunciation_katakana_index_;
  }

  /**
   * Sets the index for destination ref to katakana pronunciation.
   * @param  idx  Index for the destination ref to katakana pronunciation.
   */
  void set_destination_ref_to_pronunciation_katakana_index(const uint32_t idx) {
    destination_ref_to_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the destination ref to katakana pronunciation index.
   * @return  Returns the index for the destination ref to katakana pronunciation.
   */
  uint32_t destination_ref_to_pronunciation_katakana_index() const {
    return destination_ref_to_pronunciation_katakana_index_;
  }

  /**
   * Sets the index for destination street katakana pronunciation.
   * @param  idx  Index for the destination street katakana pronunciation.
   */
  void set_destination_street_pronunciation_katakana_index(const uint32_t idx) {
    destination_street_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the destination_street katakana pronunciation index.
   * @return  Returns the index for the destination street katakana pronunciation.
   */
  uint32_t destination_street_pronunciation_katakana_index() const {
    return destination_street_pronunciation_katakana_index_;
  }

  /**
   * Sets the index for destination street to katakana pronunciation.
   * @param  idx  Index for the destination street to katakana pronunciation.
   */
  void set_destination_street_to_pronunciation_katakana_index(const uint32_t idx) {
    destination_street_to_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the destination street to katakana pronunciation index.
   * @return  Returns the index for the destination street to katakana pronunciation.
   */
  uint32_t destination_street_to_pronunciation_katakana_index() const {
    return destination_street_to_pronunciation_katakana_index_;
  }

  /**
   * Sets the index for junction ref katakana pronunciation.
   * @param  idx  Index for the junction ref katakana pronunciation.
   */
  void set_junction_ref_pronunciation_katakana_index(const uint32_t idx) {
    junction_ref_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the junction ref katakana pronunciation index.
   * @return  Returns the katakana pronunciation index for the junction ref.
   */
  uint32_t junction_ref_pronunciation_katakana_index() const {
    return junction_ref_pronunciation_katakana_index_;
  }

  // OSM way Id
  uint64_t osmwayid_;

  // name and ref ipa pronunciations
  uint32_t ref_pronunciation_ipa_index_;
  uint32_t int_ref_pronunciation_ipa_index_;
  uint32_t name_pronunciation_ipa_index_;
  uint32_t name_en_pronunciation_ipa_index_;
  uint32_t alt_name_pronunciation_ipa_index_;
  uint32_t official_name_pronunciation_ipa_index_;
  uint32_t tunnel_name_pronunciation_ipa_index_;
  uint32_t direction_pronunciation_ipa_index_;
  uint32_t int_direction_pronunciation_ipa_index_;

  // Sign Destination ipa pronunciations
  uint32_t destination_pronunciation_ipa_index_;
  uint32_t destination_forward_pronunciation_ipa_index_;
  uint32_t destination_backward_pronunciation_ipa_index_;
  uint32_t destination_ref_pronunciation_ipa_index_;
  uint32_t destination_ref_to_pronunciation_ipa_index_;
  uint32_t destination_street_pronunciation_ipa_index_;
  uint32_t destination_street_to_pronunciation_ipa_index_;
  uint32_t junction_ref_pronunciation_ipa_index_;

  // name and ref x-sampa pronunciations
  uint32_t ref_pronunciation_x_sampa_index_;
  uint32_t int_ref_pronunciation_x_sampa_index_;
  uint32_t name_pronunciation_x_sampa_index_;
  uint32_t name_en_pronunciation_x_sampa_index_;
  uint32_t alt_name_pronunciation_x_sampa_index_;
  uint32_t official_name_pronunciation_x_sampa_index_;
  uint32_t tunnel_name_pronunciation_x_sampa_index_;
  uint32_t direction_pronunciation_x_sampa_index_;
  uint32_t int_direction_pronunciation_x_sampa_index_;

  // Sign Destination x-sampa pronunciations
  uint32_t destination_pronunciation_x_sampa_index_;
  uint32_t destination_forward_pronunciation_x_sampa_index_;
  uint32_t destination_backward_pronunciation_x_sampa_index_;
  uint32_t destination_ref_pronunciation_x_sampa_index_;
  uint32_t destination_ref_to_pronunciation_x_sampa_index_;
  uint32_t destination_street_pronunciation_x_sampa_index_;
  uint32_t destination_street_to_pronunciation_x_sampa_index_;
  uint32_t junction_ref_pronunciation_x_sampa_index_;

  // name and ref katakana pronunciations
  uint32_t ref_pronunciation_katakana_index_;
  uint32_t int_ref_pronunciation_katakana_index_;
  uint32_t name_pronunciation_katakana_index_;
  uint32_t name_en_pronunciation_katakana_index_;
  uint32_t alt_name_pronunciation_katakana_index_;
  uint32_t official_name_pronunciation_katakana_index_;
  uint32_t tunnel_name_pronunciation_katakana_index_;
  uint32_t direction_pronunciation_katakana_index_;
  uint32_t int_direction_pronunciation_katakana_index_;

  // Sign Destination katakana pronunciations
  uint32_t destination_pronunciation_katakana_index_;
  uint32_t destination_forward_pronunciation_katakana_index_;
  uint32_t destination_backward_pronunciation_katakana_index_;
  uint32_t destination_ref_pronunciation_katakana_index_;
  uint32_t destination_ref_to_pronunciation_katakana_index_;
  uint32_t destination_street_pronunciation_katakana_index_;
  uint32_t destination_street_to_pronunciation_katakana_index_;
  uint32_t junction_ref_pronunciation_katakana_index_;

  uint32_t spare_; // we have 51 indexes above so added this spare for word alignment
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_PBFGRAPHBUILDER_OSMPRONUNCIATION_H
