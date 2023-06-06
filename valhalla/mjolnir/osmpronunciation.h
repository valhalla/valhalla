#ifndef VALHALLA_MJOLNIR_PBFGRAPHBUILDER_OSMPRONUNCIATION_H
#define VALHALLA_MJOLNIR_PBFGRAPHBUILDER_OSMPRONUNCIATION_H

#include <cstdint>
#include <string>
#include <vector>

#include <valhalla/baldr/graphconstants.h>

namespace valhalla {
namespace mjolnir {

// OSM Pronunciation.  IPA/Nt-sampa/Katakana/Jeita phonems/pronunciations
struct OSMPronunciation {

  enum class Type : uint8_t {
    kName,
    kNameLeft,
    kNameRight,
    kNameForward,
    kNameBackward,
    kNodeName,
    kAltName,
    kAltNameLeft,
    kAltNameRight,
    kOfficialName,
    kOfficialNameLeft,
    kOfficialNameRight,
    kTunnelName,
    kTunnelNameLeft,
    kTunnelNameRight,
    kRef,
    kRefLeft,
    kRefRight,
    kNodeRef,
    kIntRef,
    kIntRefLeft,
    kIntRefRight,
    kDestination,
    kDestinationForward,
    kDestinationBackward,
    kDestinationRef,
    kDestinationRefTo,
    kDestinationStreet,
    kDestinationStreetTo,
    kJunctionRef,
    kJunctionName
  };
  enum class DiffType : uint8_t { kLeft, kRight, kForward, kBackward };

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
   * Sets the index for the ref ipa lang pronunciation
   * @param  idx  Index for the reference ipa lang pronunciation.
   */
  void set_ref_pronunciation_ipa_lang_index(const uint32_t idx) {
    ref_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the ref ipa lang pronunciation index.
   * @return  Returns the index for the ref ipa lang pronunciation.
   */
  uint32_t ref_pronunciation_ipa_lang_index() const {
    return ref_pronunciation_ipa_lang_index_;
  }

  /**
   * Sets the index for ref left ipa pronunciation
   * @param  idx  Index for the ref left ipa pronunciation.
   */
  void set_ref_left_pronunciation_ipa_index(const uint32_t idx) {
    ref_left_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the ref left ipa pronunciation index.
   * @return  Returns the index for the ref left ipa pronunciation.
   */
  uint32_t ref_left_pronunciation_ipa_index() const {
    return ref_left_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for ref left ipa lang pronunciation
   * @param  idx  Index for the ref left ipa lang pronunciation.
   */
  void set_ref_left_pronunciation_ipa_lang_index(const uint32_t idx) {
    ref_left_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the ref left ipa lang pronunciation index.
   * @return  Returns the index for the ref left ipa lang pronunciation.
   */
  uint32_t ref_left_pronunciation_ipa_lang_index() const {
    return ref_left_pronunciation_ipa_lang_index_;
  }

  /**
   * Sets the index for ref right ipa pronunciation
   * @param  idx  Index for the ref right ipa pronunciation.
   */
  void set_ref_right_pronunciation_ipa_index(const uint32_t idx) {
    ref_right_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the ref right ipa pronunciation index.
   * @return  Returns the index for the ref right ipa pronunciation.
   */
  uint32_t ref_right_pronunciation_ipa_index() const {
    return ref_right_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for ref right ipa lang pronunciation
   * @param  idx  Index for the ref right ipa lang pronunciation.
   */
  void set_ref_right_pronunciation_ipa_lang_index(const uint32_t idx) {
    ref_right_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the ref right ipa lang pronunciation index.
   * @return  Returns the index for the ref right ipa lang pronunciation.
   */
  uint32_t ref_right_pronunciation_ipa_lang_index() const {
    return ref_right_pronunciation_ipa_lang_index_;
  }

  /**
   * Sets the index for int ref ipa pronunciation
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
   * Sets the index for int ref ipa lang pronunciation
   * @param  idx  Index for the international reference ipa lang pronunciation.
   */
  void set_int_ref_pronunciation_ipa_lang_index(const uint32_t idx) {
    int_ref_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the int ref ipa lang pronunciation index.
   * @return  Returns the index for the int ref ipa lang pronunciation.
   */
  uint32_t int_ref_pronunciation_ipa_lang_index() const {
    return int_ref_pronunciation_ipa_lang_index_;
  }

  /**
   * Sets the index for int ref left ipa pronunciation
   * @param  idx  Index for the int ref left ipa pronunciation.
   */
  void set_int_ref_left_pronunciation_ipa_index(const uint32_t idx) {
    int_ref_left_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the int ref left ipa pronunciation index.
   * @return  Returns the index for the int ref left ipa pronunciation.
   */
  uint32_t int_ref_left_pronunciation_ipa_index() const {
    return int_ref_left_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for int ref left ipa lang pronunciation
   * @param  idx  Index for the int ref left ipa lang pronunciation.
   */
  void set_int_ref_left_pronunciation_ipa_lang_index(const uint32_t idx) {
    int_ref_left_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the int ref left ipa lang pronunciation index.
   * @return  Returns the index for the int ref left ipa lang pronunciation.
   */
  uint32_t int_ref_left_pronunciation_ipa_lang_index() const {
    return int_ref_left_pronunciation_ipa_lang_index_;
  }

  /**
   * Sets the index for int ref right ipa pronunciation
   * @param  idx  Index for the int ref right ipa pronunciation.
   */
  void set_int_ref_right_pronunciation_ipa_index(const uint32_t idx) {
    int_ref_right_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the int ref right ipa pronunciation index.
   * @return  Returns the index for the int ref right ipa pronunciation.
   */
  uint32_t int_ref_right_pronunciation_ipa_index() const {
    return int_ref_right_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for int ref right ipa lang pronunciation
   * @param  idx  Index for the int ref right ipa lang pronunciation.
   */
  void set_int_ref_right_pronunciation_ipa_lang_index(const uint32_t idx) {
    int_ref_right_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the int ref right ipa lang pronunciation index.
   * @return  Returns the index for the int ref right ipa lang pronunciation.
   */
  uint32_t int_ref_right_pronunciation_ipa_lang_index() const {
    return int_ref_right_pronunciation_ipa_lang_index_;
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
   * Sets the index for name ipa lang pronunciation
   * @param  idx  Index for the name ipa lang pronunciation.
   */
  void set_name_pronunciation_ipa_lang_index(const uint32_t idx) {
    name_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the name ipa lang pronunciation index.
   * @return  Returns the index for the name ipa lang pronunciation.
   */
  uint32_t name_pronunciation_ipa_lang_index() const {
    return name_pronunciation_ipa_lang_index_;
  }

  /**
   * Sets the index for name left ipa pronunciation
   * @param  idx  Index for the name left ipa pronunciation.
   */
  void set_name_left_pronunciation_ipa_index(const uint32_t idx) {
    name_left_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the name left ipa pronunciation index.
   * @return  Returns the index for the name left ipa pronunciation.
   */
  uint32_t name_left_pronunciation_ipa_index() const {
    return name_left_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for name left ipa lang pronunciation
   * @param  idx  Index for the name left ipa lang pronunciation.
   */
  void set_name_left_pronunciation_ipa_lang_index(const uint32_t idx) {
    name_left_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the name left ipa lang pronunciation index.
   * @return  Returns the index for the name left ipa lang pronunciation.
   */
  uint32_t name_left_pronunciation_ipa_lang_index() const {
    return name_left_pronunciation_ipa_lang_index_;
  }

  /**
   * Sets the index for name right ipa pronunciation
   * @param  idx  Index for the name right ipa pronunciation.
   */
  void set_name_right_pronunciation_ipa_index(const uint32_t idx) {
    name_right_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the name right ipa pronunciation index.
   * @return  Returns the index for the name right ipa pronunciation.
   */
  uint32_t name_right_pronunciation_ipa_index() const {
    return name_right_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for name right ipa lang pronunciation
   * @param  idx  Index for the name right ipa lang pronunciation.
   */
  void set_name_right_pronunciation_ipa_lang_index(const uint32_t idx) {
    name_right_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the name right ipa lang pronunciation index.
   * @return  Returns the index for the name right ipa lang pronunciation.
   */
  uint32_t name_right_pronunciation_ipa_lang_index() const {
    return name_right_pronunciation_ipa_lang_index_;
  }

  /**
   * Sets the index for name forward ipa pronunciation
   * @param  idx  Index for the name forward ipa pronunciation.
   */
  void set_name_forward_pronunciation_ipa_index(const uint32_t idx) {
    name_forward_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the name forward ipa pronunciation index.
   * @return  Returns the index for the name forward ipa pronunciation.
   */
  uint32_t name_forward_pronunciation_ipa_index() const {
    return name_forward_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for name forward ipa lang pronunciation
   * @param  idx  Index for the name forward ipa lang pronunciation.
   */
  void set_name_forward_pronunciation_ipa_lang_index(const uint32_t idx) {
    name_forward_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the name forward ipa lang pronunciation index.
   * @return  Returns the index for the name forward ipa lang pronunciation.
   */
  uint32_t name_forward_pronunciation_ipa_lang_index() const {
    return name_forward_pronunciation_ipa_lang_index_;
  }

  /**
   * Sets the index for name backward ipa pronunciation
   * @param  idx  Index for the name backward ipa pronunciation.
   */
  void set_name_backward_pronunciation_ipa_index(const uint32_t idx) {
    name_backward_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the name backward ipa pronunciation index.
   * @return  Returns the index for the name backward ipa pronunciation.
   */
  uint32_t name_backward_pronunciation_ipa_index() const {
    return name_backward_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for name backward ipa lang pronunciation
   * @param  idx  Index for the name backward ipa lang pronunciation.
   */
  void set_name_backward_pronunciation_ipa_lang_index(const uint32_t idx) {
    name_backward_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the name backward ipa lang pronunciation index.
   * @return  Returns the index for the name backward ipa lang pronunciation.
   */
  uint32_t name_backward_pronunciation_ipa_lang_index() const {
    return name_backward_pronunciation_ipa_lang_index_;
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
   * Sets the index for alt name ipa lang pronunciation
   * @param  idx  Index for the alt name ipa lang pronunciation.
   */
  void set_alt_name_pronunciation_ipa_lang_index(const uint32_t idx) {
    alt_name_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the alt name ipa lang pronunciation index.
   * @return  Returns the index for the alt name ipa lang pronunciation.
   */
  uint32_t alt_name_pronunciation_ipa_lang_index() const {
    return alt_name_pronunciation_ipa_lang_index_;
  }

  /**
   * Sets the index for alt name left ipa pronunciation
   * @param  idx  Index for the alt name left ipa pronunciation.
   */
  void set_alt_name_left_pronunciation_ipa_index(const uint32_t idx) {
    alt_name_left_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the alt name left ipa pronunciation index.
   * @return  Returns the index for the alt name left ipa pronunciation.
   */
  uint32_t alt_name_left_pronunciation_ipa_index() const {
    return alt_name_left_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for alt name left ipa lang pronunciation
   * @param  idx  Index for the alt name left ipa lang pronunciation.
   */
  void set_alt_name_left_pronunciation_ipa_lang_index(const uint32_t idx) {
    alt_name_left_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the alt name left ipa lang pronunciation index.
   * @return  Returns the index for the alt name left ipa lang pronunciation.
   */
  uint32_t alt_name_left_pronunciation_ipa_lang_index() const {
    return alt_name_left_pronunciation_ipa_lang_index_;
  }

  /**
   * Sets the index for alt name right ipa pronunciation
   * @param  idx  Index for the alt name right ipa pronunciation.
   */
  void set_alt_name_right_pronunciation_ipa_index(const uint32_t idx) {
    alt_name_right_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the alt name right ipa pronunciation index.
   * @return  Returns the index for the alt name right ipa pronunciation.
   */
  uint32_t alt_name_right_pronunciation_ipa_index() const {
    return alt_name_right_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for alt name right ipa lang pronunciation
   * @param  idx  Index for the alt name right ipa lang pronunciation.
   */
  void set_alt_name_right_pronunciation_ipa_lang_index(const uint32_t idx) {
    alt_name_right_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the alt name right ipa lang pronunciation index.
   * @return  Returns the index for the alt name right ipa lang pronunciation.
   */
  uint32_t alt_name_right_pronunciation_ipa_lang_index() const {
    return alt_name_right_pronunciation_ipa_lang_index_;
  }

  /**
   * Sets the index for alt name forward ipa pronunciation
   * @param  idx  Index for the alt name forward ipa pronunciation.
   */
  void set_alt_name_forward_pronunciation_ipa_index(const uint32_t idx) {
    alt_name_forward_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the alt name forward ipa pronunciation index.
   * @return  Returns the index for the alt name forward ipa pronunciation.
   */
  uint32_t alt_name_forward_pronunciation_ipa_index() const {
    return alt_name_forward_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for alt name forward ipa lang pronunciation
   * @param  idx  Index for the alt name forward ipa lang pronunciation.
   */
  void set_alt_name_forward_pronunciation_ipa_lang_index(const uint32_t idx) {
    alt_name_forward_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the alt name forward ipa lang pronunciation index.
   * @return  Returns the index for the alt name forward ipa lang pronunciation.
   */
  uint32_t alt_name_forward_pronunciation_ipa_lang_index() const {
    return alt_name_forward_pronunciation_ipa_lang_index_;
  }

  /**
   * Sets the index for alt name backward ipa pronunciation
   * @param  idx  Index for the alt name backward ipa pronunciation.
   */
  void set_alt_name_backward_pronunciation_ipa_index(const uint32_t idx) {
    alt_name_backward_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the alt name backward ipa pronunciation index.
   * @return  Returns the index for the alt name backward ipa pronunciation.
   */
  uint32_t alt_name_backward_pronunciation_ipa_index() const {
    return alt_name_backward_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for alt name backward ipa lang pronunciation
   * @param  idx  Index for the alt name backward ipa lang pronunciation.
   */
  void set_alt_name_backward_pronunciation_ipa_lang_index(const uint32_t idx) {
    alt_name_backward_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the alt name backward ipa lang pronunciation index.
   * @return  Returns the index for the alt name backward ipa lang pronunciation.
   */
  uint32_t alt_name_backward_pronunciation_ipa_lang_index() const {
    return alt_name_backward_pronunciation_ipa_lang_index_;
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
   * Sets the index for official name ipa lang pronunciation
   * @param  idx  Index for the official name ipa lang pronunciation.
   */
  void set_official_name_pronunciation_ipa_lang_index(const uint32_t idx) {
    official_name_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the official name ipa pronunciation lang index.
   * @return  Returns the index for the official name ipa lang pronunciation.
   */
  uint32_t official_name_pronunciation_ipa_lang_index() const {
    return official_name_pronunciation_ipa_lang_index_;
  }

  /**
   * Sets the index for official name left ipa pronunciation
   * @param  idx  Index for the official name left ipa pronunciation.
   */
  void set_official_name_left_pronunciation_ipa_index(const uint32_t idx) {
    official_name_left_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the official name left ipa pronunciation index.
   * @return  Returns the index for the official name left ipa pronunciation.
   */
  uint32_t official_name_left_pronunciation_ipa_index() const {
    return official_name_left_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for official name left ipa lang pronunciation
   * @param  idx  Index for the official name left ipa lang pronunciation.
   */
  void set_official_name_left_pronunciation_ipa_lang_index(const uint32_t idx) {
    official_name_left_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the official name left ipa lang pronunciation index.
   * @return  Returns the index for the official name left ipa lang pronunciation.
   */
  uint32_t official_name_left_pronunciation_ipa_lang_index() const {
    return official_name_left_pronunciation_ipa_lang_index_;
  }

  /**
   * Sets the index for official name right ipa pronunciation
   * @param  idx  Index for the official name right ipa pronunciation.
   */
  void set_official_name_right_pronunciation_ipa_index(const uint32_t idx) {
    official_name_right_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the official name right ipa pronunciation index.
   * @return  Returns the index for the official name right ipa pronunciation.
   */
  uint32_t official_name_right_pronunciation_ipa_index() const {
    return official_name_right_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for official name right ipa lang pronunciation
   * @param  idx  Index for the official name right ipa lang pronunciation.
   */
  void set_official_name_right_pronunciation_ipa_lang_index(const uint32_t idx) {
    official_name_right_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the official name right ipa lang pronunciation index.
   * @return  Returns the index for the official name right ipa lang pronunciation.
   */
  uint32_t official_name_right_pronunciation_ipa_lang_index() const {
    return official_name_right_pronunciation_ipa_lang_index_;
  }

  /**
   * Sets the index for official name forward ipa pronunciation
   * @param  idx  Index for the official name forward ipa pronunciation.
   */
  void set_official_name_forward_pronunciation_ipa_index(const uint32_t idx) {
    official_name_forward_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the official name forward ipa pronunciation index.
   * @return  Returns the index for the official name forward ipa pronunciation.
   */
  uint32_t official_name_forward_pronunciation_ipa_index() const {
    return official_name_forward_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for official name forward ipa lang pronunciation
   * @param  idx  Index for the official name forward ipa lang pronunciation.
   */
  void set_official_name_forward_pronunciation_ipa_lang_index(const uint32_t idx) {
    official_name_forward_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the official name forward ipa lang pronunciation index.
   * @return  Returns the index for the official name forward ipa lang pronunciation.
   */
  uint32_t official_name_forward_pronunciation_ipa_lang_index() const {
    return official_name_forward_pronunciation_ipa_lang_index_;
  }

  /**
   * Sets the index for official name backward ipa pronunciation
   * @param  idx  Index for the official name backward ipa pronunciation.
   */
  void set_official_name_backward_pronunciation_ipa_index(const uint32_t idx) {
    official_name_backward_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the official name backward ipa pronunciation index.
   * @return  Returns the index for the official name backward ipa pronunciation.
   */
  uint32_t official_name_backward_pronunciation_ipa_index() const {
    return official_name_backward_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for official name backward ipa lang pronunciation
   * @param  idx  Index for the official name backward ipa lang pronunciation.
   */
  void set_official_name_backward_pronunciation_ipa_lang_index(const uint32_t idx) {
    official_name_backward_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the official name backward ipa lang pronunciation index.
   * @return  Returns the index for the official name backward ipa lang pronunciation.
   */
  uint32_t official_name_backward_pronunciation_ipa_lang_index() const {
    return official_name_backward_pronunciation_ipa_lang_index_;
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
   * Sets the index for tunnel name ipa lang pronunciation
   * @param  idx  Index for the tunnel name ipa lang pronunciation.
   */
  void set_tunnel_name_pronunciation_ipa_lang_index(const uint32_t idx) {
    tunnel_name_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the tunnel name ipa pronunciation lang index.
   * @return  Returns the index for the tunnel name ipa lang pronunciation.
   */
  uint32_t tunnel_name_pronunciation_ipa_lang_index() const {
    return tunnel_name_pronunciation_ipa_lang_index_;
  }

  /**
   * Sets the index for tunnel name left ipa pronunciation
   * @param  idx  Index for the tunnel name left ipa pronunciation.
   */
  void set_tunnel_name_left_pronunciation_ipa_index(const uint32_t idx) {
    tunnel_name_left_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the tunnel name left ipa pronunciation index.
   * @return  Returns the index for the tunnel name left ipa pronunciation.
   */
  uint32_t tunnel_name_left_pronunciation_ipa_index() const {
    return tunnel_name_left_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for tunnel name left ipa lang pronunciation
   * @param  idx  Index for the tunnel name left ipa lang pronunciation.
   */
  void set_tunnel_name_left_pronunciation_ipa_lang_index(const uint32_t idx) {
    tunnel_name_left_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the tunnel name left ipa lang pronunciation index.
   * @return  Returns the index for the tunnel name left ipa lang pronunciation.
   */
  uint32_t tunnel_name_left_pronunciation_ipa_lang_index() const {
    return tunnel_name_left_pronunciation_ipa_lang_index_;
  }

  /**
   * Sets the index for tunnel name right ipa pronunciation
   * @param  idx  Index for the tunnel name right ipa pronunciation.
   */
  void set_tunnel_name_right_pronunciation_ipa_index(const uint32_t idx) {
    tunnel_name_right_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the tunnel name right ipa pronunciation index.
   * @return  Returns the index for the tunnel name right ipa pronunciation.
   */
  uint32_t tunnel_name_right_pronunciation_ipa_index() const {
    return tunnel_name_right_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for tunnel name right ipa lang pronunciation
   * @param  idx  Index for the tunnel name right ipa lang pronunciation.
   */
  void set_tunnel_name_right_pronunciation_ipa_lang_index(const uint32_t idx) {
    tunnel_name_right_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the tunnel name right ipa lang pronunciation index.
   * @return  Returns the index for the tunnel name right ipa lang pronunciation.
   */
  uint32_t tunnel_name_right_pronunciation_ipa_lang_index() const {
    return tunnel_name_right_pronunciation_ipa_lang_index_;
  }

  /**
   * Sets the index for tunnel name forward ipa pronunciation
   * @param  idx  Index for the tunnel name forward ipa pronunciation.
   */
  void set_tunnel_name_forward_pronunciation_ipa_index(const uint32_t idx) {
    tunnel_name_forward_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the tunnel name forward ipa pronunciation index.
   * @return  Returns the index for the tunnel name forward ipa pronunciation.
   */
  uint32_t tunnel_name_forward_pronunciation_ipa_index() const {
    return tunnel_name_forward_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for tunnel name forward ipa lang pronunciation
   * @param  idx  Index for the tunnel name forward ipa lang pronunciation.
   */
  void set_tunnel_name_forward_pronunciation_ipa_lang_index(const uint32_t idx) {
    tunnel_name_forward_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the tunnel name forward ipa lang pronunciation index.
   * @return  Returns the index for the tunnel name forward ipa lang pronunciation.
   */
  uint32_t tunnel_name_forward_pronunciation_ipa_lang_index() const {
    return tunnel_name_forward_pronunciation_ipa_lang_index_;
  }

  /**
   * Sets the index for tunnel name backward ipa pronunciation
   * @param  idx  Index for the tunnel name backward ipa pronunciation.
   */
  void set_tunnel_name_backward_pronunciation_ipa_index(const uint32_t idx) {
    tunnel_name_backward_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the tunnel name backward ipa pronunciation index.
   * @return  Returns the index for the tunnel name backward ipa pronunciation.
   */
  uint32_t tunnel_name_backward_pronunciation_ipa_index() const {
    return tunnel_name_backward_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for tunnel name backward ipa lang pronunciation
   * @param  idx  Index for the tunnel name backward ipa lang pronunciation.
   */
  void set_tunnel_name_backward_pronunciation_ipa_lang_index(const uint32_t idx) {
    tunnel_name_backward_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the tunnel name backward ipa lang pronunciation index.
   * @return  Returns the index for the tunnel name backward ipa lang pronunciation.
   */
  uint32_t tunnel_name_backward_pronunciation_ipa_lang_index() const {
    return tunnel_name_backward_pronunciation_ipa_lang_index_;
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
   * Sets the index for destination ipa lang pronunciation.
   * @param  idx  Index for the destination ipa lang pronunciation.
   */
  void set_destination_pronunciation_ipa_lang_index(const uint32_t idx) {
    destination_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the get_destination ipa lang pronunciation index.
   * @return  Returns the index for the destination ipa lang pronunciation.
   */
  uint32_t destination_pronunciation_ipa_lang_index() const {
    return destination_pronunciation_ipa_lang_index_;
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
   * Sets the index for destination in forward direction ipa lang pronunciation.
   * @param  idx  Index for the destination ipa lang pronunciation.
   */
  void set_destination_forward_pronunciation_ipa_lang_index(const uint32_t idx) {
    destination_forward_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the forward direction ipa lang pronunciation index.
   * @return  Returns the index for the forward direction ipa lang pronunciation.
   */
  uint32_t destination_forward_pronunciation_ipa_lang_index() const {
    return destination_forward_pronunciation_ipa_lang_index_;
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
   * Sets the index for destination in backward direction ipa lang pronunciation.
   * @param  idx  Index for the backward direction ipa lang pronunciation.
   */
  void set_destination_backward_pronunciation_ipa_lang_index(const uint32_t idx) {
    destination_backward_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the backward direction ipa lang pronunciation index.
   * @return  Returns the index for the backward direction ipa lang pronunciation.
   */
  uint32_t destination_backward_pronunciation_ipa_lang_index() const {
    return destination_backward_pronunciation_ipa_lang_index_;
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
   * Sets the index for destination ref ipa lang pronunciation.
   * @param  idx  Index for the destination ref ipa lang pronunciation.
   */
  void set_destination_ref_pronunciation_ipa_lang_index(const uint32_t idx) {
    destination_ref_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the destination ref ipa lang pronunciation index.
   * @return  Returns the index for the destination ref ipa lang pronunciation.
   */
  uint32_t destination_ref_pronunciation_ipa_lang_index() const {
    return destination_ref_pronunciation_ipa_lang_index_;
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
   * Sets the index for destination ref to ipa lang pronunciation.
   * @param  idx  Index for the destination ref to ipa lang pronunciation.
   */
  void set_destination_ref_to_pronunciation_ipa_lang_index(const uint32_t idx) {
    destination_ref_to_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the destination ref to ipa lang pronunciation index.
   * @return  Returns the index for the destination ref to ipa lang pronunciation.
   */
  uint32_t destination_ref_to_pronunciation_ipa_lang_index() const {
    return destination_ref_to_pronunciation_ipa_lang_index_;
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
   * Sets the index for destination street ipa lang pronunciation.
   * @param  idx  Index for the destination street ipa lang pronunciation.
   */
  void set_destination_street_pronunciation_ipa_lang_index(const uint32_t idx) {
    destination_street_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the destination_street ipa lang pronunciation index.
   * @return  Returns the index for the destination street ipa lang pronunciation.
   */
  uint32_t destination_street_pronunciation_ipa_lang_index() const {
    return destination_street_pronunciation_ipa_lang_index_;
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
   * Sets the index for destination street to ipa lang pronunciation.
   * @param  idx  Index for the destination street to ipa lang pronunciation.
   */
  void set_destination_street_to_pronunciation_ipa_lang_index(const uint32_t idx) {
    destination_street_to_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the destination street to ipa lang pronunciation index.
   * @return  Returns the index for the destination street to ipa lang pronunciation.
   */
  uint32_t destination_street_to_pronunciation_ipa_lang_index() const {
    return destination_street_to_pronunciation_ipa_lang_index_;
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
   * Sets the index for junction ref ipa lang pronunciation.
   * @param  idx  Index for the junction ref ipa lang pronunciation.
   */
  void set_junction_ref_pronunciation_ipa_lang_index(const uint32_t idx) {
    junction_ref_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the junction ref ipa lang pronunciation index.
   * @return  Returns the ipa lang pronunciation index for the junction ref.
   */
  uint32_t junction_ref_pronunciation_ipa_lang_index() const {
    return junction_ref_pronunciation_ipa_lang_index_;
  }

  /**
   * Sets the index for junction name ipa pronunciation.
   * @param  idx  Index for the junction name ipa pronunciation.
   */
  void set_junction_name_pronunciation_ipa_index(const uint32_t idx) {
    junction_name_pronunciation_ipa_index_ = idx;
  }

  /**
   * Get the junction name ipa pronunciation index.
   * @return  Returns the ipa pronunciation index for the junction name.
   */
  uint32_t junction_name_pronunciation_ipa_index() const {
    return junction_name_pronunciation_ipa_index_;
  }

  /**
   * Sets the index for junction name ipa lang pronunciation.
   * @param  idx  Index for the junction name ipa lang pronunciation.
   */
  void set_junction_name_pronunciation_ipa_lang_index(const uint32_t idx) {
    junction_name_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the junction name ipa lang pronunciation index.
   * @return  Returns the ipa lang pronunciation index for the junction name.
   */
  uint32_t junction_name_pronunciation_ipa_lang_index() const {
    return junction_name_pronunciation_ipa_lang_index_;
  }

  /**
   * Sets the index for the ref nt_sampa pronunciation
   * @param  idx  Index for the reference nt_sampa pronunciation.
   */
  void set_ref_pronunciation_nt_sampa_index(const uint32_t idx) {
    ref_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the ref nt_sampa pronunciation index.
   * @return  Returns the index for the ref nt_sampa pronunciation.
   */
  uint32_t ref_pronunciation_nt_sampa_index() const {
    return ref_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for the ref nt_sampa lang pronunciation
   * @param  idx  Index for the reference nt_sampa lang pronunciation.
   */
  void set_ref_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    ref_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the ref nt_sampa lang pronunciation index.
   * @return  Returns the index for the ref nt_sampa lang pronunciation.
   */
  uint32_t ref_pronunciation_nt_sampa_lang_index() const {
    return ref_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for ref left nt_sampa pronunciation
   * @param  idx  Index for the ref left nt_sampa pronunciation.
   */
  void set_ref_left_pronunciation_nt_sampa_index(const uint32_t idx) {
    ref_left_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the ref left nt_sampa pronunciation index.
   * @return  Returns the index for the ref left nt_sampa pronunciation.
   */
  uint32_t ref_left_pronunciation_nt_sampa_index() const {
    return ref_left_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for ref left nt_sampa lang pronunciation
   * @param  idx  Index for the ref left nt_sampa lang pronunciation.
   */
  void set_ref_left_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    ref_left_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the ref left nt_sampa lang pronunciation index.
   * @return  Returns the index for the ref left nt_sampa lang pronunciation.
   */
  uint32_t ref_left_pronunciation_nt_sampa_lang_index() const {
    return ref_left_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for ref right nt_sampa pronunciation
   * @param  idx  Index for the ref right nt_sampa pronunciation.
   */
  void set_ref_right_pronunciation_nt_sampa_index(const uint32_t idx) {
    ref_right_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the ref right nt_sampa pronunciation index.
   * @return  Returns the index for the ref right nt_sampa pronunciation.
   */
  uint32_t ref_right_pronunciation_nt_sampa_index() const {
    return ref_right_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for ref right nt_sampa lang pronunciation
   * @param  idx  Index for the ref right nt_sampa lang pronunciation.
   */
  void set_ref_right_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    ref_right_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the ref right nt_sampa lang pronunciation index.
   * @return  Returns the index for the ref right nt_sampa lang pronunciation.
   */
  uint32_t ref_right_pronunciation_nt_sampa_lang_index() const {
    return ref_right_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for int ref nt_sampa pronunciation
   * @param  idx  Index for the international reference nt_sampa pronunciation.
   */
  void set_int_ref_pronunciation_nt_sampa_index(const uint32_t idx) {
    int_ref_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the int ref nt_sampa pronunciation index.
   * @return  Returns the index for the int ref nt_sampa pronunciation.
   */
  uint32_t int_ref_pronunciation_nt_sampa_index() const {
    return int_ref_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for int ref nt_sampa lang pronunciation
   * @param  idx  Index for the international reference nt_sampa lang pronunciation.
   */
  void set_int_ref_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    int_ref_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the int ref nt_sampa lang pronunciation index.
   * @return  Returns the index for the int ref nt_sampa lang pronunciation.
   */
  uint32_t int_ref_pronunciation_nt_sampa_lang_index() const {
    return int_ref_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for int ref left nt_sampa pronunciation
   * @param  idx  Index for the int ref left nt_sampa pronunciation.
   */
  void set_int_ref_left_pronunciation_nt_sampa_index(const uint32_t idx) {
    int_ref_left_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the int ref left nt_sampa pronunciation index.
   * @return  Returns the index for the int ref left nt_sampa pronunciation.
   */
  uint32_t int_ref_left_pronunciation_nt_sampa_index() const {
    return int_ref_left_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for int ref left nt_sampa lang pronunciation
   * @param  idx  Index for the int ref left nt_sampa lang pronunciation.
   */
  void set_int_ref_left_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    int_ref_left_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the int ref left nt_sampa lang pronunciation index.
   * @return  Returns the index for the int ref left nt_sampa lang pronunciation.
   */
  uint32_t int_ref_left_pronunciation_nt_sampa_lang_index() const {
    return int_ref_left_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for int ref right nt_sampa pronunciation
   * @param  idx  Index for the int ref right nt_sampa pronunciation.
   */
  void set_int_ref_right_pronunciation_nt_sampa_index(const uint32_t idx) {
    int_ref_right_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the int ref right nt_sampa pronunciation index.
   * @return  Returns the index for the int ref right nt_sampa pronunciation.
   */
  uint32_t int_ref_right_pronunciation_nt_sampa_index() const {
    return int_ref_right_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for int ref right nt_sampa lang pronunciation
   * @param  idx  Index for the int ref right nt_sampa lang pronunciation.
   */
  void set_int_ref_right_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    int_ref_right_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the int ref right nt_sampa lang pronunciation index.
   * @return  Returns the index for the int ref right nt_sampa lang pronunciation.
   */
  uint32_t int_ref_right_pronunciation_nt_sampa_lang_index() const {
    return int_ref_right_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for name nt_sampa pronunciation
   * @param  idx  Index for the name nt_sampa pronunciation.
   */
  void set_name_pronunciation_nt_sampa_index(const uint32_t idx) {
    name_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the name nt_sampa pronunciation index.
   * @return  Returns the index for the name nt_sampa pronunciation.
   */
  uint32_t name_pronunciation_nt_sampa_index() const {
    return name_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for name nt_sampa lang pronunciation
   * @param  idx  Index for the name nt_sampa lang pronunciation.
   */
  void set_name_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    name_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the name nt_sampa lang pronunciation index.
   * @return  Returns the index for the name nt_sampa lang pronunciation.
   */
  uint32_t name_pronunciation_nt_sampa_lang_index() const {
    return name_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for name left nt_sampa pronunciation
   * @param  idx  Index for the name left nt_sampa pronunciation.
   */
  void set_name_left_pronunciation_nt_sampa_index(const uint32_t idx) {
    name_left_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the name left nt_sampa pronunciation index.
   * @return  Returns the index for the name left nt_sampa pronunciation.
   */
  uint32_t name_left_pronunciation_nt_sampa_index() const {
    return name_left_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for name left nt_sampa lang pronunciation
   * @param  idx  Index for the name left nt_sampa lang pronunciation.
   */
  void set_name_left_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    name_left_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the name left nt_sampa lang pronunciation index.
   * @return  Returns the index for the name left nt_sampa lang pronunciation.
   */
  uint32_t name_left_pronunciation_nt_sampa_lang_index() const {
    return name_left_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for name right nt_sampa pronunciation
   * @param  idx  Index for the name right nt_sampa pronunciation.
   */
  void set_name_right_pronunciation_nt_sampa_index(const uint32_t idx) {
    name_right_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the name right nt_sampa pronunciation index.
   * @return  Returns the index for the name right nt_sampa pronunciation.
   */
  uint32_t name_right_pronunciation_nt_sampa_index() const {
    return name_right_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for name right nt_sampa lang pronunciation
   * @param  idx  Index for the name right nt_sampa lang pronunciation.
   */
  void set_name_right_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    name_right_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the name right nt_sampa lang pronunciation index.
   * @return  Returns the index for the name right nt_sampa lang pronunciation.
   */
  uint32_t name_right_pronunciation_nt_sampa_lang_index() const {
    return name_right_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for name forward nt_sampa pronunciation
   * @param  idx  Index for the name forward nt_sampa pronunciation.
   */
  void set_name_forward_pronunciation_nt_sampa_index(const uint32_t idx) {
    name_forward_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the name forward nt_sampa pronunciation index.
   * @return  Returns the index for the name forward nt_sampa pronunciation.
   */
  uint32_t name_forward_pronunciation_nt_sampa_index() const {
    return name_forward_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for name forward nt_sampa lang pronunciation
   * @param  idx  Index for the name forward nt_sampa lang pronunciation.
   */
  void set_name_forward_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    name_forward_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the name forward nt_sampa lang pronunciation index.
   * @return  Returns the index for the name forward nt_sampa lang pronunciation.
   */
  uint32_t name_forward_pronunciation_nt_sampa_lang_index() const {
    return name_forward_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for name backward nt_sampa pronunciation
   * @param  idx  Index for the name backward nt_sampa pronunciation.
   */
  void set_name_backward_pronunciation_nt_sampa_index(const uint32_t idx) {
    name_backward_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the name backward nt_sampa pronunciation index.
   * @return  Returns the index for the name backward nt_sampa pronunciation.
   */
  uint32_t name_backward_pronunciation_nt_sampa_index() const {
    return name_backward_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for name backward nt_sampa lang pronunciation
   * @param  idx  Index for the name backward nt_sampa lang pronunciation.
   */
  void set_name_backward_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    name_backward_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the name backward nt_sampa lang pronunciation index.
   * @return  Returns the index for the name backward nt_sampa lang pronunciation.
   */
  uint32_t name_backward_pronunciation_nt_sampa_lang_index() const {
    return name_backward_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for alt name nt_sampa pronunciation
   * @param  idx  Index for the alt name nt_sampa pronunciation.
   */
  void set_alt_name_pronunciation_nt_sampa_index(const uint32_t idx) {
    alt_name_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the alt name nt_sampa pronunciation index.
   * @return  Returns the index for the alt name nt_sampa pronunciation.
   */
  uint32_t alt_name_pronunciation_nt_sampa_index() const {
    return alt_name_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for alt name nt_sampa lang pronunciation
   * @param  idx  Index for the alt name nt_sampa lang pronunciation.
   */
  void set_alt_name_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    alt_name_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the alt name nt_sampa lang pronunciation index.
   * @return  Returns the index for the alt name nt_sampa lang pronunciation.
   */
  uint32_t alt_name_pronunciation_nt_sampa_lang_index() const {
    return alt_name_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for alt name left nt_sampa pronunciation
   * @param  idx  Index for the alt name left nt_sampa pronunciation.
   */
  void set_alt_name_left_pronunciation_nt_sampa_index(const uint32_t idx) {
    alt_name_left_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the alt name left nt_sampa pronunciation index.
   * @return  Returns the index for the alt name left nt_sampa pronunciation.
   */
  uint32_t alt_name_left_pronunciation_nt_sampa_index() const {
    return alt_name_left_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for alt name left nt_sampa lang pronunciation
   * @param  idx  Index for the alt name left nt_sampa lang pronunciation.
   */
  void set_alt_name_left_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    alt_name_left_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the alt name left nt_sampa lang pronunciation index.
   * @return  Returns the index for the alt name left nt_sampa lang pronunciation.
   */
  uint32_t alt_name_left_pronunciation_nt_sampa_lang_index() const {
    return alt_name_left_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for alt name right nt_sampa pronunciation
   * @param  idx  Index for the alt name right nt_sampa pronunciation.
   */
  void set_alt_name_right_pronunciation_nt_sampa_index(const uint32_t idx) {
    alt_name_right_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the alt name right nt_sampa pronunciation index.
   * @return  Returns the index for the alt name right nt_sampa pronunciation.
   */
  uint32_t alt_name_right_pronunciation_nt_sampa_index() const {
    return alt_name_right_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for alt name right nt_sampa lang pronunciation
   * @param  idx  Index for the alt name right nt_sampa lang pronunciation.
   */
  void set_alt_name_right_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    alt_name_right_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the alt name right nt_sampa lang pronunciation index.
   * @return  Returns the index for the alt name right nt_sampa lang pronunciation.
   */
  uint32_t alt_name_right_pronunciation_nt_sampa_lang_index() const {
    return alt_name_right_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for alt name forward nt_sampa pronunciation
   * @param  idx  Index for the alt name forward nt_sampa pronunciation.
   */
  void set_alt_name_forward_pronunciation_nt_sampa_index(const uint32_t idx) {
    alt_name_forward_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the alt name forward nt_sampa pronunciation index.
   * @return  Returns the index for the alt name forward nt_sampa pronunciation.
   */
  uint32_t alt_name_forward_pronunciation_nt_sampa_index() const {
    return alt_name_forward_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for alt name forward nt_sampa lang pronunciation
   * @param  idx  Index for the alt name forward nt_sampa lang pronunciation.
   */
  void set_alt_name_forward_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    alt_name_forward_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the alt name forward nt_sampa lang pronunciation index.
   * @return  Returns the index for the alt name forward nt_sampa lang pronunciation.
   */
  uint32_t alt_name_forward_pronunciation_nt_sampa_lang_index() const {
    return alt_name_forward_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for alt name backward nt_sampa pronunciation
   * @param  idx  Index for the alt name backward nt_sampa pronunciation.
   */
  void set_alt_name_backward_pronunciation_nt_sampa_index(const uint32_t idx) {
    alt_name_backward_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the alt name backward nt_sampa pronunciation index.
   * @return  Returns the index for the alt name backward nt_sampa pronunciation.
   */
  uint32_t alt_name_backward_pronunciation_nt_sampa_index() const {
    return alt_name_backward_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for alt name backward nt_sampa lang pronunciation
   * @param  idx  Index for the alt name backward nt_sampa lang pronunciation.
   */
  void set_alt_name_backward_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    alt_name_backward_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the alt name backward nt_sampa lang pronunciation index.
   * @return  Returns the index for the alt name backward nt_sampa lang pronunciation.
   */
  uint32_t alt_name_backward_pronunciation_nt_sampa_lang_index() const {
    return alt_name_backward_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for official name nt_sampa pronunciation
   * @param  idx  Index for the official name nt_sampa pronunciation.
   */
  void set_official_name_pronunciation_nt_sampa_index(const uint32_t idx) {
    official_name_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the official name nt_sampa pronunciation index.
   * @return  Returns the index for the official name nt_sampa pronunciation.
   */
  uint32_t official_name_pronunciation_nt_sampa_index() const {
    return official_name_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for official name nt_sampa lang pronunciation
   * @param  idx  Index for the official name nt_sampa lang pronunciation.
   */
  void set_official_name_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    official_name_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the official name nt_sampa pronunciation lang index.
   * @return  Returns the index for the official name nt_sampa lang pronunciation.
   */
  uint32_t official_name_pronunciation_nt_sampa_lang_index() const {
    return official_name_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for official name left nt_sampa pronunciation
   * @param  idx  Index for the official name left nt_sampa pronunciation.
   */
  void set_official_name_left_pronunciation_nt_sampa_index(const uint32_t idx) {
    official_name_left_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the official name left nt_sampa pronunciation index.
   * @return  Returns the index for the official name left nt_sampa pronunciation.
   */
  uint32_t official_name_left_pronunciation_nt_sampa_index() const {
    return official_name_left_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for official name left nt_sampa lang pronunciation
   * @param  idx  Index for the official name left nt_sampa lang pronunciation.
   */
  void set_official_name_left_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    official_name_left_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the official name left nt_sampa lang pronunciation index.
   * @return  Returns the index for the official name left nt_sampa lang pronunciation.
   */
  uint32_t official_name_left_pronunciation_nt_sampa_lang_index() const {
    return official_name_left_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for official name right nt_sampa pronunciation
   * @param  idx  Index for the official name right nt_sampa pronunciation.
   */
  void set_official_name_right_pronunciation_nt_sampa_index(const uint32_t idx) {
    official_name_right_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the official name right nt_sampa pronunciation index.
   * @return  Returns the index for the official name right nt_sampa pronunciation.
   */
  uint32_t official_name_right_pronunciation_nt_sampa_index() const {
    return official_name_right_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for official name right nt_sampa lang pronunciation
   * @param  idx  Index for the official name right nt_sampa lang pronunciation.
   */
  void set_official_name_right_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    official_name_right_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the official name right nt_sampa lang pronunciation index.
   * @return  Returns the index for the official name right nt_sampa lang pronunciation.
   */
  uint32_t official_name_right_pronunciation_nt_sampa_lang_index() const {
    return official_name_right_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for official name forward nt_sampa pronunciation
   * @param  idx  Index for the official name forward nt_sampa pronunciation.
   */
  void set_official_name_forward_pronunciation_nt_sampa_index(const uint32_t idx) {
    official_name_forward_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the official name forward nt_sampa pronunciation index.
   * @return  Returns the index for the official name forward nt_sampa pronunciation.
   */
  uint32_t official_name_forward_pronunciation_nt_sampa_index() const {
    return official_name_forward_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for official name forward nt_sampa lang pronunciation
   * @param  idx  Index for the official name forward nt_sampa lang pronunciation.
   */
  void set_official_name_forward_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    official_name_forward_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the official name forward nt_sampa lang pronunciation index.
   * @return  Returns the index for the official name forward nt_sampa lang pronunciation.
   */
  uint32_t official_name_forward_pronunciation_nt_sampa_lang_index() const {
    return official_name_forward_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for official name backward nt_sampa pronunciation
   * @param  idx  Index for the official name backward nt_sampa pronunciation.
   */
  void set_official_name_backward_pronunciation_nt_sampa_index(const uint32_t idx) {
    official_name_backward_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the official name backward nt_sampa pronunciation index.
   * @return  Returns the index for the official name backward nt_sampa pronunciation.
   */
  uint32_t official_name_backward_pronunciation_nt_sampa_index() const {
    return official_name_backward_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for official name backward nt_sampa lang pronunciation
   * @param  idx  Index for the official name backward nt_sampa lang pronunciation.
   */
  void set_official_name_backward_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    official_name_backward_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the official name backward nt_sampa lang pronunciation index.
   * @return  Returns the index for the official name backward nt_sampa lang pronunciation.
   */
  uint32_t official_name_backward_pronunciation_nt_sampa_lang_index() const {
    return official_name_backward_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for tunnel name nt_sampa pronunciation
   * @param  idx  Index for the tunnel name nt_sampa pronunciation.
   */
  void set_tunnel_name_pronunciation_nt_sampa_index(const uint32_t idx) {
    tunnel_name_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the tunnel name nt_sampa pronunciation index.
   * @return  Returns the index for the tunnel name nt_sampa pronunciation.
   */
  uint32_t tunnel_name_pronunciation_nt_sampa_index() const {
    return tunnel_name_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for tunnel name nt_sampa lang pronunciation
   * @param  idx  Index for the tunnel name nt_sampa lang pronunciation.
   */
  void set_tunnel_name_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    tunnel_name_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the tunnel name nt_sampa pronunciation lang index.
   * @return  Returns the index for the tunnel name nt_sampa lang pronunciation.
   */
  uint32_t tunnel_name_pronunciation_nt_sampa_lang_index() const {
    return tunnel_name_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for tunnel name left nt_sampa pronunciation
   * @param  idx  Index for the tunnel name left nt_sampa pronunciation.
   */
  void set_tunnel_name_left_pronunciation_nt_sampa_index(const uint32_t idx) {
    tunnel_name_left_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the tunnel name left nt_sampa pronunciation index.
   * @return  Returns the index for the tunnel name left nt_sampa pronunciation.
   */
  uint32_t tunnel_name_left_pronunciation_nt_sampa_index() const {
    return tunnel_name_left_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for tunnel name left nt_sampa lang pronunciation
   * @param  idx  Index for the tunnel name left nt_sampa lang pronunciation.
   */
  void set_tunnel_name_left_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    tunnel_name_left_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the tunnel name left nt_sampa lang pronunciation index.
   * @return  Returns the index for the tunnel name left nt_sampa lang pronunciation.
   */
  uint32_t tunnel_name_left_pronunciation_nt_sampa_lang_index() const {
    return tunnel_name_left_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for tunnel name right nt_sampa pronunciation
   * @param  idx  Index for the tunnel name right nt_sampa pronunciation.
   */
  void set_tunnel_name_right_pronunciation_nt_sampa_index(const uint32_t idx) {
    tunnel_name_right_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the tunnel name right nt_sampa pronunciation index.
   * @return  Returns the index for the tunnel name right nt_sampa pronunciation.
   */
  uint32_t tunnel_name_right_pronunciation_nt_sampa_index() const {
    return tunnel_name_right_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for tunnel name right nt_sampa lang pronunciation
   * @param  idx  Index for the tunnel name right nt_sampa lang pronunciation.
   */
  void set_tunnel_name_right_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    tunnel_name_right_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the tunnel name right nt_sampa lang pronunciation index.
   * @return  Returns the index for the tunnel name right nt_sampa lang pronunciation.
   */
  uint32_t tunnel_name_right_pronunciation_nt_sampa_lang_index() const {
    return tunnel_name_right_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for tunnel name forward nt_sampa pronunciation
   * @param  idx  Index for the tunnel name forward nt_sampa pronunciation.
   */
  void set_tunnel_name_forward_pronunciation_nt_sampa_index(const uint32_t idx) {
    tunnel_name_forward_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the tunnel name forward nt_sampa pronunciation index.
   * @return  Returns the index for the tunnel name forward nt_sampa pronunciation.
   */
  uint32_t tunnel_name_forward_pronunciation_nt_sampa_index() const {
    return tunnel_name_forward_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for tunnel name forward nt_sampa lang pronunciation
   * @param  idx  Index for the tunnel name forward nt_sampa lang pronunciation.
   */
  void set_tunnel_name_forward_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    tunnel_name_forward_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the tunnel name forward nt_sampa lang pronunciation index.
   * @return  Returns the index for the tunnel name forward nt_sampa lang pronunciation.
   */
  uint32_t tunnel_name_forward_pronunciation_nt_sampa_lang_index() const {
    return tunnel_name_forward_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for tunnel name backward nt_sampa pronunciation
   * @param  idx  Index for the tunnel name backward nt_sampa pronunciation.
   */
  void set_tunnel_name_backward_pronunciation_nt_sampa_index(const uint32_t idx) {
    tunnel_name_backward_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the tunnel name backward nt_sampa pronunciation index.
   * @return  Returns the index for the tunnel name backward nt_sampa pronunciation.
   */
  uint32_t tunnel_name_backward_pronunciation_nt_sampa_index() const {
    return tunnel_name_backward_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for tunnel name backward nt_sampa lang pronunciation
   * @param  idx  Index for the tunnel name backward nt_sampa lang pronunciation.
   */
  void set_tunnel_name_backward_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    tunnel_name_backward_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the tunnel name backward nt_sampa lang pronunciation index.
   * @return  Returns the index for the tunnel name backward nt_sampa lang pronunciation.
   */
  uint32_t tunnel_name_backward_pronunciation_nt_sampa_lang_index() const {
    return tunnel_name_backward_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for destination nt_sampa pronunciation.
   * @param  idx  Index for the destination nt_sampa pronunciation.
   */
  void set_destination_pronunciation_nt_sampa_index(const uint32_t idx) {
    destination_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the get_destination nt_sampa pronunciation index.
   * @return  Returns the index for the destination nt_sampa pronunciation.
   */
  uint32_t destination_pronunciation_nt_sampa_index() const {
    return destination_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for destination nt_sampa lang pronunciation.
   * @param  idx  Index for the destination nt_sampa lang pronunciation.
   */
  void set_destination_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    destination_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the get_destination nt_sampa lang pronunciation index.
   * @return  Returns the index for the destination nt_sampa lang pronunciation.
   */
  uint32_t destination_pronunciation_nt_sampa_lang_index() const {
    return destination_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for destination in forward direction nt_sampa pronunciation.
   * @param  idx  Index for the destination nt_sampa pronunciation.
   */
  void set_destination_forward_pronunciation_nt_sampa_index(const uint32_t idx) {
    destination_forward_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the forward direction nt_sampa pronunciation index.
   * @return  Returns the index for the forward direction nt_sampa pronunciation.
   */
  uint32_t destination_forward_pronunciation_nt_sampa_index() const {
    return destination_forward_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for destination in forward direction nt_sampa lang pronunciation.
   * @param  idx  Index for the destination nt_sampa lang pronunciation.
   */
  void set_destination_forward_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    destination_forward_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the forward direction nt_sampa lang pronunciation index.
   * @return  Returns the index for the forward direction nt_sampa lang pronunciation.
   */
  uint32_t destination_forward_pronunciation_nt_sampa_lang_index() const {
    return destination_forward_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for destination in backward direction nt_sampa pronunciation.
   * @param  idx  Index for the backward direction nt_sampa pronunciation.
   */
  void set_destination_backward_pronunciation_nt_sampa_index(const uint32_t idx) {
    destination_backward_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the backward direction nt_sampa pronunciation index.
   * @return  Returns the index for the backward direction nt_sampa pronunciation.
   */
  uint32_t destination_backward_pronunciation_nt_sampa_index() const {
    return destination_backward_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for destination in backward direction nt_sampa lang pronunciation.
   * @param  idx  Index for the backward direction nt_sampa lang pronunciation.
   */
  void set_destination_backward_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    destination_backward_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the backward direction nt_sampa lang pronunciation index.
   * @return  Returns the index for the backward direction nt_sampa lang pronunciation.
   */
  uint32_t destination_backward_pronunciation_nt_sampa_lang_index() const {
    return destination_backward_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for destination ref nt_sampa pronunciation.
   * @param  idx  Index for the destination ref nt_sampa pronunciation.
   */
  void set_destination_ref_pronunciation_nt_sampa_index(const uint32_t idx) {
    destination_ref_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the destination ref nt_sampa pronunciation index.
   * @return  Returns the index for the destination ref nt_sampa pronunciation.
   */
  uint32_t destination_ref_pronunciation_nt_sampa_index() const {
    return destination_ref_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for destination ref nt_sampa lang pronunciation.
   * @param  idx  Index for the destination ref nt_sampa lang pronunciation.
   */
  void set_destination_ref_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    destination_ref_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the destination ref nt_sampa lang pronunciation index.
   * @return  Returns the index for the destination ref nt_sampa lang pronunciation.
   */
  uint32_t destination_ref_pronunciation_nt_sampa_lang_index() const {
    return destination_ref_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for destination ref to nt_sampa pronunciation.
   * @param  idx  Index for the destination ref to nt_sampa pronunciation.
   */
  void set_destination_ref_to_pronunciation_nt_sampa_index(const uint32_t idx) {
    destination_ref_to_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the destination ref to nt_sampa pronunciation index.
   * @return  Returns the index for the destination ref to nt_sampa pronunciation.
   */
  uint32_t destination_ref_to_pronunciation_nt_sampa_index() const {
    return destination_ref_to_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for destination ref to nt_sampa lang pronunciation.
   * @param  idx  Index for the destination ref to nt_sampa lang pronunciation.
   */
  void set_destination_ref_to_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    destination_ref_to_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the destination ref to nt_sampa lang pronunciation index.
   * @return  Returns the index for the destination ref to nt_sampa lang pronunciation.
   */
  uint32_t destination_ref_to_pronunciation_nt_sampa_lang_index() const {
    return destination_ref_to_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for destination street nt_sampa pronunciation.
   * @param  idx  Index for the destination street nt_sampa pronunciation.
   */
  void set_destination_street_pronunciation_nt_sampa_index(const uint32_t idx) {
    destination_street_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the destination_street nt_sampa pronunciation index.
   * @return  Returns the index for the destination street nt_sampa pronunciation.
   */
  uint32_t destination_street_pronunciation_nt_sampa_index() const {
    return destination_street_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for destination street nt_sampa lang pronunciation.
   * @param  idx  Index for the destination street nt_sampa lang pronunciation.
   */
  void set_destination_street_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    destination_street_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the destination_street nt_sampa lang pronunciation index.
   * @return  Returns the index for the destination street nt_sampa lang pronunciation.
   */
  uint32_t destination_street_pronunciation_nt_sampa_lang_index() const {
    return destination_street_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for destination street to nt_sampa pronunciation.
   * @param  idx  Index for the destination street to nt_sampa pronunciation.
   */
  void set_destination_street_to_pronunciation_nt_sampa_index(const uint32_t idx) {
    destination_street_to_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the destination street to nt_sampa pronunciation index.
   * @return  Returns the index for the destination street to nt_sampa pronunciation.
   */
  uint32_t destination_street_to_pronunciation_nt_sampa_index() const {
    return destination_street_to_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for destination street to nt_sampa lang pronunciation.
   * @param  idx  Index for the destination street to nt_sampa lang pronunciation.
   */
  void set_destination_street_to_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    destination_street_to_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the destination street to nt_sampa lang pronunciation index.
   * @return  Returns the index for the destination street to nt_sampa lang pronunciation.
   */
  uint32_t destination_street_to_pronunciation_nt_sampa_lang_index() const {
    return destination_street_to_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for junction ref nt_sampa pronunciation.
   * @param  idx  Index for the junction ref nt_sampa pronunciation.
   */
  void set_junction_ref_pronunciation_nt_sampa_index(const uint32_t idx) {
    junction_ref_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the junction ref nt_sampa pronunciation index.
   * @return  Returns the nt_sampa pronunciation index for the junction ref.
   */
  uint32_t junction_ref_pronunciation_nt_sampa_index() const {
    return junction_ref_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for junction ref nt_sampa lang pronunciation.
   * @param  idx  Index for the junction ref nt_sampa lang pronunciation.
   */
  void set_junction_ref_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    junction_ref_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the junction ref nt_sampa lang pronunciation index.
   * @return  Returns the nt_sampa lang pronunciation index for the junction ref.
   */
  uint32_t junction_ref_pronunciation_nt_sampa_lang_index() const {
    return junction_ref_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for junction name nt_sampa pronunciation.
   * @param  idx  Index for the junction name nt_sampa pronunciation.
   */
  void set_junction_name_pronunciation_nt_sampa_index(const uint32_t idx) {
    junction_name_pronunciation_nt_sampa_index_ = idx;
  }

  /**
   * Get the junction name nt_sampa pronunciation index.
   * @return  Returns the nt_sampa pronunciation index for the junction name.
   */
  uint32_t junction_name_pronunciation_nt_sampa_index() const {
    return junction_name_pronunciation_nt_sampa_index_;
  }

  /**
   * Sets the index for junction name nt_sampa lang pronunciation.
   * @param  idx  Index for the junction name nt_sampa lang pronunciation.
   */
  void set_junction_name_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    junction_name_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the junction name nt_sampa lang pronunciation index.
   * @return  Returns the nt_sampa lang pronunciation index for the junction name.
   */
  uint32_t junction_name_pronunciation_nt_sampa_lang_index() const {
    return junction_name_pronunciation_nt_sampa_lang_index_;
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
   * Sets the index for the ref katakana lang pronunciation
   * @param  idx  Index for the reference katakana lang pronunciation.
   */
  void set_ref_pronunciation_katakana_lang_index(const uint32_t idx) {
    ref_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the ref katakana lang pronunciation index.
   * @return  Returns the index for the ref katakana lang pronunciation.
   */
  uint32_t ref_pronunciation_katakana_lang_index() const {
    return ref_pronunciation_katakana_lang_index_;
  }

  /**
   * Sets the index for ref left katakana pronunciation
   * @param  idx  Index for the ref left katakana pronunciation.
   */
  void set_ref_left_pronunciation_katakana_index(const uint32_t idx) {
    ref_left_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the ref left katakana pronunciation index.
   * @return  Returns the index for the ref left katakana pronunciation.
   */
  uint32_t ref_left_pronunciation_katakana_index() const {
    return ref_left_pronunciation_katakana_index_;
  }

  /**
   * Sets the index for ref left katakana lang pronunciation
   * @param  idx  Index for the ref left katakana lang pronunciation.
   */
  void set_ref_left_pronunciation_katakana_lang_index(const uint32_t idx) {
    ref_left_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the ref left katakana lang pronunciation index.
   * @return  Returns the index for the ref left katakana lang pronunciation.
   */
  uint32_t ref_left_pronunciation_katakana_lang_index() const {
    return ref_left_pronunciation_katakana_lang_index_;
  }

  /**
   * Sets the index for ref right katakana pronunciation
   * @param  idx  Index for the ref right katakana pronunciation.
   */
  void set_ref_right_pronunciation_katakana_index(const uint32_t idx) {
    ref_right_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the ref right katakana pronunciation index.
   * @return  Returns the index for the ref right katakana pronunciation.
   */
  uint32_t ref_right_pronunciation_katakana_index() const {
    return ref_right_pronunciation_katakana_index_;
  }

  /**
   * Sets the index for ref right katakana lang pronunciation
   * @param  idx  Index for the ref right katakana lang pronunciation.
   */
  void set_ref_right_pronunciation_katakana_lang_index(const uint32_t idx) {
    ref_right_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the ref right katakana lang pronunciation index.
   * @return  Returns the index for the ref right katakana lang pronunciation.
   */
  uint32_t ref_right_pronunciation_katakana_lang_index() const {
    return ref_right_pronunciation_katakana_lang_index_;
  }

  /**
   * Sets the index for int ref katakana pronunciation
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
   * Sets the index for int ref katakana lang pronunciation
   * @param  idx  Index for the international reference katakana lang pronunciation.
   */
  void set_int_ref_pronunciation_katakana_lang_index(const uint32_t idx) {
    int_ref_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the int ref katakana lang pronunciation index.
   * @return  Returns the index for the int ref katakana lang pronunciation.
   */
  uint32_t int_ref_pronunciation_katakana_lang_index() const {
    return int_ref_pronunciation_katakana_lang_index_;
  }

  /**
   * Sets the index for int ref left katakana pronunciation
   * @param  idx  Index for the int ref left katakana pronunciation.
   */
  void set_int_ref_left_pronunciation_katakana_index(const uint32_t idx) {
    int_ref_left_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the int ref left katakana pronunciation index.
   * @return  Returns the index for the int ref left katakana pronunciation.
   */
  uint32_t int_ref_left_pronunciation_katakana_index() const {
    return int_ref_left_pronunciation_katakana_index_;
  }

  /**
   * Sets the index for int ref left katakana lang pronunciation
   * @param  idx  Index for the int ref left katakana lang pronunciation.
   */
  void set_int_ref_left_pronunciation_katakana_lang_index(const uint32_t idx) {
    int_ref_left_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the int ref left katakana lang pronunciation index.
   * @return  Returns the index for the int ref left katakana lang pronunciation.
   */
  uint32_t int_ref_left_pronunciation_katakana_lang_index() const {
    return int_ref_left_pronunciation_katakana_lang_index_;
  }

  /**
   * Sets the index for int ref right katakana pronunciation
   * @param  idx  Index for the int ref right katakana pronunciation.
   */
  void set_int_ref_right_pronunciation_katakana_index(const uint32_t idx) {
    int_ref_right_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the int ref right katakana pronunciation index.
   * @return  Returns the index for the int ref right katakana pronunciation.
   */
  uint32_t int_ref_right_pronunciation_katakana_index() const {
    return int_ref_right_pronunciation_katakana_index_;
  }

  /**
   * Sets the index for int ref right katakana lang pronunciation
   * @param  idx  Index for the int ref right katakana lang pronunciation.
   */
  void set_int_ref_right_pronunciation_katakana_lang_index(const uint32_t idx) {
    int_ref_right_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the int ref right katakana lang pronunciation index.
   * @return  Returns the index for the int ref right katakana lang pronunciation.
   */
  uint32_t int_ref_right_pronunciation_katakana_lang_index() const {
    return int_ref_right_pronunciation_katakana_lang_index_;
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
   * Sets the index for name katakana lang pronunciation
   * @param  idx  Index for the name katakana lang pronunciation.
   */
  void set_name_pronunciation_katakana_lang_index(const uint32_t idx) {
    name_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the name katakana lang pronunciation index.
   * @return  Returns the index for the name katakana lang pronunciation.
   */
  uint32_t name_pronunciation_katakana_lang_index() const {
    return name_pronunciation_katakana_lang_index_;
  }

  /**
   * Sets the index for name left katakana pronunciation
   * @param  idx  Index for the name left katakana pronunciation.
   */
  void set_name_left_pronunciation_katakana_index(const uint32_t idx) {
    name_left_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the name left katakana pronunciation index.
   * @return  Returns the index for the name left katakana pronunciation.
   */
  uint32_t name_left_pronunciation_katakana_index() const {
    return name_left_pronunciation_katakana_index_;
  }

  /**
   * Sets the index for name left katakana lang pronunciation
   * @param  idx  Index for the name left katakana lang pronunciation.
   */
  void set_name_left_pronunciation_katakana_lang_index(const uint32_t idx) {
    name_left_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the name left katakana lang pronunciation index.
   * @return  Returns the index for the name left katakana lang pronunciation.
   */
  uint32_t name_left_pronunciation_katakana_lang_index() const {
    return name_left_pronunciation_katakana_lang_index_;
  }

  /**
   * Sets the index for name right katakana pronunciation
   * @param  idx  Index for the name right katakana pronunciation.
   */
  void set_name_right_pronunciation_katakana_index(const uint32_t idx) {
    name_right_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the name right katakana pronunciation index.
   * @return  Returns the index for the name right katakana pronunciation.
   */
  uint32_t name_right_pronunciation_katakana_index() const {
    return name_right_pronunciation_katakana_index_;
  }

  /**
   * Sets the index for name right katakana lang pronunciation
   * @param  idx  Index for the name right katakana lang pronunciation.
   */
  void set_name_right_pronunciation_katakana_lang_index(const uint32_t idx) {
    name_right_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the name right katakana lang pronunciation index.
   * @return  Returns the index for the name right katakana lang pronunciation.
   */
  uint32_t name_right_pronunciation_katakana_lang_index() const {
    return name_right_pronunciation_katakana_lang_index_;
  }

  /**
   * Sets the index for name forward katakana pronunciation
   * @param  idx  Index for the name forward katakana pronunciation.
   */
  void set_name_forward_pronunciation_katakana_index(const uint32_t idx) {
    name_forward_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the name forward katakana pronunciation index.
   * @return  Returns the index for the name forward katakana pronunciation.
   */
  uint32_t name_forward_pronunciation_katakana_index() const {
    return name_forward_pronunciation_katakana_index_;
  }

  /**
   * Sets the index for name forward katakana lang pronunciation
   * @param  idx  Index for the name forward katakana lang pronunciation.
   */
  void set_name_forward_pronunciation_katakana_lang_index(const uint32_t idx) {
    name_forward_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the name forward katakana lang pronunciation index.
   * @return  Returns the index for the name forward katakana lang pronunciation.
   */
  uint32_t name_forward_pronunciation_katakana_lang_index() const {
    return name_forward_pronunciation_katakana_lang_index_;
  }

  /**
   * Sets the index for name backward katakana pronunciation
   * @param  idx  Index for the name backward katakana pronunciation.
   */
  void set_name_backward_pronunciation_katakana_index(const uint32_t idx) {
    name_backward_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the name backward katakana pronunciation index.
   * @return  Returns the index for the name backward katakana pronunciation.
   */
  uint32_t name_backward_pronunciation_katakana_index() const {
    return name_backward_pronunciation_katakana_index_;
  }

  /**
   * Sets the index for name backward katakana lang pronunciation
   * @param  idx  Index for the name backward katakana lang pronunciation.
   */
  void set_name_backward_pronunciation_katakana_lang_index(const uint32_t idx) {
    name_backward_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the name backward katakana lang pronunciation index.
   * @return  Returns the index for the name backward katakana lang pronunciation.
   */
  uint32_t name_backward_pronunciation_katakana_lang_index() const {
    return name_backward_pronunciation_katakana_lang_index_;
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
   * Sets the index for alt name katakana lang pronunciation
   * @param  idx  Index for the alt name katakana lang pronunciation.
   */
  void set_alt_name_pronunciation_katakana_lang_index(const uint32_t idx) {
    alt_name_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the alt name katakana pronunciation lang index.
   * @return  Returns the index for the alt name katakana lang pronunciation.
   */
  uint32_t alt_name_pronunciation_katakana_lang_index() const {
    return alt_name_pronunciation_katakana_lang_index_;
  }

  /**
   * Sets the index for alt name left katakana pronunciation
   * @param  idx  Index for the alt name left katakana pronunciation.
   */
  void set_alt_name_left_pronunciation_katakana_index(const uint32_t idx) {
    alt_name_left_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the alt name left katakana pronunciation index.
   * @return  Returns the index for the alt name left katakana pronunciation.
   */
  uint32_t alt_name_left_pronunciation_katakana_index() const {
    return alt_name_left_pronunciation_katakana_index_;
  }

  /**
   * Sets the index for alt name left katakana lang pronunciation
   * @param  idx  Index for the alt name left katakana lang pronunciation.
   */
  void set_alt_name_left_pronunciation_katakana_lang_index(const uint32_t idx) {
    alt_name_left_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the alt name left katakana lang pronunciation index.
   * @return  Returns the index for the alt name left katakana lang pronunciation.
   */
  uint32_t alt_name_left_pronunciation_katakana_lang_index() const {
    return alt_name_left_pronunciation_katakana_lang_index_;
  }

  /**
   * Sets the index for alt name right katakana pronunciation
   * @param  idx  Index for the alt name right katakana pronunciation.
   */
  void set_alt_name_right_pronunciation_katakana_index(const uint32_t idx) {
    alt_name_right_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the alt name right katakana pronunciation index.
   * @return  Returns the index for the alt name right katakana pronunciation.
   */
  uint32_t alt_name_right_pronunciation_katakana_index() const {
    return alt_name_right_pronunciation_katakana_index_;
  }

  /**
   * Sets the index for alt name right katakana lang pronunciation
   * @param  idx  Index for the alt name right katakana lang pronunciation.
   */
  void set_alt_name_right_pronunciation_katakana_lang_index(const uint32_t idx) {
    alt_name_right_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the alt name right katakana lang pronunciation index.
   * @return  Returns the index for the alt name right katakana lang pronunciation.
   */
  uint32_t alt_name_right_pronunciation_katakana_lang_index() const {
    return alt_name_right_pronunciation_katakana_lang_index_;
  }

  /**
   * Sets the index for alt name forward katakana pronunciation
   * @param  idx  Index for the alt name forward katakana pronunciation.
   */
  void set_alt_name_forward_pronunciation_katakana_index(const uint32_t idx) {
    alt_name_forward_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the alt name forward katakana pronunciation index.
   * @return  Returns the index for the alt name forward katakana pronunciation.
   */
  uint32_t alt_name_forward_pronunciation_katakana_index() const {
    return alt_name_forward_pronunciation_katakana_index_;
  }

  /**
   * Sets the index for alt name forward katakana lang pronunciation
   * @param  idx  Index for the alt name forward katakana lang pronunciation.
   */
  void set_alt_name_forward_pronunciation_katakana_lang_index(const uint32_t idx) {
    alt_name_forward_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the alt name forward katakana lang pronunciation index.
   * @return  Returns the index for the alt name forward katakana lang pronunciation.
   */
  uint32_t alt_name_forward_pronunciation_katakana_lang_index() const {
    return alt_name_forward_pronunciation_katakana_lang_index_;
  }

  /**
   * Sets the index for alt name backward katakana pronunciation
   * @param  idx  Index for the alt name backward katakana pronunciation.
   */
  void set_alt_name_backward_pronunciation_katakana_index(const uint32_t idx) {
    alt_name_backward_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the alt name backward katakana pronunciation index.
   * @return  Returns the index for the alt name backward katakana pronunciation.
   */
  uint32_t alt_name_backward_pronunciation_katakana_index() const {
    return alt_name_backward_pronunciation_katakana_index_;
  }

  /**
   * Sets the index for alt name backward katakana lang pronunciation
   * @param  idx  Index for the alt name backward katakana lang pronunciation.
   */
  void set_alt_name_backward_pronunciation_katakana_lang_index(const uint32_t idx) {
    alt_name_backward_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the alt name backward katakana lang pronunciation index.
   * @return  Returns the index for the alt name backward katakana lang pronunciation.
   */
  uint32_t alt_name_backward_pronunciation_katakana_lang_index() const {
    return alt_name_backward_pronunciation_katakana_lang_index_;
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
   * Sets the index for official name katakana lang pronunciation
   * @param  idx  Index for the official name katakana lang pronunciation.
   */
  void set_official_name_pronunciation_katakana_lang_index(const uint32_t idx) {
    official_name_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the official name katakana pronunciation lang index.
   * @return  Returns the index for the official name katakana lang pronunciation.
   */
  uint32_t official_name_pronunciation_katakana_lang_index() const {
    return official_name_pronunciation_katakana_lang_index_;
  }

  /**
   * Sets the index for official name left katakana pronunciation
   * @param  idx  Index for the official name left katakana pronunciation.
   */
  void set_official_name_left_pronunciation_katakana_index(const uint32_t idx) {
    official_name_left_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the official name left katakana pronunciation index.
   * @return  Returns the index for the official name left katakana pronunciation.
   */
  uint32_t official_name_left_pronunciation_katakana_index() const {
    return official_name_left_pronunciation_katakana_index_;
  }

  /**
   * Sets the index for official name left katakana lang pronunciation
   * @param  idx  Index for the official name left katakana lang pronunciation.
   */
  void set_official_name_left_pronunciation_katakana_lang_index(const uint32_t idx) {
    official_name_left_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the official name left katakana lang pronunciation index.
   * @return  Returns the index for the official name left katakana lang pronunciation.
   */
  uint32_t official_name_left_pronunciation_katakana_lang_index() const {
    return official_name_left_pronunciation_katakana_lang_index_;
  }

  /**
   * Sets the index for official name right katakana pronunciation
   * @param  idx  Index for the official name right katakana pronunciation.
   */
  void set_official_name_right_pronunciation_katakana_index(const uint32_t idx) {
    official_name_right_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the official name right katakana pronunciation index.
   * @return  Returns the index for the official name right katakana pronunciation.
   */
  uint32_t official_name_right_pronunciation_katakana_index() const {
    return official_name_right_pronunciation_katakana_index_;
  }

  /**
   * Sets the index for official name right katakana lang pronunciation
   * @param  idx  Index for the official name right katakana lang pronunciation.
   */
  void set_official_name_right_pronunciation_katakana_lang_index(const uint32_t idx) {
    official_name_right_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the official name right katakana lang pronunciation index.
   * @return  Returns the index for the official name right katakana lang pronunciation.
   */
  uint32_t official_name_right_pronunciation_katakana_lang_index() const {
    return official_name_right_pronunciation_katakana_lang_index_;
  }

  /**
   * Sets the index for official name forward katakana pronunciation
   * @param  idx  Index for the official name forward katakana pronunciation.
   */
  void set_official_name_forward_pronunciation_katakana_index(const uint32_t idx) {
    official_name_forward_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the official name forward katakana pronunciation index.
   * @return  Returns the index for the official name forward katakana pronunciation.
   */
  uint32_t official_name_forward_pronunciation_katakana_index() const {
    return official_name_forward_pronunciation_katakana_index_;
  }

  /**
   * Sets the index for official name forward katakana lang pronunciation
   * @param  idx  Index for the official name forward katakana lang pronunciation.
   */
  void set_official_name_forward_pronunciation_katakana_lang_index(const uint32_t idx) {
    official_name_forward_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the official name forward katakana lang pronunciation index.
   * @return  Returns the index for the official name forward katakana lang pronunciation.
   */
  uint32_t official_name_forward_pronunciation_katakana_lang_index() const {
    return official_name_forward_pronunciation_katakana_lang_index_;
  }

  /**
   * Sets the index for official name backward katakana pronunciation
   * @param  idx  Index for the official name backward katakana pronunciation.
   */
  void set_official_name_backward_pronunciation_katakana_index(const uint32_t idx) {
    official_name_backward_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the official name backward katakana pronunciation index.
   * @return  Returns the index for the official name backward katakana pronunciation.
   */
  uint32_t official_name_backward_pronunciation_katakana_index() const {
    return official_name_backward_pronunciation_katakana_index_;
  }

  /**
   * Sets the index for official name backward katakana lang pronunciation
   * @param  idx  Index for the official name backward katakana lang pronunciation.
   */
  void set_official_name_backward_pronunciation_katakana_lang_index(const uint32_t idx) {
    official_name_backward_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the official name backward katakana lang pronunciation index.
   * @return  Returns the index for the official name backward katakana lang pronunciation.
   */
  uint32_t official_name_backward_pronunciation_katakana_lang_index() const {
    return official_name_backward_pronunciation_katakana_lang_index_;
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
   * Sets the index for tunnel name katakana lang pronunciation
   * @param  idx  Index for the tunnel name katakana lang pronunciation.
   */
  void set_tunnel_name_pronunciation_katakana_lang_index(const uint32_t idx) {
    tunnel_name_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the tunnel name katakana pronunciation lang index.
   * @return  Returns the index for the tunnel name katakana lang pronunciation.
   */
  uint32_t tunnel_name_pronunciation_katakana_lang_index() const {
    return tunnel_name_pronunciation_katakana_lang_index_;
  }

  /**
   * Sets the index for tunnel name left katakana pronunciation
   * @param  idx  Index for the tunnel name left katakana pronunciation.
   */
  void set_tunnel_name_left_pronunciation_katakana_index(const uint32_t idx) {
    tunnel_name_left_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the tunnel name left katakana pronunciation index.
   * @return  Returns the index for the tunnel name left katakana pronunciation.
   */
  uint32_t tunnel_name_left_pronunciation_katakana_index() const {
    return tunnel_name_left_pronunciation_katakana_index_;
  }

  /**
   * Sets the index for tunnel name left katakana lang pronunciation
   * @param  idx  Index for the tunnel name left katakana lang pronunciation.
   */
  void set_tunnel_name_left_pronunciation_katakana_lang_index(const uint32_t idx) {
    tunnel_name_left_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the tunnel name left katakana lang pronunciation index.
   * @return  Returns the index for the tunnel name left katakana lang pronunciation.
   */
  uint32_t tunnel_name_left_pronunciation_katakana_lang_index() const {
    return tunnel_name_left_pronunciation_katakana_lang_index_;
  }

  /**
   * Sets the index for tunnel name right katakana pronunciation
   * @param  idx  Index for the tunnel name right katakana pronunciation.
   */
  void set_tunnel_name_right_pronunciation_katakana_index(const uint32_t idx) {
    tunnel_name_right_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the tunnel name right katakana pronunciation index.
   * @return  Returns the index for the tunnel name right katakana pronunciation.
   */
  uint32_t tunnel_name_right_pronunciation_katakana_index() const {
    return tunnel_name_right_pronunciation_katakana_index_;
  }

  /**
   * Sets the index for tunnel name right katakana lang pronunciation
   * @param  idx  Index for the tunnel name right katakana lang pronunciation.
   */
  void set_tunnel_name_right_pronunciation_katakana_lang_index(const uint32_t idx) {
    tunnel_name_right_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the tunnel name right katakana lang pronunciation index.
   * @return  Returns the index for the tunnel name right katakana lang pronunciation.
   */
  uint32_t tunnel_name_right_pronunciation_katakana_lang_index() const {
    return tunnel_name_right_pronunciation_katakana_lang_index_;
  }

  /**
   * Sets the index for tunnel name forward katakana pronunciation
   * @param  idx  Index for the tunnel name forward katakana pronunciation.
   */
  void set_tunnel_name_forward_pronunciation_katakana_index(const uint32_t idx) {
    tunnel_name_forward_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the tunnel name forward katakana pronunciation index.
   * @return  Returns the index for the tunnel name forward katakana pronunciation.
   */
  uint32_t tunnel_name_forward_pronunciation_katakana_index() const {
    return tunnel_name_forward_pronunciation_katakana_index_;
  }

  /**
   * Sets the index for tunnel name forward katakana lang pronunciation
   * @param  idx  Index for the tunnel name forward katakana lang pronunciation.
   */
  void set_tunnel_name_forward_pronunciation_katakana_lang_index(const uint32_t idx) {
    tunnel_name_forward_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the tunnel name forward katakana lang pronunciation index.
   * @return  Returns the index for the tunnel name forward katakana lang pronunciation.
   */
  uint32_t tunnel_name_forward_pronunciation_katakana_lang_index() const {
    return tunnel_name_forward_pronunciation_katakana_lang_index_;
  }

  /**
   * Sets the index for tunnel name backward katakana pronunciation
   * @param  idx  Index for the tunnel name backward katakana pronunciation.
   */
  void set_tunnel_name_backward_pronunciation_katakana_index(const uint32_t idx) {
    tunnel_name_backward_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the tunnel name backward katakana pronunciation index.
   * @return  Returns the index for the tunnel name backward katakana pronunciation.
   */
  uint32_t tunnel_name_backward_pronunciation_katakana_index() const {
    return tunnel_name_backward_pronunciation_katakana_index_;
  }

  /**
   * Sets the index for tunnel name backward katakana lang pronunciation
   * @param  idx  Index for the tunnel name backward katakana lang pronunciation.
   */
  void set_tunnel_name_backward_pronunciation_katakana_lang_index(const uint32_t idx) {
    tunnel_name_backward_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the tunnel name backward katakana lang pronunciation index.
   * @return  Returns the index for the tunnel name backward katakana lang pronunciation.
   */
  uint32_t tunnel_name_backward_pronunciation_katakana_lang_index() const {
    return tunnel_name_backward_pronunciation_katakana_lang_index_;
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
   * Sets the index for destination katakana lang pronunciation.
   * @param  idx  Index for the destination katakana lang pronunciation.
   */
  void set_destination_pronunciation_katakana_lang_index(const uint32_t idx) {
    destination_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the get_destination katakana lang pronunciation index.
   * @return  Returns the index for the destination katakana lang pronunciation.
   */
  uint32_t destination_pronunciation_katakana_lang_index() const {
    return destination_pronunciation_katakana_lang_index_;
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
   * Sets the index for destination in forward direction katakana lang pronunciation.
   * @param  idx  Index for the destination katakana lang pronunciation.
   */
  void set_destination_forward_pronunciation_katakana_lang_index(const uint32_t idx) {
    destination_forward_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the forward direction katakana lang pronunciation index.
   * @return  Returns the index for the forward direction katakana lang pronunciation.
   */
  uint32_t destination_forward_pronunciation_katakana_lang_index() const {
    return destination_forward_pronunciation_katakana_lang_index_;
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
   * Sets the index for destination in backward direction katakana lang pronunciation.
   * @param  idx  Index for the backward direction katakana lang pronunciation.
   */
  void set_destination_backward_pronunciation_katakana_lang_index(const uint32_t idx) {
    destination_backward_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the backward direction katakana lang pronunciation index.
   * @return  Returns the index for the backward direction katakana lang pronunciation.
   */
  uint32_t destination_backward_pronunciation_katakana_lang_index() const {
    return destination_backward_pronunciation_katakana_lang_index_;
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
   * Sets the index for destination ref katakana lang pronunciation.
   * @param  idx  Index for the destination ref katakana lang pronunciation.
   */
  void set_destination_ref_pronunciation_katakana_lang_index(const uint32_t idx) {
    destination_ref_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the destination ref katakana lang pronunciation index.
   * @return  Returns the index for the destination ref katakana lang pronunciation.
   */
  uint32_t destination_ref_pronunciation_katakana_lang_index() const {
    return destination_ref_pronunciation_katakana_lang_index_;
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
   * Sets the index for destination ref to katakana lang pronunciation.
   * @param  idx  Index for the destination ref to katakana lang pronunciation.
   */
  void set_destination_ref_to_pronunciation_katakana_lang_index(const uint32_t idx) {
    destination_ref_to_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the destination ref to katakana lang pronunciation index.
   * @return  Returns the index for the destination ref to katakana lang pronunciation.
   */
  uint32_t destination_ref_to_pronunciation_katakana_lang_index() const {
    return destination_ref_to_pronunciation_katakana_lang_index_;
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
   * Sets the index for destination street katakana lang pronunciation.
   * @param  idx  Index for the destination street katakana lang pronunciation.
   */
  void set_destination_street_pronunciation_katakana_lang_index(const uint32_t idx) {
    destination_street_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the destination_street katakana lang pronunciation index.
   * @return  Returns the index for the destination street katakana lang pronunciation.
   */
  uint32_t destination_street_pronunciation_katakana_lang_index() const {
    return destination_street_pronunciation_katakana_lang_index_;
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
   * Sets the index for destination street to katakana lang pronunciation.
   * @param  idx  Index for the destination street to katakana lang pronunciation.
   */
  void set_destination_street_to_pronunciation_katakana_lang_index(const uint32_t idx) {
    destination_street_to_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the destination street to katakana lang pronunciation index.
   * @return  Returns the index for the destination street to katakana lang pronunciation.
   */
  uint32_t destination_street_to_pronunciation_katakana_lang_index() const {
    return destination_street_to_pronunciation_katakana_lang_index_;
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

  /**
   * Sets the index for junction ref katakana lang pronunciation.
   * @param  idx  Index for the junction ref katakana lang pronunciation.
   */
  void set_junction_ref_pronunciation_katakana_lang_index(const uint32_t idx) {
    junction_ref_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the junction ref katakana lang pronunciation index.
   * @return  Returns the katakana lang pronunciation index for the junction ref.
   */
  uint32_t junction_ref_pronunciation_katakana_lang_index() const {
    return junction_ref_pronunciation_katakana_lang_index_;
  }

  /**
   * Sets the index for junction name katakana pronunciation.
   * @param  idx  Index for the junction name katakana pronunciation.
   */
  void set_junction_name_pronunciation_katakana_index(const uint32_t idx) {
    junction_name_pronunciation_katakana_index_ = idx;
  }

  /**
   * Get the junction name katakana pronunciation index.
   * @return  Returns the katakana pronunciation index for the junction name.
   */
  uint32_t junction_name_pronunciation_katakana_index() const {
    return junction_name_pronunciation_katakana_index_;
  }

  /**
   * Sets the index for junction name katakana lang pronunciation.
   * @param  idx  Index for the junction name katakana lang pronunciation.
   */
  void set_junction_name_pronunciation_katakana_lang_index(const uint32_t idx) {
    junction_name_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the junction name katakana lang pronunciation index.
   * @return  Returns the katakana lang pronunciation index for the junction name.
   */
  uint32_t junction_name_pronunciation_katakana_lang_index() const {
    return junction_name_pronunciation_katakana_lang_index_;
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
   * Sets the index for the ref jeita lang pronunciation
   * @param  idx  Index for the reference jeita lang pronunciation.
   */
  void set_ref_pronunciation_jeita_lang_index(const uint32_t idx) {
    ref_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the ref jeita lang pronunciation index.
   * @return  Returns the index for the ref jeita lang pronunciation.
   */
  uint32_t ref_pronunciation_jeita_lang_index() const {
    return ref_pronunciation_jeita_lang_index_;
  }

  /**
   * Sets the index for ref left jeita pronunciation
   * @param  idx  Index for the ref left jeita pronunciation.
   */
  void set_ref_left_pronunciation_jeita_index(const uint32_t idx) {
    ref_left_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the ref left jeita pronunciation index.
   * @return  Returns the index for the ref left jeita pronunciation.
   */
  uint32_t ref_left_pronunciation_jeita_index() const {
    return ref_left_pronunciation_jeita_index_;
  }

  /**
   * Sets the index for ref left jeita lang pronunciation
   * @param  idx  Index for the ref left jeita lang pronunciation.
   */
  void set_ref_left_pronunciation_jeita_lang_index(const uint32_t idx) {
    ref_left_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the ref left jeita lang pronunciation index.
   * @return  Returns the index for the ref left jeita lang pronunciation.
   */
  uint32_t ref_left_pronunciation_jeita_lang_index() const {
    return ref_left_pronunciation_jeita_lang_index_;
  }

  /**
   * Sets the index for ref right jeita pronunciation
   * @param  idx  Index for the ref right jeita pronunciation.
   */
  void set_ref_right_pronunciation_jeita_index(const uint32_t idx) {
    ref_right_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the ref right jeita pronunciation index.
   * @return  Returns the index for the ref right jeita pronunciation.
   */
  uint32_t ref_right_pronunciation_jeita_index() const {
    return ref_right_pronunciation_jeita_index_;
  }

  /**
   * Sets the index for ref right jeita lang pronunciation
   * @param  idx  Index for the ref right jeita lang pronunciation.
   */
  void set_ref_right_pronunciation_jeita_lang_index(const uint32_t idx) {
    ref_right_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the ref right jeita lang pronunciation index.
   * @return  Returns the index for the ref right jeita lang pronunciation.
   */
  uint32_t ref_right_pronunciation_jeita_lang_index() const {
    return ref_right_pronunciation_jeita_lang_index_;
  }

  /**
   * Sets the index for int ref jeita pronunciation
   * @param  idx  Index for the international reference jeita pronunciation.
   */
  void set_int_ref_pronunciation_jeita_index(const uint32_t idx) {
    int_ref_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the int ref jeita pronunciation index.
   * @return  Returns the index for the int ref jeita pronunciation.
   */
  uint32_t int_ref_pronunciation_jeita_index() const {
    return int_ref_pronunciation_jeita_index_;
  }

  /**
   * Sets the index for int ref jeita lang pronunciation
   * @param  idx  Index for the international reference jeita lang pronunciation.
   */
  void set_int_ref_pronunciation_jeita_lang_index(const uint32_t idx) {
    int_ref_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the int ref jeita lang pronunciation index.
   * @return  Returns the index for the int ref jeita lang pronunciation.
   */
  uint32_t int_ref_pronunciation_jeita_lang_index() const {
    return int_ref_pronunciation_jeita_lang_index_;
  }

  /**
   * Sets the index for int ref left jeita pronunciation
   * @param  idx  Index for the int ref left jeita pronunciation.
   */
  void set_int_ref_left_pronunciation_jeita_index(const uint32_t idx) {
    int_ref_left_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the int ref left jeita pronunciation index.
   * @return  Returns the index for the int ref left jeita pronunciation.
   */
  uint32_t int_ref_left_pronunciation_jeita_index() const {
    return int_ref_left_pronunciation_jeita_index_;
  }

  /**
   * Sets the index for int ref left jeita lang pronunciation
   * @param  idx  Index for the int ref left jeita lang pronunciation.
   */
  void set_int_ref_left_pronunciation_jeita_lang_index(const uint32_t idx) {
    int_ref_left_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the int ref left jeita lang pronunciation index.
   * @return  Returns the index for the int ref left jeita lang pronunciation.
   */
  uint32_t int_ref_left_pronunciation_jeita_lang_index() const {
    return int_ref_left_pronunciation_jeita_lang_index_;
  }

  /**
   * Sets the index for int ref right jeita pronunciation
   * @param  idx  Index for the int ref right jeita pronunciation.
   */
  void set_int_ref_right_pronunciation_jeita_index(const uint32_t idx) {
    int_ref_right_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the int ref right jeita pronunciation index.
   * @return  Returns the index for the int ref right jeita pronunciation.
   */
  uint32_t int_ref_right_pronunciation_jeita_index() const {
    return int_ref_right_pronunciation_jeita_index_;
  }

  /**
   * Sets the index for int ref right jeita lang pronunciation
   * @param  idx  Index for the int ref right jeita lang pronunciation.
   */
  void set_int_ref_right_pronunciation_jeita_lang_index(const uint32_t idx) {
    int_ref_right_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the int ref right jeita lang pronunciation index.
   * @return  Returns the index for the int ref right jeita lang pronunciation.
   */
  uint32_t int_ref_right_pronunciation_jeita_lang_index() const {
    return int_ref_right_pronunciation_jeita_lang_index_;
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
   * Sets the index for name jeita lang pronunciation
   * @param  idx  Index for the name jeita lang pronunciation.
   */
  void set_name_pronunciation_jeita_lang_index(const uint32_t idx) {
    name_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the name jeita lang pronunciation index.
   * @return  Returns the index for the name jeita lang pronunciation.
   */
  uint32_t name_pronunciation_jeita_lang_index() const {
    return name_pronunciation_jeita_lang_index_;
  }

  /**
   * Sets the index for name left jeita pronunciation
   * @param  idx  Index for the name left jeita pronunciation.
   */
  void set_name_left_pronunciation_jeita_index(const uint32_t idx) {
    name_left_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the name left jeita pronunciation index.
   * @return  Returns the index for the name left jeita pronunciation.
   */
  uint32_t name_left_pronunciation_jeita_index() const {
    return name_left_pronunciation_jeita_index_;
  }

  /**
   * Sets the index for name left jeita lang pronunciation
   * @param  idx  Index for the name left jeita lang pronunciation.
   */
  void set_name_left_pronunciation_jeita_lang_index(const uint32_t idx) {
    name_left_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the name left jeita lang pronunciation index.
   * @return  Returns the index for the name left jeita lang pronunciation.
   */
  uint32_t name_left_pronunciation_jeita_lang_index() const {
    return name_left_pronunciation_jeita_lang_index_;
  }

  /**
   * Sets the index for name right jeita pronunciation
   * @param  idx  Index for the name right jeita pronunciation.
   */
  void set_name_right_pronunciation_jeita_index(const uint32_t idx) {
    name_right_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the name right jeita pronunciation index.
   * @return  Returns the index for the name right jeita pronunciation.
   */
  uint32_t name_right_pronunciation_jeita_index() const {
    return name_right_pronunciation_jeita_index_;
  }

  /**
   * Sets the index for name right jeita lang pronunciation
   * @param  idx  Index for the name right jeita lang pronunciation.
   */
  void set_name_right_pronunciation_jeita_lang_index(const uint32_t idx) {
    name_right_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the name right jeita lang pronunciation index.
   * @return  Returns the index for the name right jeita lang pronunciation.
   */
  uint32_t name_right_pronunciation_jeita_lang_index() const {
    return name_right_pronunciation_jeita_lang_index_;
  }

  /**
   * Sets the index for name forward jeita pronunciation
   * @param  idx  Index for the name forward jeita pronunciation.
   */
  void set_name_forward_pronunciation_jeita_index(const uint32_t idx) {
    name_forward_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the name forward jeita pronunciation index.
   * @return  Returns the index for the name forward jeita pronunciation.
   */
  uint32_t name_forward_pronunciation_jeita_index() const {
    return name_forward_pronunciation_jeita_index_;
  }

  /**
   * Sets the index for name forward jeita lang pronunciation
   * @param  idx  Index for the name forward jeita lang pronunciation.
   */
  void set_name_forward_pronunciation_jeita_lang_index(const uint32_t idx) {
    name_forward_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the name forward jeita lang pronunciation index.
   * @return  Returns the index for the name forward jeita lang pronunciation.
   */
  uint32_t name_forward_pronunciation_jeita_lang_index() const {
    return name_forward_pronunciation_jeita_lang_index_;
  }

  /**
   * Sets the index for name backward jeita pronunciation
   * @param  idx  Index for the name backward jeita pronunciation.
   */
  void set_name_backward_pronunciation_jeita_index(const uint32_t idx) {
    name_backward_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the name backward jeita pronunciation index.
   * @return  Returns the index for the name backward jeita pronunciation.
   */
  uint32_t name_backward_pronunciation_jeita_index() const {
    return name_backward_pronunciation_jeita_index_;
  }

  /**
   * Sets the index for name backward jeita lang pronunciation
   * @param  idx  Index for the name backward jeita lang pronunciation.
   */
  void set_name_backward_pronunciation_jeita_lang_index(const uint32_t idx) {
    name_backward_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the name backward jeita lang pronunciation index.
   * @return  Returns the index for the name backward jeita lang pronunciation.
   */
  uint32_t name_backward_pronunciation_jeita_lang_index() const {
    return name_backward_pronunciation_jeita_lang_index_;
  }

  /**
   * Sets the index for alt name jeita pronunciation
   * @param  idx  Index for the alt name jeita pronunciation.
   */
  void set_alt_name_pronunciation_jeita_index(const uint32_t idx) {
    alt_name_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the alt name jeita pronunciation index.
   * @return  Returns the index for the alt name jeita pronunciation.
   */
  uint32_t alt_name_pronunciation_jeita_index() const {
    return alt_name_pronunciation_jeita_index_;
  }

  /**
   * Sets the index for alt name jeita lang pronunciation
   * @param  idx  Index for the alt name jeita lang pronunciation.
   */
  void set_alt_name_pronunciation_jeita_lang_index(const uint32_t idx) {
    alt_name_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the alt name jeita pronunciation lang index.
   * @return  Returns the index for the alt name jeita lang pronunciation.
   */
  uint32_t alt_name_pronunciation_jeita_lang_index() const {
    return alt_name_pronunciation_jeita_lang_index_;
  }

  /**
   * Sets the index for alt name left jeita pronunciation
   * @param  idx  Index for the alt name left jeita pronunciation.
   */
  void set_alt_name_left_pronunciation_jeita_index(const uint32_t idx) {
    alt_name_left_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the alt name left jeita pronunciation index.
   * @return  Returns the index for the alt name left jeita pronunciation.
   */
  uint32_t alt_name_left_pronunciation_jeita_index() const {
    return alt_name_left_pronunciation_jeita_index_;
  }

  /**
   * Sets the index for alt name left jeita lang pronunciation
   * @param  idx  Index for the alt name left jeita lang pronunciation.
   */
  void set_alt_name_left_pronunciation_jeita_lang_index(const uint32_t idx) {
    alt_name_left_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the alt name left jeita lang pronunciation index.
   * @return  Returns the index for the alt name left jeita lang pronunciation.
   */
  uint32_t alt_name_left_pronunciation_jeita_lang_index() const {
    return alt_name_left_pronunciation_jeita_lang_index_;
  }

  /**
   * Sets the index for alt name right jeita pronunciation
   * @param  idx  Index for the alt name right jeita pronunciation.
   */
  void set_alt_name_right_pronunciation_jeita_index(const uint32_t idx) {
    alt_name_right_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the alt name right jeita pronunciation index.
   * @return  Returns the index for the alt name right jeita pronunciation.
   */
  uint32_t alt_name_right_pronunciation_jeita_index() const {
    return alt_name_right_pronunciation_jeita_index_;
  }

  /**
   * Sets the index for alt name right jeita lang pronunciation
   * @param  idx  Index for the alt name right jeita lang pronunciation.
   */
  void set_alt_name_right_pronunciation_jeita_lang_index(const uint32_t idx) {
    alt_name_right_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the alt name right jeita lang pronunciation index.
   * @return  Returns the index for the alt name right jeita lang pronunciation.
   */
  uint32_t alt_name_right_pronunciation_jeita_lang_index() const {
    return alt_name_right_pronunciation_jeita_lang_index_;
  }

  /**
   * Sets the index for alt name forward jeita pronunciation
   * @param  idx  Index for the alt name forward jeita pronunciation.
   */
  void set_alt_name_forward_pronunciation_jeita_index(const uint32_t idx) {
    alt_name_forward_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the alt name forward jeita pronunciation index.
   * @return  Returns the index for the alt name forward jeita pronunciation.
   */
  uint32_t alt_name_forward_pronunciation_jeita_index() const {
    return alt_name_forward_pronunciation_jeita_index_;
  }

  /**
   * Sets the index for alt name forward jeita lang pronunciation
   * @param  idx  Index for the alt name forward jeita lang pronunciation.
   */
  void set_alt_name_forward_pronunciation_jeita_lang_index(const uint32_t idx) {
    alt_name_forward_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the alt name forward jeita lang pronunciation index.
   * @return  Returns the index for the alt name forward jeita lang pronunciation.
   */
  uint32_t alt_name_forward_pronunciation_jeita_lang_index() const {
    return alt_name_forward_pronunciation_jeita_lang_index_;
  }

  /**
   * Sets the index for alt name backward jeita pronunciation
   * @param  idx  Index for the alt name backward jeita pronunciation.
   */
  void set_alt_name_backward_pronunciation_jeita_index(const uint32_t idx) {
    alt_name_backward_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the alt name backward jeita pronunciation index.
   * @return  Returns the index for the alt name backward jeita pronunciation.
   */
  uint32_t alt_name_backward_pronunciation_jeita_index() const {
    return alt_name_backward_pronunciation_jeita_index_;
  }

  /**
   * Sets the index for alt name backward jeita lang pronunciation
   * @param  idx  Index for the alt name backward jeita lang pronunciation.
   */
  void set_alt_name_backward_pronunciation_jeita_lang_index(const uint32_t idx) {
    alt_name_backward_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the alt name backward jeita lang pronunciation index.
   * @return  Returns the index for the alt name backward jeita lang pronunciation.
   */
  uint32_t alt_name_backward_pronunciation_jeita_lang_index() const {
    return alt_name_backward_pronunciation_jeita_lang_index_;
  }

  /**
   * Sets the index for official name jeita pronunciation
   * @param  idx  Index for the official name jeita pronunciation.
   */
  void set_official_name_pronunciation_jeita_index(const uint32_t idx) {
    official_name_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the official name jeita pronunciation index.
   * @return  Returns the index for the official name jeita pronunciation.
   */
  uint32_t official_name_pronunciation_jeita_index() const {
    return official_name_pronunciation_jeita_index_;
  }

  /**
   * Sets the index for official name jeita lang pronunciation
   * @param  idx  Index for the official name jeita lang pronunciation.
   */
  void set_official_name_pronunciation_jeita_lang_index(const uint32_t idx) {
    official_name_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the official name jeita pronunciation lang index.
   * @return  Returns the index for the official name jeita lang pronunciation.
   */
  uint32_t official_name_pronunciation_jeita_lang_index() const {
    return official_name_pronunciation_jeita_lang_index_;
  }

  /**
   * Sets the index for official name left jeita pronunciation
   * @param  idx  Index for the official name left jeita pronunciation.
   */
  void set_official_name_left_pronunciation_jeita_index(const uint32_t idx) {
    official_name_left_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the official name left jeita pronunciation index.
   * @return  Returns the index for the official name left jeita pronunciation.
   */
  uint32_t official_name_left_pronunciation_jeita_index() const {
    return official_name_left_pronunciation_jeita_index_;
  }

  /**
   * Sets the index for official name left jeita lang pronunciation
   * @param  idx  Index for the official name left jeita lang pronunciation.
   */
  void set_official_name_left_pronunciation_jeita_lang_index(const uint32_t idx) {
    official_name_left_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the official name left jeita lang pronunciation index.
   * @return  Returns the index for the official name left jeita lang pronunciation.
   */
  uint32_t official_name_left_pronunciation_jeita_lang_index() const {
    return official_name_left_pronunciation_jeita_lang_index_;
  }

  /**
   * Sets the index for official name right jeita pronunciation
   * @param  idx  Index for the official name right jeita pronunciation.
   */
  void set_official_name_right_pronunciation_jeita_index(const uint32_t idx) {
    official_name_right_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the official name right jeita pronunciation index.
   * @return  Returns the index for the official name right jeita pronunciation.
   */
  uint32_t official_name_right_pronunciation_jeita_index() const {
    return official_name_right_pronunciation_jeita_index_;
  }

  /**
   * Sets the index for official name right jeita lang pronunciation
   * @param  idx  Index for the official name right jeita lang pronunciation.
   */
  void set_official_name_right_pronunciation_jeita_lang_index(const uint32_t idx) {
    official_name_right_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the official name right jeita lang pronunciation index.
   * @return  Returns the index for the official name right jeita lang pronunciation.
   */
  uint32_t official_name_right_pronunciation_jeita_lang_index() const {
    return official_name_right_pronunciation_jeita_lang_index_;
  }

  /**
   * Sets the index for official name forward jeita pronunciation
   * @param  idx  Index for the official name forward jeita pronunciation.
   */
  void set_official_name_forward_pronunciation_jeita_index(const uint32_t idx) {
    official_name_forward_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the official name forward jeita pronunciation index.
   * @return  Returns the index for the official name forward jeita pronunciation.
   */
  uint32_t official_name_forward_pronunciation_jeita_index() const {
    return official_name_forward_pronunciation_jeita_index_;
  }

  /**
   * Sets the index for official name forward jeita lang pronunciation
   * @param  idx  Index for the official name forward jeita lang pronunciation.
   */
  void set_official_name_forward_pronunciation_jeita_lang_index(const uint32_t idx) {
    official_name_forward_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the official name forward jeita lang pronunciation index.
   * @return  Returns the index for the official name forward jeita lang pronunciation.
   */
  uint32_t official_name_forward_pronunciation_jeita_lang_index() const {
    return official_name_forward_pronunciation_jeita_lang_index_;
  }

  /**
   * Sets the index for official name backward jeita pronunciation
   * @param  idx  Index for the official name backward jeita pronunciation.
   */
  void set_official_name_backward_pronunciation_jeita_index(const uint32_t idx) {
    official_name_backward_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the official name backward jeita pronunciation index.
   * @return  Returns the index for the official name backward jeita pronunciation.
   */
  uint32_t official_name_backward_pronunciation_jeita_index() const {
    return official_name_backward_pronunciation_jeita_index_;
  }

  /**
   * Sets the index for official name backward jeita lang pronunciation
   * @param  idx  Index for the official name backward jeita lang pronunciation.
   */
  void set_official_name_backward_pronunciation_jeita_lang_index(const uint32_t idx) {
    official_name_backward_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the official name backward jeita lang pronunciation index.
   * @return  Returns the index for the official name backward jeita lang pronunciation.
   */
  uint32_t official_name_backward_pronunciation_jeita_lang_index() const {
    return official_name_backward_pronunciation_jeita_lang_index_;
  }

  /**
   * Sets the index for tunnel name jeita pronunciation
   * @param  idx  Index for the tunnel name jeita pronunciation.
   */
  void set_tunnel_name_pronunciation_jeita_index(const uint32_t idx) {
    tunnel_name_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the tunnel name jeita pronunciation index.
   * @return  Returns the index for the tunnel name jeita pronunciation.
   */
  uint32_t tunnel_name_pronunciation_jeita_index() const {
    return tunnel_name_pronunciation_jeita_index_;
  }

  /**
   * Sets the index for tunnel name jeita lang pronunciation
   * @param  idx  Index for the tunnel name jeita lang pronunciation.
   */
  void set_tunnel_name_pronunciation_jeita_lang_index(const uint32_t idx) {
    tunnel_name_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the tunnel name jeita pronunciation lang index.
   * @return  Returns the index for the tunnel name jeita lang pronunciation.
   */
  uint32_t tunnel_name_pronunciation_jeita_lang_index() const {
    return tunnel_name_pronunciation_jeita_lang_index_;
  }

  /**
   * Sets the index for tunnel name left jeita pronunciation
   * @param  idx  Index for the tunnel name left jeita pronunciation.
   */
  void set_tunnel_name_left_pronunciation_jeita_index(const uint32_t idx) {
    tunnel_name_left_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the tunnel name left jeita pronunciation index.
   * @return  Returns the index for the tunnel name left jeita pronunciation.
   */
  uint32_t tunnel_name_left_pronunciation_jeita_index() const {
    return tunnel_name_left_pronunciation_jeita_index_;
  }

  /**
   * Sets the index for tunnel name left jeita lang pronunciation
   * @param  idx  Index for the tunnel name left jeita lang pronunciation.
   */
  void set_tunnel_name_left_pronunciation_jeita_lang_index(const uint32_t idx) {
    tunnel_name_left_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the tunnel name left jeita lang pronunciation index.
   * @return  Returns the index for the tunnel name left jeita lang pronunciation.
   */
  uint32_t tunnel_name_left_pronunciation_jeita_lang_index() const {
    return tunnel_name_left_pronunciation_jeita_lang_index_;
  }

  /**
   * Sets the index for tunnel name right jeita pronunciation
   * @param  idx  Index for the tunnel name right jeita pronunciation.
   */
  void set_tunnel_name_right_pronunciation_jeita_index(const uint32_t idx) {
    tunnel_name_right_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the tunnel name right jeita pronunciation index.
   * @return  Returns the index for the tunnel name right jeita pronunciation.
   */
  uint32_t tunnel_name_right_pronunciation_jeita_index() const {
    return tunnel_name_right_pronunciation_jeita_index_;
  }

  /**
   * Sets the index for tunnel name right jeita lang pronunciation
   * @param  idx  Index for the tunnel name right jeita lang pronunciation.
   */
  void set_tunnel_name_right_pronunciation_jeita_lang_index(const uint32_t idx) {
    tunnel_name_right_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the tunnel name right jeita lang pronunciation index.
   * @return  Returns the index for the tunnel name right jeita lang pronunciation.
   */
  uint32_t tunnel_name_right_pronunciation_jeita_lang_index() const {
    return tunnel_name_right_pronunciation_jeita_lang_index_;
  }

  /**
   * Sets the index for tunnel name forward jeita pronunciation
   * @param  idx  Index for the tunnel name forward jeita pronunciation.
   */
  void set_tunnel_name_forward_pronunciation_jeita_index(const uint32_t idx) {
    tunnel_name_forward_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the tunnel name forward jeita pronunciation index.
   * @return  Returns the index for the tunnel name forward jeita pronunciation.
   */
  uint32_t tunnel_name_forward_pronunciation_jeita_index() const {
    return tunnel_name_forward_pronunciation_jeita_index_;
  }

  /**
   * Sets the index for tunnel name forward jeita lang pronunciation
   * @param  idx  Index for the tunnel name forward jeita lang pronunciation.
   */
  void set_tunnel_name_forward_pronunciation_jeita_lang_index(const uint32_t idx) {
    tunnel_name_forward_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the tunnel name forward jeita lang pronunciation index.
   * @return  Returns the index for the tunnel name forward jeita lang pronunciation.
   */
  uint32_t tunnel_name_forward_pronunciation_jeita_lang_index() const {
    return tunnel_name_forward_pronunciation_jeita_lang_index_;
  }

  /**
   * Sets the index for tunnel name backward jeita pronunciation
   * @param  idx  Index for the tunnel name backward jeita pronunciation.
   */
  void set_tunnel_name_backward_pronunciation_jeita_index(const uint32_t idx) {
    tunnel_name_backward_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the tunnel name backward jeita pronunciation index.
   * @return  Returns the index for the tunnel name backward jeita pronunciation.
   */
  uint32_t tunnel_name_backward_pronunciation_jeita_index() const {
    return tunnel_name_backward_pronunciation_jeita_index_;
  }

  /**
   * Sets the index for tunnel name backward jeita lang pronunciation
   * @param  idx  Index for the tunnel name backward jeita lang pronunciation.
   */
  void set_tunnel_name_backward_pronunciation_jeita_lang_index(const uint32_t idx) {
    tunnel_name_backward_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the tunnel name backward jeita lang pronunciation index.
   * @return  Returns the index for the tunnel name backward jeita lang pronunciation.
   */
  uint32_t tunnel_name_backward_pronunciation_jeita_lang_index() const {
    return tunnel_name_backward_pronunciation_jeita_lang_index_;
  }

  /**
   * Sets the index for destination jeita pronunciation.
   * @param  idx  Index for the destination jeita pronunciation.
   */
  void set_destination_pronunciation_jeita_index(const uint32_t idx) {
    destination_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the get_destination jeita pronunciation index.
   * @return  Returns the index for the destination jeita pronunciation.
   */
  uint32_t destination_pronunciation_jeita_index() const {
    return destination_pronunciation_jeita_index_;
  }

  /**
   * Sets the index for destination jeita lang pronunciation.
   * @param  idx  Index for the destination jeita lang pronunciation.
   */
  void set_destination_pronunciation_jeita_lang_index(const uint32_t idx) {
    destination_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the get_destination jeita lang pronunciation index.
   * @return  Returns the index for the destination jeita lang pronunciation.
   */
  uint32_t destination_pronunciation_jeita_lang_index() const {
    return destination_pronunciation_jeita_lang_index_;
  }

  /**
   * Sets the index for destination in forward direction jeita pronunciation.
   * @param  idx  Index for the destination jeita pronunciation.
   */
  void set_destination_forward_pronunciation_jeita_index(const uint32_t idx) {
    destination_forward_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the forward direction jeita pronunciation index.
   * @return  Returns the index for the forward direction jeita pronunciation.
   */
  uint32_t destination_forward_pronunciation_jeita_index() const {
    return destination_forward_pronunciation_jeita_index_;
  }

  /**
   * Sets the index for destination in forward direction jeita lang pronunciation.
   * @param  idx  Index for the destination jeita lang pronunciation.
   */
  void set_destination_forward_pronunciation_jeita_lang_index(const uint32_t idx) {
    destination_forward_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the forward direction jeita lang pronunciation index.
   * @return  Returns the index for the forward direction jeita lang pronunciation.
   */
  uint32_t destination_forward_pronunciation_jeita_lang_index() const {
    return destination_forward_pronunciation_jeita_lang_index_;
  }

  /**
   * Sets the index for destination in backward direction jeita pronunciation.
   * @param  idx  Index for the backward direction jeita pronunciation.
   */
  void set_destination_backward_pronunciation_jeita_index(const uint32_t idx) {
    destination_backward_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the backward direction jeita pronunciation index.
   * @return  Returns the index for the backward direction jeita pronunciation.
   */
  uint32_t destination_backward_pronunciation_jeita_index() const {
    return destination_backward_pronunciation_jeita_index_;
  }

  /**
   * Sets the index for destination in backward direction jeita lang pronunciation.
   * @param  idx  Index for the backward direction jeita lang pronunciation.
   */
  void set_destination_backward_pronunciation_jeita_lang_index(const uint32_t idx) {
    destination_backward_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the backward direction jeita lang pronunciation index.
   * @return  Returns the index for the backward direction jeita lang pronunciation.
   */
  uint32_t destination_backward_pronunciation_jeita_lang_index() const {
    return destination_backward_pronunciation_jeita_lang_index_;
  }

  /**
   * Sets the index for destination ref jeita pronunciation.
   * @param  idx  Index for the destination ref jeita pronunciation.
   */
  void set_destination_ref_pronunciation_jeita_index(const uint32_t idx) {
    destination_ref_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the destination ref jeita pronunciation index.
   * @return  Returns the index for the destination ref jeita pronunciation.
   */
  uint32_t destination_ref_pronunciation_jeita_index() const {
    return destination_ref_pronunciation_jeita_index_;
  }

  /**
   * Sets the index for destination ref jeita lang pronunciation.
   * @param  idx  Index for the destination ref jeita lang pronunciation.
   */
  void set_destination_ref_pronunciation_jeita_lang_index(const uint32_t idx) {
    destination_ref_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the destination ref jeita lang pronunciation index.
   * @return  Returns the index for the destination ref jeita lang pronunciation.
   */
  uint32_t destination_ref_pronunciation_jeita_lang_index() const {
    return destination_ref_pronunciation_jeita_lang_index_;
  }

  /**
   * Sets the index for destination ref to jeita pronunciation.
   * @param  idx  Index for the destination ref to jeita pronunciation.
   */
  void set_destination_ref_to_pronunciation_jeita_index(const uint32_t idx) {
    destination_ref_to_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the destination ref to jeita pronunciation index.
   * @return  Returns the index for the destination ref to jeita pronunciation.
   */
  uint32_t destination_ref_to_pronunciation_jeita_index() const {
    return destination_ref_to_pronunciation_jeita_index_;
  }

  /**
   * Sets the index for destination ref to jeita lang pronunciation.
   * @param  idx  Index for the destination ref to jeita lang pronunciation.
   */
  void set_destination_ref_to_pronunciation_jeita_lang_index(const uint32_t idx) {
    destination_ref_to_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the destination ref to jeita lang pronunciation index.
   * @return  Returns the index for the destination ref to jeita lang pronunciation.
   */
  uint32_t destination_ref_to_pronunciation_jeita_lang_index() const {
    return destination_ref_to_pronunciation_jeita_lang_index_;
  }

  /**
   * Sets the index for destination street jeita pronunciation.
   * @param  idx  Index for the destination street jeita pronunciation.
   */
  void set_destination_street_pronunciation_jeita_index(const uint32_t idx) {
    destination_street_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the destination_street jeita pronunciation index.
   * @return  Returns the index for the destination street jeita pronunciation.
   */
  uint32_t destination_street_pronunciation_jeita_index() const {
    return destination_street_pronunciation_jeita_index_;
  }

  /**
   * Sets the index for destination street jeita lang pronunciation.
   * @param  idx  Index for the destination street jeita lang pronunciation.
   */
  void set_destination_street_pronunciation_jeita_lang_index(const uint32_t idx) {
    destination_street_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the destination_street jeita lang pronunciation index.
   * @return  Returns the index for the destination street jeita lang pronunciation.
   */
  uint32_t destination_street_pronunciation_jeita_lang_index() const {
    return destination_street_pronunciation_jeita_lang_index_;
  }

  /**
   * Sets the index for destination street to jeita pronunciation.
   * @param  idx  Index for the destination street to jeita pronunciation.
   */
  void set_destination_street_to_pronunciation_jeita_index(const uint32_t idx) {
    destination_street_to_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the destination street to jeita pronunciation index.
   * @return  Returns the index for the destination street to jeita pronunciation.
   */
  uint32_t destination_street_to_pronunciation_jeita_index() const {
    return destination_street_to_pronunciation_jeita_index_;
  }

  /**
   * Sets the index for destination street to jeita lang pronunciation.
   * @param  idx  Index for the destination street to jeita lang pronunciation.
   */
  void set_destination_street_to_pronunciation_jeita_lang_index(const uint32_t idx) {
    destination_street_to_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the destination street to jeita lang pronunciation index.
   * @return  Returns the index for the destination street to jeita lang pronunciation.
   */
  uint32_t destination_street_to_pronunciation_jeita_lang_index() const {
    return destination_street_to_pronunciation_jeita_lang_index_;
  }

  /**
   * Sets the index for junction ref jeita pronunciation.
   * @param  idx  Index for the junction ref jeita pronunciation.
   */
  void set_junction_ref_pronunciation_jeita_index(const uint32_t idx) {
    junction_ref_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the junction ref jeita pronunciation index.
   * @return  Returns the jeita pronunciation index for the junction ref.
   */
  uint32_t junction_ref_pronunciation_jeita_index() const {
    return junction_ref_pronunciation_jeita_index_;
  }

  /**
   * Sets the index for junction ref jeita lang pronunciation.
   * @param  idx  Index for the junction ref jeita lang pronunciation.
   */
  void set_junction_ref_pronunciation_jeita_lang_index(const uint32_t idx) {
    junction_ref_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the junction ref jeita lang pronunciation index.
   * @return  Returns the jeita lang pronunciation index for the junction ref.
   */
  uint32_t junction_ref_pronunciation_jeita_lang_index() const {
    return junction_ref_pronunciation_jeita_lang_index_;
  }

  /**
   * Sets the index for junction name jeita pronunciation.
   * @param  idx  Index for the junction name jeita pronunciation.
   */
  void set_junction_name_pronunciation_jeita_index(const uint32_t idx) {
    junction_name_pronunciation_jeita_index_ = idx;
  }

  /**
   * Get the junction name jeita pronunciation index.
   * @return  Returns the jeita pronunciation index for the junction name.
   */
  uint32_t junction_name_pronunciation_jeita_index() const {
    return junction_name_pronunciation_jeita_index_;
  }

  /**
   * Sets the index for junction name jeita lang pronunciation.
   * @param  idx  Index for the junction name jeita lang pronunciation.
   */
  void set_junction_name_pronunciation_jeita_lang_index(const uint32_t idx) {
    junction_name_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the junction name jeita lang pronunciation index.
   * @return  Returns the jeita lang pronunciation index for the junction name.
   */
  uint32_t junction_name_pronunciation_jeita_lang_index() const {
    return junction_name_pronunciation_jeita_lang_index_;
  }

  // OSM way Id
  uint64_t osmwayid_;

  // name and ref ipa pronunciations
  uint32_t ref_pronunciation_ipa_index_;
  uint32_t ref_pronunciation_ipa_lang_index_;
  uint32_t ref_left_pronunciation_ipa_index_;
  uint32_t ref_left_pronunciation_ipa_lang_index_;
  uint32_t ref_right_pronunciation_ipa_index_;
  uint32_t ref_right_pronunciation_ipa_lang_index_;
  uint32_t int_ref_pronunciation_ipa_index_;
  uint32_t int_ref_pronunciation_ipa_lang_index_;
  uint32_t int_ref_left_pronunciation_ipa_index_;
  uint32_t int_ref_left_pronunciation_ipa_lang_index_;
  uint32_t int_ref_right_pronunciation_ipa_index_;
  uint32_t int_ref_right_pronunciation_ipa_lang_index_;
  uint32_t name_pronunciation_ipa_index_;
  uint32_t name_pronunciation_ipa_lang_index_;
  uint32_t name_left_pronunciation_ipa_index_;
  uint32_t name_left_pronunciation_ipa_lang_index_;
  uint32_t name_right_pronunciation_ipa_index_;
  uint32_t name_right_pronunciation_ipa_lang_index_;
  uint32_t name_forward_pronunciation_ipa_index_;
  uint32_t name_forward_pronunciation_ipa_lang_index_;
  uint32_t name_backward_pronunciation_ipa_index_;
  uint32_t name_backward_pronunciation_ipa_lang_index_;
  uint32_t alt_name_pronunciation_ipa_index_;
  uint32_t alt_name_pronunciation_ipa_lang_index_;
  uint32_t alt_name_left_pronunciation_ipa_index_;
  uint32_t alt_name_left_pronunciation_ipa_lang_index_;
  uint32_t alt_name_right_pronunciation_ipa_index_;
  uint32_t alt_name_right_pronunciation_ipa_lang_index_;
  uint32_t alt_name_forward_pronunciation_ipa_index_;
  uint32_t alt_name_forward_pronunciation_ipa_lang_index_;
  uint32_t alt_name_backward_pronunciation_ipa_index_;
  uint32_t alt_name_backward_pronunciation_ipa_lang_index_;
  uint32_t official_name_pronunciation_ipa_index_;
  uint32_t official_name_pronunciation_ipa_lang_index_;
  uint32_t official_name_left_pronunciation_ipa_index_;
  uint32_t official_name_left_pronunciation_ipa_lang_index_;
  uint32_t official_name_right_pronunciation_ipa_index_;
  uint32_t official_name_right_pronunciation_ipa_lang_index_;
  uint32_t official_name_forward_pronunciation_ipa_index_;
  uint32_t official_name_forward_pronunciation_ipa_lang_index_;
  uint32_t official_name_backward_pronunciation_ipa_index_;
  uint32_t official_name_backward_pronunciation_ipa_lang_index_;
  uint32_t tunnel_name_pronunciation_ipa_index_;
  uint32_t tunnel_name_pronunciation_ipa_lang_index_;
  uint32_t tunnel_name_left_pronunciation_ipa_index_;
  uint32_t tunnel_name_left_pronunciation_ipa_lang_index_;
  uint32_t tunnel_name_right_pronunciation_ipa_index_;
  uint32_t tunnel_name_right_pronunciation_ipa_lang_index_;
  uint32_t tunnel_name_forward_pronunciation_ipa_index_;
  uint32_t tunnel_name_forward_pronunciation_ipa_lang_index_;
  uint32_t tunnel_name_backward_pronunciation_ipa_index_;
  uint32_t tunnel_name_backward_pronunciation_ipa_lang_index_;
  uint32_t direction_pronunciation_ipa_index_;
  uint32_t direction_pronunciation_ipa_lang_index_;
  uint32_t int_direction_pronunciation_ipa_index_;
  uint32_t int_direction_pronunciation_lang_ipa_index_;

  // Sign Destination ipa pronunciations
  uint32_t destination_pronunciation_ipa_index_;
  uint32_t destination_pronunciation_ipa_lang_index_;
  uint32_t destination_forward_pronunciation_ipa_index_;
  uint32_t destination_forward_pronunciation_ipa_lang_index_;
  uint32_t destination_backward_pronunciation_ipa_index_;
  uint32_t destination_backward_pronunciation_ipa_lang_index_;
  uint32_t destination_ref_pronunciation_ipa_index_;
  uint32_t destination_ref_pronunciation_ipa_lang_index_;
  uint32_t destination_ref_to_pronunciation_ipa_index_;
  uint32_t destination_ref_to_pronunciation_ipa_lang_index_;
  uint32_t destination_street_pronunciation_ipa_index_;
  uint32_t destination_street_pronunciation_ipa_lang_index_;
  uint32_t destination_street_to_pronunciation_ipa_index_;
  uint32_t destination_street_to_pronunciation_ipa_lang_index_;
  uint32_t junction_ref_pronunciation_ipa_index_;
  uint32_t junction_ref_pronunciation_ipa_lang_index_;
  uint32_t junction_name_pronunciation_ipa_index_;
  uint32_t junction_name_pronunciation_ipa_lang_index_;

  // name and ref nt-sampa pronunciations
  uint32_t ref_pronunciation_nt_sampa_index_;
  uint32_t ref_pronunciation_nt_sampa_lang_index_;
  uint32_t ref_left_pronunciation_nt_sampa_index_;
  uint32_t ref_left_pronunciation_nt_sampa_lang_index_;
  uint32_t ref_right_pronunciation_nt_sampa_index_;
  uint32_t ref_right_pronunciation_nt_sampa_lang_index_;
  uint32_t int_ref_pronunciation_nt_sampa_index_;
  uint32_t int_ref_pronunciation_nt_sampa_lang_index_;
  uint32_t int_ref_left_pronunciation_nt_sampa_index_;
  uint32_t int_ref_left_pronunciation_nt_sampa_lang_index_;
  uint32_t int_ref_right_pronunciation_nt_sampa_index_;
  uint32_t int_ref_right_pronunciation_nt_sampa_lang_index_;
  uint32_t name_pronunciation_nt_sampa_index_;
  uint32_t name_pronunciation_nt_sampa_lang_index_;
  uint32_t name_left_pronunciation_nt_sampa_index_;
  uint32_t name_left_pronunciation_nt_sampa_lang_index_;
  uint32_t name_right_pronunciation_nt_sampa_index_;
  uint32_t name_right_pronunciation_nt_sampa_lang_index_;
  uint32_t name_forward_pronunciation_nt_sampa_index_;
  uint32_t name_forward_pronunciation_nt_sampa_lang_index_;
  uint32_t name_backward_pronunciation_nt_sampa_index_;
  uint32_t name_backward_pronunciation_nt_sampa_lang_index_;
  uint32_t alt_name_pronunciation_nt_sampa_index_;
  uint32_t alt_name_pronunciation_nt_sampa_lang_index_;
  uint32_t alt_name_left_pronunciation_nt_sampa_index_;
  uint32_t alt_name_left_pronunciation_nt_sampa_lang_index_;
  uint32_t alt_name_right_pronunciation_nt_sampa_index_;
  uint32_t alt_name_right_pronunciation_nt_sampa_lang_index_;
  uint32_t alt_name_forward_pronunciation_nt_sampa_index_;
  uint32_t alt_name_forward_pronunciation_nt_sampa_lang_index_;
  uint32_t alt_name_backward_pronunciation_nt_sampa_index_;
  uint32_t alt_name_backward_pronunciation_nt_sampa_lang_index_;
  uint32_t official_name_pronunciation_nt_sampa_index_;
  uint32_t official_name_pronunciation_nt_sampa_lang_index_;
  uint32_t official_name_left_pronunciation_nt_sampa_index_;
  uint32_t official_name_left_pronunciation_nt_sampa_lang_index_;
  uint32_t official_name_right_pronunciation_nt_sampa_index_;
  uint32_t official_name_right_pronunciation_nt_sampa_lang_index_;
  uint32_t official_name_forward_pronunciation_nt_sampa_index_;
  uint32_t official_name_forward_pronunciation_nt_sampa_lang_index_;
  uint32_t official_name_backward_pronunciation_nt_sampa_index_;
  uint32_t official_name_backward_pronunciation_nt_sampa_lang_index_;
  uint32_t tunnel_name_pronunciation_nt_sampa_index_;
  uint32_t tunnel_name_pronunciation_nt_sampa_lang_index_;
  uint32_t tunnel_name_left_pronunciation_nt_sampa_index_;
  uint32_t tunnel_name_left_pronunciation_nt_sampa_lang_index_;
  uint32_t tunnel_name_right_pronunciation_nt_sampa_index_;
  uint32_t tunnel_name_right_pronunciation_nt_sampa_lang_index_;
  uint32_t tunnel_name_forward_pronunciation_nt_sampa_index_;
  uint32_t tunnel_name_forward_pronunciation_nt_sampa_lang_index_;
  uint32_t tunnel_name_backward_pronunciation_nt_sampa_index_;
  uint32_t tunnel_name_backward_pronunciation_nt_sampa_lang_index_;
  uint32_t direction_pronunciation_nt_sampa_index_;
  uint32_t direction_pronunciation_nt_sampa_lang_index_;
  uint32_t int_direction_pronunciation_nt_sampa_index_;
  uint32_t int_direction_pronunciation_lang_nt_sampa_index_;

  // Sign Destination nt-sampa pronunciations
  uint32_t destination_pronunciation_nt_sampa_index_;
  uint32_t destination_pronunciation_nt_sampa_lang_index_;
  uint32_t destination_forward_pronunciation_nt_sampa_index_;
  uint32_t destination_forward_pronunciation_nt_sampa_lang_index_;
  uint32_t destination_backward_pronunciation_nt_sampa_index_;
  uint32_t destination_backward_pronunciation_nt_sampa_lang_index_;
  uint32_t destination_ref_pronunciation_nt_sampa_index_;
  uint32_t destination_ref_pronunciation_nt_sampa_lang_index_;
  uint32_t destination_ref_to_pronunciation_nt_sampa_index_;
  uint32_t destination_ref_to_pronunciation_nt_sampa_lang_index_;
  uint32_t destination_street_pronunciation_nt_sampa_index_;
  uint32_t destination_street_pronunciation_nt_sampa_lang_index_;
  uint32_t destination_street_to_pronunciation_nt_sampa_index_;
  uint32_t destination_street_to_pronunciation_nt_sampa_lang_index_;
  uint32_t junction_ref_pronunciation_nt_sampa_index_;
  uint32_t junction_ref_pronunciation_nt_sampa_lang_index_;
  uint32_t junction_name_pronunciation_nt_sampa_index_;
  uint32_t junction_name_pronunciation_nt_sampa_lang_index_;

  // name and ref katakana pronunciations
  uint32_t ref_pronunciation_katakana_index_;
  uint32_t ref_pronunciation_katakana_lang_index_;
  uint32_t ref_left_pronunciation_katakana_index_;
  uint32_t ref_left_pronunciation_katakana_lang_index_;
  uint32_t ref_right_pronunciation_katakana_index_;
  uint32_t ref_right_pronunciation_katakana_lang_index_;
  uint32_t int_ref_pronunciation_katakana_index_;
  uint32_t int_ref_pronunciation_katakana_lang_index_;
  uint32_t int_ref_left_pronunciation_katakana_index_;
  uint32_t int_ref_left_pronunciation_katakana_lang_index_;
  uint32_t int_ref_right_pronunciation_katakana_index_;
  uint32_t int_ref_right_pronunciation_katakana_lang_index_;
  uint32_t name_pronunciation_katakana_index_;
  uint32_t name_pronunciation_katakana_lang_index_;
  uint32_t name_left_pronunciation_katakana_index_;
  uint32_t name_left_pronunciation_katakana_lang_index_;
  uint32_t name_right_pronunciation_katakana_index_;
  uint32_t name_right_pronunciation_katakana_lang_index_;
  uint32_t name_forward_pronunciation_katakana_index_;
  uint32_t name_forward_pronunciation_katakana_lang_index_;
  uint32_t name_backward_pronunciation_katakana_index_;
  uint32_t name_backward_pronunciation_katakana_lang_index_;
  uint32_t alt_name_pronunciation_katakana_index_;
  uint32_t alt_name_pronunciation_katakana_lang_index_;
  uint32_t alt_name_left_pronunciation_katakana_index_;
  uint32_t alt_name_left_pronunciation_katakana_lang_index_;
  uint32_t alt_name_right_pronunciation_katakana_index_;
  uint32_t alt_name_right_pronunciation_katakana_lang_index_;
  uint32_t alt_name_forward_pronunciation_katakana_index_;
  uint32_t alt_name_forward_pronunciation_katakana_lang_index_;
  uint32_t alt_name_backward_pronunciation_katakana_index_;
  uint32_t alt_name_backward_pronunciation_katakana_lang_index_;
  uint32_t official_name_pronunciation_katakana_index_;
  uint32_t official_name_pronunciation_katakana_lang_index_;
  uint32_t official_name_left_pronunciation_katakana_index_;
  uint32_t official_name_left_pronunciation_katakana_lang_index_;
  uint32_t official_name_right_pronunciation_katakana_index_;
  uint32_t official_name_right_pronunciation_katakana_lang_index_;
  uint32_t official_name_forward_pronunciation_katakana_index_;
  uint32_t official_name_forward_pronunciation_katakana_lang_index_;
  uint32_t official_name_backward_pronunciation_katakana_index_;
  uint32_t official_name_backward_pronunciation_katakana_lang_index_;
  uint32_t tunnel_name_pronunciation_katakana_index_;
  uint32_t tunnel_name_pronunciation_katakana_lang_index_;
  uint32_t tunnel_name_left_pronunciation_katakana_index_;
  uint32_t tunnel_name_left_pronunciation_katakana_lang_index_;
  uint32_t tunnel_name_right_pronunciation_katakana_index_;
  uint32_t tunnel_name_right_pronunciation_katakana_lang_index_;
  uint32_t tunnel_name_forward_pronunciation_katakana_index_;
  uint32_t tunnel_name_forward_pronunciation_katakana_lang_index_;
  uint32_t tunnel_name_backward_pronunciation_katakana_index_;
  uint32_t tunnel_name_backward_pronunciation_katakana_lang_index_;
  uint32_t direction_pronunciation_katakana_index_;
  uint32_t direction_pronunciation_katakana_lang_index_;
  uint32_t int_direction_pronunciation_katakana_index_;
  uint32_t int_direction_pronunciation_lang_katakana_index_;

  // Sign Destination katakana pronunciations
  uint32_t destination_pronunciation_katakana_index_;
  uint32_t destination_pronunciation_katakana_lang_index_;
  uint32_t destination_forward_pronunciation_katakana_index_;
  uint32_t destination_forward_pronunciation_katakana_lang_index_;
  uint32_t destination_backward_pronunciation_katakana_index_;
  uint32_t destination_backward_pronunciation_katakana_lang_index_;
  uint32_t destination_ref_pronunciation_katakana_index_;
  uint32_t destination_ref_pronunciation_katakana_lang_index_;
  uint32_t destination_ref_to_pronunciation_katakana_index_;
  uint32_t destination_ref_to_pronunciation_katakana_lang_index_;
  uint32_t destination_street_pronunciation_katakana_index_;
  uint32_t destination_street_pronunciation_katakana_lang_index_;
  uint32_t destination_street_to_pronunciation_katakana_index_;
  uint32_t destination_street_to_pronunciation_katakana_lang_index_;
  uint32_t junction_ref_pronunciation_katakana_index_;
  uint32_t junction_ref_pronunciation_katakana_lang_index_;
  uint32_t junction_name_pronunciation_katakana_index_;
  uint32_t junction_name_pronunciation_katakana_lang_index_;

  // name and ref jeita pronunciations
  uint32_t ref_pronunciation_jeita_index_;
  uint32_t ref_pronunciation_jeita_lang_index_;
  uint32_t ref_left_pronunciation_jeita_index_;
  uint32_t ref_left_pronunciation_jeita_lang_index_;
  uint32_t ref_right_pronunciation_jeita_index_;
  uint32_t ref_right_pronunciation_jeita_lang_index_;
  uint32_t int_ref_pronunciation_jeita_index_;
  uint32_t int_ref_pronunciation_jeita_lang_index_;
  uint32_t int_ref_left_pronunciation_jeita_index_;
  uint32_t int_ref_left_pronunciation_jeita_lang_index_;
  uint32_t int_ref_right_pronunciation_jeita_index_;
  uint32_t int_ref_right_pronunciation_jeita_lang_index_;
  uint32_t name_pronunciation_jeita_index_;
  uint32_t name_pronunciation_jeita_lang_index_;
  uint32_t name_left_pronunciation_jeita_index_;
  uint32_t name_left_pronunciation_jeita_lang_index_;
  uint32_t name_right_pronunciation_jeita_index_;
  uint32_t name_right_pronunciation_jeita_lang_index_;
  uint32_t name_forward_pronunciation_jeita_index_;
  uint32_t name_forward_pronunciation_jeita_lang_index_;
  uint32_t name_backward_pronunciation_jeita_index_;
  uint32_t name_backward_pronunciation_jeita_lang_index_;
  uint32_t alt_name_pronunciation_jeita_index_;
  uint32_t alt_name_pronunciation_jeita_lang_index_;
  uint32_t alt_name_left_pronunciation_jeita_index_;
  uint32_t alt_name_left_pronunciation_jeita_lang_index_;
  uint32_t alt_name_right_pronunciation_jeita_index_;
  uint32_t alt_name_right_pronunciation_jeita_lang_index_;
  uint32_t alt_name_forward_pronunciation_jeita_index_;
  uint32_t alt_name_forward_pronunciation_jeita_lang_index_;
  uint32_t alt_name_backward_pronunciation_jeita_index_;
  uint32_t alt_name_backward_pronunciation_jeita_lang_index_;
  uint32_t official_name_pronunciation_jeita_index_;
  uint32_t official_name_pronunciation_jeita_lang_index_;
  uint32_t official_name_left_pronunciation_jeita_index_;
  uint32_t official_name_left_pronunciation_jeita_lang_index_;
  uint32_t official_name_right_pronunciation_jeita_index_;
  uint32_t official_name_right_pronunciation_jeita_lang_index_;
  uint32_t official_name_forward_pronunciation_jeita_index_;
  uint32_t official_name_forward_pronunciation_jeita_lang_index_;
  uint32_t official_name_backward_pronunciation_jeita_index_;
  uint32_t official_name_backward_pronunciation_jeita_lang_index_;
  uint32_t tunnel_name_pronunciation_jeita_index_;
  uint32_t tunnel_name_pronunciation_jeita_lang_index_;
  uint32_t tunnel_name_left_pronunciation_jeita_index_;
  uint32_t tunnel_name_left_pronunciation_jeita_lang_index_;
  uint32_t tunnel_name_right_pronunciation_jeita_index_;
  uint32_t tunnel_name_right_pronunciation_jeita_lang_index_;
  uint32_t tunnel_name_forward_pronunciation_jeita_index_;
  uint32_t tunnel_name_forward_pronunciation_jeita_lang_index_;
  uint32_t tunnel_name_backward_pronunciation_jeita_index_;
  uint32_t tunnel_name_backward_pronunciation_jeita_lang_index_;
  uint32_t direction_pronunciation_jeita_index_;
  uint32_t direction_pronunciation_jeita_lang_index_;
  uint32_t int_direction_pronunciation_jeita_index_;
  uint32_t int_direction_pronunciation_lang_jeita_index_;

  // Sign Destination jeita pronunciations
  uint32_t destination_pronunciation_jeita_index_;
  uint32_t destination_pronunciation_jeita_lang_index_;
  uint32_t destination_forward_pronunciation_jeita_index_;
  uint32_t destination_forward_pronunciation_jeita_lang_index_;
  uint32_t destination_backward_pronunciation_jeita_index_;
  uint32_t destination_backward_pronunciation_jeita_lang_index_;
  uint32_t destination_ref_pronunciation_jeita_index_;
  uint32_t destination_ref_pronunciation_jeita_lang_index_;
  uint32_t destination_ref_to_pronunciation_jeita_index_;
  uint32_t destination_ref_to_pronunciation_jeita_lang_index_;
  uint32_t destination_street_pronunciation_jeita_index_;
  uint32_t destination_street_pronunciation_jeita_lang_index_;
  uint32_t destination_street_to_pronunciation_jeita_index_;
  uint32_t destination_street_to_pronunciation_jeita_lang_index_;
  uint32_t junction_ref_pronunciation_jeita_index_;
  uint32_t junction_ref_pronunciation_jeita_lang_index_;
  uint32_t junction_name_pronunciation_jeita_index_;
  uint32_t junction_name_pronunciation_jeita_lang_index_;
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_PBFGRAPHBUILDER_OSMPRONUNCIATION_H
