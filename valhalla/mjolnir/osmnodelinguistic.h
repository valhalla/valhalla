#ifndef VALHALLA_MJOLNIR_OSMNODELINGUISTIC_H_
#define VALHALLA_MJOLNIR_OSMNODELINGUISTIC_H_

#include <cstdint>

namespace valhalla {
namespace mjolnir {

/**
 * OSM node linguistic information.
 */
struct OSMNodeLinguistic {

  OSMNodeLinguistic() {
    memset(this, 0, sizeof(OSMNodeLinguistic));
  }

  /**
   * Sets the index for name:<lang>
   * @param  idx  Index for the languages.
   */
  void set_name_lang_index(const uint32_t idx) {
    name_lang_index_ = idx;
  }

  /**
   * Get the name:<lang> index.
   * @return  Returns the index for the languages.
   */
  uint32_t name_lang_index() const {
    return name_lang_index_;
  }

  /**
   * Sets the index for the name ipa pronunciation
   * @param  idx  Index for the nameerence ipa pronunciation.
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
   * Sets the index for the name ipa lang pronunciation
   * @param  idx  Index for the nameerence ipa lang pronunciation.
   */
  void set_name_pronunciation_ipa_lang_index(const uint32_t idx) {
    name_pronunciation_ipa_lang_index_ = idx;
  }

  /**
   * Get the name ipa pronunciation lang index.
   * @return  Returns the index for the name ipa lang pronunciation.
   */
  uint32_t name_pronunciation_ipa_lang_index() const {
    return name_pronunciation_ipa_lang_index_;
  }

  /**
   * Sets the index for the name nt-sampa pronunciation
   * @param  idx  Index for the nameerence nt-sampa pronunciation.
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
   * Sets the index for the name nt-sampa lang pronunciation
   * @param  idx  Index for the nameerence nt-sampa lang pronunciation.
   */
  void set_name_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    name_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the name nt-sampa pronunciation lang index.
   * @return  Returns the index for the name nt-sampa lang pronunciation.
   */
  uint32_t name_pronunciation_nt_sampa_lang_index() const {
    return name_pronunciation_nt_sampa_lang_index_;
  }

  /**
   * Sets the index for the name katakana pronunciation
   * @param  idx  Index for the nameerence katakana pronunciation.
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
   * Sets the index for the name katakana lang pronunciation
   * @param  idx  Index for the nameerence katakana lang pronunciation.
   */
  void set_name_pronunciation_katakana_lang_index(const uint32_t idx) {
    name_pronunciation_katakana_lang_index_ = idx;
  }

  /**
   * Get the name katakana pronunciation lang index.
   * @return  Returns the index for the name katakana lang pronunciation.
   */
  uint32_t name_pronunciation_katakana_lang_index() const {
    return name_pronunciation_katakana_lang_index_;
  }

  /**
   * Sets the index for the name jeita pronunciation
   * @param  idx  Index for the nameerence jeita pronunciation.
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
   * Sets the index for the name jeita lang pronunciation
   * @param  idx  Index for the nameerence jeita lang pronunciation.
   */
  void set_name_pronunciation_jeita_lang_index(const uint32_t idx) {
    name_pronunciation_jeita_lang_index_ = idx;
  }

  /**
   * Get the name jeita pronunciation lang index.
   * @return  Returns the index for the name jeita lang pronunciation.
   */
  uint32_t name_pronunciation_jeita_lang_index() const {
    return name_pronunciation_jeita_lang_index_;
  }

  /**
   * Sets the index for ref:<lang>
   * @param  idx  Index for the languages.
   */
  void set_ref_lang_index(const uint32_t idx) {
    ref_lang_index_ = idx;
  }

  /**
   * Get the ref:<lang> index.
   * @return  Returns the index for the languages.
   */
  uint32_t ref_lang_index() const {
    return ref_lang_index_;
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
   * Get the ref ipa pronunciation lang index.
   * @return  Returns the index for the ref ipa lang pronunciation.
   */
  uint32_t ref_pronunciation_ipa_lang_index() const {
    return ref_pronunciation_ipa_lang_index_;
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
   * Sets the index for the ref nt-sampa lang pronunciation
   * @param  idx  Index for the reference nt-sampa lang pronunciation.
   */
  void set_ref_pronunciation_nt_sampa_lang_index(const uint32_t idx) {
    ref_pronunciation_nt_sampa_lang_index_ = idx;
  }

  /**
   * Get the ref nt-sampa pronunciation lang index.
   * @return  Returns the index for the ref nt-sampa lang pronunciation.
   */
  uint32_t ref_pronunciation_nt_sampa_lang_index() const {
    return ref_pronunciation_nt_sampa_lang_index_;
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
   * Get the ref katakana pronunciation lang index.
   * @return  Returns the index for the ref katakana lang pronunciation.
   */
  uint32_t ref_pronunciation_katakana_lang_index() const {
    return ref_pronunciation_katakana_lang_index_;
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
   * Get the ref jeita pronunciation lang index.
   * @return  Returns the index for the ref jeita lang pronunciation.
   */
  uint32_t ref_pronunciation_jeita_lang_index() const {
    return ref_pronunciation_jeita_lang_index_;
  }

  /**
   * Has any index been set?
   * @return  True is nothing is set
   */
  bool isEmpty() const {
    return (name_lang_index_ == 0 && name_pronunciation_ipa_index_ == 0 &&
            name_pronunciation_ipa_lang_index_ == 0 && name_pronunciation_nt_sampa_index_ == 0 &&
            name_pronunciation_nt_sampa_lang_index_ == 0 && name_pronunciation_katakana_index_ == 0 &&
            name_pronunciation_katakana_lang_index_ == 0 && name_pronunciation_jeita_index_ == 0 &&
            name_pronunciation_jeita_lang_index_ == 0 && ref_lang_index_ == 0 &&
            ref_pronunciation_ipa_index_ == 0 && ref_pronunciation_ipa_lang_index_ == 0 &&
            ref_pronunciation_nt_sampa_index_ == 0 && ref_pronunciation_nt_sampa_lang_index_ == 0 &&
            ref_pronunciation_katakana_index_ == 0 && ref_pronunciation_katakana_lang_index_ == 0 &&
            ref_pronunciation_jeita_index_ == 0 && ref_pronunciation_jeita_lang_index_ == 0);
  }

  // pronunciations / langs
  uint32_t name_lang_index_;
  uint32_t name_pronunciation_ipa_index_;
  uint32_t name_pronunciation_ipa_lang_index_;
  uint32_t name_pronunciation_nt_sampa_index_;
  uint32_t name_pronunciation_nt_sampa_lang_index_;
  uint32_t name_pronunciation_katakana_index_;
  uint32_t name_pronunciation_katakana_lang_index_;
  uint32_t name_pronunciation_jeita_index_;
  uint32_t name_pronunciation_jeita_lang_index_;
  uint32_t ref_lang_index_;
  uint32_t ref_pronunciation_ipa_index_;
  uint32_t ref_pronunciation_ipa_lang_index_;
  uint32_t ref_pronunciation_nt_sampa_index_;
  uint32_t ref_pronunciation_nt_sampa_lang_index_;
  uint32_t ref_pronunciation_katakana_index_;
  uint32_t ref_pronunciation_katakana_lang_index_;
  uint32_t ref_pronunciation_jeita_index_;
  uint32_t ref_pronunciation_jeita_lang_index_;
};
} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_OSMLINGUISTIC_H
