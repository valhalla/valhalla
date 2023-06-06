#ifndef VALHALLA_BALDR_SIGNINFO_H_
#define VALHALLA_BALDR_SIGNINFO_H_

#include <valhalla/baldr/sign.h>

namespace valhalla {
namespace baldr {

/**
 * Interface class used to pass information about a sign.
 * Encapsulates the sign type and the associated text.
 */
class SignInfo {
public:
  /**
   * Constructor.
   * @param  type                 Sign type.
   * @param  rn                   Bool indicating if this sign is a route number.
   * @param  tagged               Bool indicating if this sign is a special tagged type.
   * @param  has_phoneme          Bool indicating if this has a phoneme or not.
   * @param  phoneme_start_index  uint32_t. The phoneme start index.
   * @param  phoneme_count        uint32_t. The number of phonemes
   * @param  lang_start_index     uint32_t. The language start index.
   * @param  lang_count           uint32_t. The number of languages
   * @param  text   Text string.
   */
  SignInfo(const Sign::Type& type,
           const bool rn,
           const bool tagged,
           const bool has_phoneme,
           const bool has_lang,
           const uint32_t phoneme_start_index,
           const uint32_t phoneme_count,
           const uint32_t lang_start_index,
           const uint32_t lang_count,
           const std::string& text)
      : phoneme_start_index_(phoneme_start_index), phoneme_count_(phoneme_count),
        lang_start_index_(lang_start_index), lang_count_(lang_count), type_(type), is_route_num_(rn),
        is_tagged_(tagged), has_phoneme_(has_phoneme), has_lang_(has_lang), text_(text) {
  }

  /**
   * Returns the phoneme start index.
   * @return Returns the phoneme start index.
   */
  uint32_t phoneme_start_index() const {
    return phoneme_start_index_;
  }

  /**
   * Returns the phoneme count.
   * @return Returns the phoneme count.
   */
  uint32_t phoneme_count() const {
    return phoneme_count_;
  }

  /**
   * Returns the language start index.
   * @return Returns the language start index.
   */
  uint32_t lang_start_index() const {
    return lang_start_index_;
  }

  /**
   * Returns the lang count.
   * @return Returns the lang count.
   */
  uint32_t lang_count() const {
    return lang_count_;
  }

  /**
   * Returns the sign type.
   * @return Returns the sign type.
   */
  const Sign::Type& type() const {
    return type_;
  }

  /**
   * Does this sign record indicate a route number.
   * @return  Returns true if the sign record is a route number.
   */
  bool is_route_num() const {
    return is_route_num_;
  }

  /**
   * Is the sign text tagged
   * @return Returns true if the sign text is tagged.
   */
  bool is_tagged() const {
    return is_tagged_;
  }

  /**
   * Does the sign have a phoneme set?
   * @return Returns true the sign has a phoneme set?
   */
  bool has_phoneme() const {
    return has_phoneme_;
  }

  /**
   * Does the sign have a language?
   * @return Returns true the sign has a language?
   */
  bool has_language() const {
    return has_lang_;
  }

  /**
   * Returns the sign text.
   * @return  Returns the sign text as a const reference to the text string.
   */
  const std::string& text() const {
    return text_;
  }

  // operator < - for sorting. Sort by type.
  bool operator<(const SignInfo& other) const {
    return type() < other.type();
  }

protected:
  uint32_t phoneme_start_index_;
  uint32_t phoneme_count_;
  uint32_t lang_start_index_;
  uint32_t lang_count_;

  Sign::Type type_;
  bool is_route_num_;
  bool is_tagged_;
  bool has_phoneme_;
  bool has_lang_;

  std::string text_;
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_SIGNINFO_H_
