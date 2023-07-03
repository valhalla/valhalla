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
   * @param  type   Sign type.
   *
   * @param  rn     Bool indicating if this sign is a route number.
   * @param  text   Text string.
   */
  SignInfo(const Sign::Type& type,
           const bool rn,
           const bool tagged,
           const bool has_phoneme,
           const uint32_t phoneme_start_index,
           const uint32_t phoneme_count,
           const std::string& text)
      : phoneme_start_index_(phoneme_start_index), phoneme_count_(phoneme_count), type_(type),
        is_route_num_(rn), is_tagged_(tagged), has_phoneme_(has_phoneme), text_(text) {
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
   * Does the sign have a phoneme?
   * @return Returns true the sign has a phoneme?
   */
  bool has_phoneme() const {
    return has_phoneme_;
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
  Sign::Type type_;
  bool is_route_num_;
  bool is_tagged_;
  bool has_phoneme_;

  std::string text_;
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_SIGNINFO_H_
