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
   * @param  has_linguistic       Bool indicating if this has a linguistic record or not.
   * @param  linguistic_start_index  uint32_t. The linguistic start index.
   * @param  linguistic_count        uint32_t. The number of linguistic records
   * @param  text   Text string.
   */
  SignInfo(const Sign::Type& type,
           const bool rn,
           const bool tagged,
           const bool has_linguistic,
           const uint32_t linguistic_start_index,
           const uint32_t linguistic_count,
           const std::string& text)
      : linguistic_start_index_(linguistic_start_index), linguistic_count_(linguistic_count),
        type_(type), is_route_num_(rn), is_tagged_(tagged), has_linguistic_(has_linguistic),
        text_(text) {
  }

  /**
   * Returns the linguistic start index.
   * @return Returns the linguistic start index.
   */
  uint32_t linguistic_start_index() const {
    return linguistic_start_index_;
  }

  /**
   * Returns the linguistic count.
   * @return Returns the linguistic count.
   */
  uint32_t linguistic_count() const {
    return linguistic_count_;
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
   * Does the sign have a linguistic set?
   * @return Returns true the sign has a linguistic set?
   */
  bool has_linguistic() const {
    return has_linguistic_;
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
  uint32_t linguistic_start_index_;
  uint32_t linguistic_count_;

  Sign::Type type_;
  bool is_route_num_;
  bool is_tagged_;
  bool has_linguistic_;

  std::string text_;
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_SIGNINFO_H_
