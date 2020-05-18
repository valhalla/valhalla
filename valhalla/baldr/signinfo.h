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
  SignInfo(const Sign::Type& type, const bool rn, const std::string& text)
      : type_(type), is_route_num_(rn), text_(text) {
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
   * Returns the sign text.
   * @return  Returns the sign text as a const reference to the text string.
   */
  const std::string& text() const {
    return text_;
  }

protected:
  Sign::Type type_;
  bool is_route_num_;
  std::string text_;
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_SIGNINFO_H_
