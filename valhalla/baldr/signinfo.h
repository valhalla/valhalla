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
   * @param  text   Text string.
   */
  SignInfo(const Sign::Type& type, const std::string& text) : type_(type), text_(text) {
  }

  /**
   * Returns the sign type.
   * @return Returns the sign type.
   */
  const Sign::Type& type() const {
    return type_;
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
  std::string text_;
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_SIGNINFO_H_
