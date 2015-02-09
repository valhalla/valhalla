#ifndef VALHALLA_ODIN_SIGN_H_
#define VALHALLA_ODIN_SIGN_H_

#include <valhalla/baldr/sign.h>
#include <valhalla/baldr/signinfo.h>

namespace valhalla {
namespace odin {

class Sign : public baldr::SignInfo {
 public:
  /**
   * Constructor.
   * @param  type   Sign type.
   * @param  text   Text string.
   */
  Sign(const baldr::Sign::Type& type, const std::string& text);

  /**
   * Returns the frequency of this sign within a set a consecutive signs.
   * @return the frequency of this sign within a set a consecutive signs.
   */
  uint32_t consecutive_count() const;

  /**
   * Sets the frequency of this sign within a set a consecutive signs.
   * @param consecutive_count the consecutive count value.
   */
  void set_consecutive_count(uint32_t consecutive_count);

 protected:
  uint32_t consecutive_count_;

};

bool DescendingSortByConsecutiveCount(const Sign& lhs, const Sign& rhs) {
  return lhs.consecutive_count() > rhs.consecutive_count();
}

}
}

#endif  // VALHALLA_ODIN_SIGN_H_
