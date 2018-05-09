#ifndef VALHALLA_ODIN_SIGN_H_
#define VALHALLA_ODIN_SIGN_H_

#include <cstdint>
#include <string>

namespace valhalla {
namespace odin {

class Sign {
public:
  /**
   * Constructor.
   * @param  text   Text string.
   */
  Sign(const std::string& text);

  /**
   * Returns the sign text.
   * @return  Returns the sign text as a const reference to the text string.
   */
  const std::string& text() const;

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

  std::string ToParameterString() const;

  bool operator==(const Sign& rhs) const;

protected:
  std::string text_;
  uint32_t consecutive_count_;
};

} // namespace odin
} // namespace valhalla

#endif // VALHALLA_ODIN_SIGN_H_
