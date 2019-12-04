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
   * @param  text  Text string.
   * @param  is_route_number   boolean indicating if sign element is a reference route number.
   */
  Sign(const std::string& text, const bool is_route_number);

  /**
   * Returns the sign text.
   * @return  Returns the sign text as a const reference to the text string.
   */
  const std::string& text() const;

  /**
   * Returns true if sign element is a reference route number such as: I 81 South or US 322 West.
   * @return true if sign element is a reference route number such as: I 81 South or US 322 West.
   */
  bool is_route_number() const;

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

#ifdef LOGGING_LEVEL_TRACE
  std::string ToParameterString() const;
#endif

  bool operator==(const Sign& rhs) const;

protected:
  std::string text_;
  bool is_route_number_;
  uint32_t consecutive_count_;
};

} // namespace odin
} // namespace valhalla

#endif // VALHALLA_ODIN_SIGN_H_
