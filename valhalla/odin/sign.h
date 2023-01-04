#ifndef VALHALLA_ODIN_SIGN_H_
#define VALHALLA_ODIN_SIGN_H_

#include <cstdint>
#include <optional>
#include <string>

#include <valhalla/baldr/streetname.h>

namespace valhalla {
namespace odin {

class Sign {
public:
  /**
   * Constructor.
   * @param  text  Text string.
   * @param  is_route_number   boolean indicating if sign element is a reference route number.
   * @param  pronunciation  the pronunciation of this sign.
   */
  Sign(const std::string& text,
       const bool is_route_number,
       const std::optional<baldr::Pronunciation>& pronunciation = std::nullopt);

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

  /**
   * Returns the pronunciation of this sign.
   * @return the pronunciation of this sign.
   */
  const std::optional<baldr::Pronunciation>& pronunciation() const;

#ifdef LOGGING_LEVEL_TRACE
  std::string ToParameterString() const;
#endif

  bool operator==(const Sign& rhs) const;

protected:
  std::string text_;
  bool is_route_number_;
  uint32_t consecutive_count_;
  std::optional<baldr::Pronunciation> pronunciation_;
};

} // namespace odin
} // namespace valhalla

#endif // VALHALLA_ODIN_SIGN_H_
