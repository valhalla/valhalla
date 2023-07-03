#ifndef VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_FACTORY_H_
#define VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_FACTORY_H_

#include <memory>
#include <string>

#include <valhalla/baldr/verbal_text_formatter.h>

namespace valhalla {
namespace baldr {

/**
 * A factory class that creates a specific VerbalTextFormatter pointer
 * based on the specified country and state codes.
 */
class VerbalTextFormatterFactory {
public:
  VerbalTextFormatterFactory() = delete;

  /**
   * Returns a specific VerbalTextFormatter pointer based on the specified
   * country and state codes.
   *
   * @param  country_code  the country code that will help determine the type
   *                       of verbal formatter. (example: US)
   * @param  state_code  the state code that will help determine the type
   *                      of verbal formatter. (example: PA)
   * @return VerbalTextFormatter unique pointer.
   */
  static std::unique_ptr<VerbalTextFormatter> Create(const std::string& country_code,
                                                     const std::string& state_code);
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_FACTORY_H_
