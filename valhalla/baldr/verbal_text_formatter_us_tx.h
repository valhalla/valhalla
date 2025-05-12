#ifndef VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_US_TX_H_
#define VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_US_TX_H_

#include <string>

#include <valhalla/baldr/verbal_text_formatter_us.h>

namespace valhalla {
namespace baldr {

// Farm to Market
const std::regex kFmRegex("(\\bF[ -]?M)([ -])?(\\d{1,4})", std::regex_constants::icase);
const std::string kFmOutPattern = "Farm to Market Road $3";

// Ranch to Market
const std::regex kRmRegex("(\\bR[ -]?M)([ -])?(\\d{1,4})", std::regex_constants::icase);
const std::string kRmOutPattern = "Ranch to Market Road $3";

/**
 * The Texas, US specific verbal text formatter class that prepares strings
 * for use with a text-to-speech engine.
 */
class VerbalTextFormatterUsTx : public VerbalTextFormatterUs {
public:
  VerbalTextFormatterUsTx(const std::string& country_code, const std::string& state_code);

  ~VerbalTextFormatterUsTx();

  /**
   * Returns a Texas, US text-to-speech formatted string based on the specified text.
   *
   * @param  text  the source string to transform.
   *
   * @return a Texas, US text-to-speech formatted string based on the specified text.
   */
  std::string Format(const std::string& text) const override;

protected:
  std::string FormFmTts(const std::string& source) const;

  std::string FormRmTts(const std::string& source) const;
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_US_TX_H_
