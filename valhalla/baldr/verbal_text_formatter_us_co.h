#ifndef VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_US_CO_H_
#define VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_US_CO_H_

#include <valhalla/baldr/verbal_text_formatter.h>

#include <string>
#include <array>
#include <utility>

namespace valhalla {
namespace baldr {


const re::regex kColoradoRegex("(\\bCO)([ -])(\\d{1,3})",
                                  re::regex_constants::icase);
const std::string kColoradoOutPattern = "Colorado $3";


/**
 * The Colorado, US specific verbal text formatter class that prepares strings
 * for use with a text-to-speech engine.
 */
class VerbalTextFormatterUsCo : public VerbalTextFormatterUs {
 public:
  VerbalTextFormatterUsCo(const std::string& country_code,
                        const std::string& state_code);

  ~VerbalTextFormatterUsCo();

 protected:

  std::string ProcessStatesTts(const std::string& source) const override;

};

}
}

#endif  // VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_US_CO_H_
