#include <iostream>
#include <memory>

#include "baldr/verbal_text_formatter.h"
#include "baldr/verbal_text_formatter_us.h"
#include "baldr/verbal_text_formatter_us_co.h"
#include "midgard/util.h"

namespace valhalla {
namespace baldr {

VerbalTextFormatterUsCo::VerbalTextFormatterUsCo(const std::string& country_code,
                                                 const std::string& state_code)
    : VerbalTextFormatterUs(country_code, state_code) {
}

VerbalTextFormatterUsCo::~VerbalTextFormatterUsCo() {
}

std::string VerbalTextFormatterUsCo::ProcessStatesTts(const std::string& source) const {

  std::string tts;
  if (FormStateTts(source, kColoradoRegex, kColoradoOutPattern, tts)) {
    // Colorado has been found and transformed - so return
    return tts;
  }
  return VerbalTextFormatterUs::ProcessStatesTts(source);
}

} // namespace baldr
} // namespace valhalla
