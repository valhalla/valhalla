#include "baldr/verbal_text_formatter_us_tx.h"
#include "baldr/verbal_text_formatter.h"
#include "baldr/verbal_text_formatter_us.h"
#include "midgard/util.h"

namespace valhalla {
namespace baldr {

VerbalTextFormatterUsTx::VerbalTextFormatterUsTx(const std::string& country_code,
                                                 const std::string& state_code)
    : VerbalTextFormatterUs(country_code, state_code) {
}

VerbalTextFormatterUsTx::~VerbalTextFormatterUsTx() {
}

std::string VerbalTextFormatterUsTx::Format(const std::string& text) const {
  std::string verbal_text(text);
  verbal_text = FormFmTts(verbal_text);
  verbal_text = FormRmTts(verbal_text);
  verbal_text = VerbalTextFormatterUs::Format(verbal_text);
  return verbal_text;
}

std::string VerbalTextFormatterUsTx::FormFmTts(const std::string& source) const {
  return std::regex_replace(source, kFmRegex, kFmOutPattern);
}
std::string VerbalTextFormatterUsTx::FormRmTts(const std::string& source) const {
  return std::regex_replace(source, kRmRegex, kRmOutPattern);
}

} // namespace baldr
} // namespace valhalla
