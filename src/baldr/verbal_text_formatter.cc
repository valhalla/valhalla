
#include "baldr/verbal_text_formatter.h"
#include "midgard/util.h"

#include <string>

namespace valhalla {
namespace baldr {

VerbalTextFormatter::VerbalTextFormatter(const std::string& country_code,
                                         const std::string& state_code)
    : country_code_(country_code), state_code_(state_code) {
}

VerbalTextFormatter::~VerbalTextFormatter() {
}

std::string VerbalTextFormatter::Format(const std::string& text) const {
  std::string verbal_text(text);

  verbal_text = FormNumberSplitTts(verbal_text);

  return verbal_text;
}

std::string VerbalTextFormatter::ProcessNumberSplitMatch(const std::smatch& m) const {
  std::string tts;
  if (m[1].matched) {
    tts += m[1].str();
  }

  std::string num = m[2].str();
  const size_t step = 2;
  const char space = ' ';
  for (size_t i = (num.size() % 2 == 0) ? step : (step - 1); i < num.size(); i += step + 1) {
    num.insert(num.begin() + i, space);
  }
  tts += num;

  if (m[3].matched) {
    tts += m[3].str();
  }

  return tts;
}

std::string VerbalTextFormatter::FormNumberSplitTts(const std::string& source) const {

  std::string tts;
  for (std::sregex_iterator it(source.begin(), source.end(), kNumberSplitRegex), end_it; it != end_it;
       ++it) {
    tts += ProcessNumberSplitMatch(*it);
  }
  return tts.empty() ? source : tts;
}

} // namespace baldr
} // namespace valhalla
