#include <iostream>
#include <memory>

#include "baldr/verbal_text_formatter.h"
#include "baldr/verbal_text_formatter_us.h"
#include <valhalla/midgard/util.h>

namespace valhalla {
namespace baldr {

VerbalTextFormatterUs::VerbalTextFormatterUs(const std::string& country_code,
                                             const std::string& state_code)
    : VerbalTextFormatter(country_code, state_code) {
}

VerbalTextFormatterUs::~VerbalTextFormatterUs() {
}

std::string VerbalTextFormatterUs::Format(const std::string& text) const {
  std::string verbal_text(text);

  verbal_text = FormInterstateTts(verbal_text);
  verbal_text = FormUsHighwayTts(verbal_text);

  verbal_text = FormThousandTts(verbal_text);
  verbal_text = FormHundredTts(verbal_text);
  verbal_text = FormNumberSplitTts(verbal_text);
  verbal_text = FormLeadingOhTts(verbal_text);

  return verbal_text;
}

std::string VerbalTextFormatterUs::ProcessNumberSplitMatch(
    const std::smatch& m) const {
  std::string tts;
  if (m[1].matched) {
    tts += m[1].str();
  }

  if (m[3].matched) {
    tts += m[2].str();
    tts += m[3].str();
  } else {
    std::string num = m[2].str();
    const size_t step = 2;
    const char space = ' ';
    for (size_t i = (num.size() % 2 == 0) ? step : (step - 1); i < num.size();
        i += step + 1) {
      num.insert(num.begin() + i, space);
    }
    tts += num;
  }

  if (m[4].matched) {
    tts += m[4].str();
  }

  return tts;
}

std::string VerbalTextFormatterUs::FormNumberSplitTts(
    const std::string& source) const {

  std::string tts;
  for (std::sregex_iterator it(source.begin(), source.end(), kUsNumberSplitRegex),
      end_it; it != end_it; ++it) {
    tts += ProcessNumberSplitMatch(*it);
  }
  return tts.empty() ? source : tts;
}

std::string VerbalTextFormatterUs::FormInterstateTts(
    const std::string& source) const {

  std::string tts;
  tts = std::regex_replace(source, kInterstateRegex, kInterstateOutPattern);
  return tts.empty() ? source : tts;
}

std::string VerbalTextFormatterUs::FormUsHighwayTts(
    const std::string& source) const {

  std::string tts;
  tts = std::regex_replace(source, kUsHighwayRegex, kUsHighwayOutPattern);
  return tts.empty() ? source : tts;
}

std::string VerbalTextFormatterUs::FormThousandTts(
    const std::string& source) const {
  std::string tts;
  tts = std::regex_replace(source, kThousandRegex, kThousandOutPattern);
  return tts.empty() ? source : tts;
}

std::string VerbalTextFormatterUs::FormHundredTts(
    const std::string& source) const {

  std::string tts;
  tts = std::regex_replace(source, kHundredRegex, kHundredOutPattern);
  return tts.empty() ? source : tts;
}

std::string VerbalTextFormatterUs::FormLeadingOhTts(
    const std::string& source) const {

  std::string tts;
  tts = std::regex_replace(source, kLeadingOhRegex, kLeadingOhOutPattern);
  return tts.empty() ? source : tts;
}

}
}
