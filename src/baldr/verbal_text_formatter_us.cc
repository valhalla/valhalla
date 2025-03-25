#include "baldr/verbal_text_formatter_us.h"
#include "baldr/verbal_text_formatter.h"
#include "midgard/util.h"

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
  verbal_text = ProcessStatesTts(verbal_text);
  verbal_text = ProcessCountysTts(verbal_text);
  // TODO township T >> Township or not
  // TODO FM farm to market, others? TR?

  verbal_text = ProcessThousandTts(verbal_text);
  verbal_text = ProcessHundredTts(verbal_text);
  verbal_text = FormNumberSplitTts(verbal_text);
  verbal_text = FormLeadingOhTts(verbal_text);

  return verbal_text;
}

std::string VerbalTextFormatterUs::ProcessNumberSplitMatch(const std::smatch& m) const {
  std::string tts;
  if (m[1].matched) {
    tts += m[1].str();
  }

  // if source number has st,nd,rd,th appended to it
  // then do not split the number just return source value
  if (m[3].matched) {
    tts += m[2].str();
    tts += m[3].str();
  } else {
    std::string num = m[2].str();
    const size_t step = 2;
    const char space = ' ';
    for (size_t i = (num.size() % 2 == 0) ? step : (step - 1); i < num.size(); i += step + 1) {
      num.insert(num.begin() + i, space);
    }
    tts += num;
  }

  if (m[4].matched) {
    tts += m[4].str();
  }

  return tts;
}

std::string VerbalTextFormatterUs::FormNumberSplitTts(const std::string& source) const {

  std::string tts;
  for (std::sregex_iterator it(source.begin(), source.end(), kUsNumberSplitRegex), end_it;
       it != end_it; ++it) {
    tts += ProcessNumberSplitMatch(*it);
  }
  return tts.empty() ? source : tts;
}

std::string VerbalTextFormatterUs::FormInterstateTts(const std::string& source) const {
  return std::regex_replace(source, kInterstateRegex, kInterstateOutPattern);
}

std::string VerbalTextFormatterUs::FormUsHighwayTts(const std::string& source) const {
  return std::regex_replace(source, kUsHighwayRegex, kUsHighwayOutPattern);
}

std::string VerbalTextFormatterUs::ProcessStatesTts(const std::string& source) const {

  std::string tts;
  for (auto& state_find_replace : kStateRoutes) {
    if (FormStateTts(source, state_find_replace.first, state_find_replace.second, tts)) {
      // State has been found and transformed - so return
      return tts;
    }
  }
  // Nothing transformed so return source
  return source;
}

bool VerbalTextFormatterUs::FormStateTts(const std::string& source,
                                         const std::regex& state_regex,
                                         const std::string& state_output_pattern,
                                         std::string& tts) const {

  tts = std::regex_replace(source, state_regex, state_output_pattern);

  // Return true if transformed
  return (tts != source);
}

std::string VerbalTextFormatterUs::ProcessCountysTts(const std::string& source) const {

  std::string tts;
  for (auto& county_find_replace : kCountyRoutes) {
    if (FormCountyTts(source, county_find_replace.first, county_find_replace.second, tts)) {
      // County has been found and transformed - so return
      return tts;
    }
  }
  // Nothing transformed so return source
  return source;
}

bool VerbalTextFormatterUs::FormCountyTts(const std::string& source,
                                          const std::regex& county_regex,
                                          const std::string& county_output_pattern,
                                          std::string& tts) const {

  tts = std::regex_replace(source, county_regex, county_output_pattern);

  // Return true if transformed
  return (tts != source);
}

std::string VerbalTextFormatterUs::ProcessThousandTts(const std::string& source) const {

  std::string tts = source;
  for (auto& thousand_find_replace : kThousandFindReplace) {
    tts = FormThousandTts(tts, thousand_find_replace.first, thousand_find_replace.second);
  }
  return tts;
}

std::string VerbalTextFormatterUs::FormThousandTts(const std::string& source,
                                                   const std::regex& thousand_regex,
                                                   const std::string& thousand_output_pattern) const {
  return std::regex_replace(source, thousand_regex, thousand_output_pattern);
}

std::string VerbalTextFormatterUs::ProcessHundredTts(const std::string& source) const {

  std::string tts = source;
  for (auto& hundred_find_replace : kHundredFindReplace) {
    tts = FormHundredTts(tts, hundred_find_replace.first, hundred_find_replace.second);
  }
  return tts;
}

std::string VerbalTextFormatterUs::FormHundredTts(const std::string& source,
                                                  const std::regex& hundred_regex,
                                                  const std::string& hundred_output_pattern) const {
  return std::regex_replace(source, hundred_regex, hundred_output_pattern);
}

std::string VerbalTextFormatterUs::FormLeadingOhTts(const std::string& source) const {
  return std::regex_replace(source, kLeadingOhRegex, kLeadingOhOutPattern);
}

} // namespace baldr
} // namespace valhalla
