#include "baldr/verbal_text_formatter_us.h"

#include <array>

namespace {

const std::regex kUsNumberSplitRegex("(\\D*)(\\d+)(st|nd|rd|th)?(\\D*)", std::regex_constants::icase);

const std::regex kInterstateRegex("(\\bI)([ -])(H)?(\\d{1,3})", std::regex_constants::icase);
const std::string kInterstateOutPattern = "Interstate $3$4";

const std::regex kUsHighwayRegex("(\\bUS)([ -])(Highway )?(\\d{1,3})", std::regex_constants::icase);
const std::string kUsHighwayOutPattern = "U.S. $3$4";

const std::regex kLeadingOhRegex("( )(0)([1-9])");
const std::string kLeadingOhOutPattern = "$1o$3";

const std::array<std::pair<std::regex, std::string>, 4> kThousandFindReplace = {
    {{std::regex("(^|\\D)([1-9]{1,2})(000$)"), "$1$2 thousand"},
     {std::regex("(^|\\D)([1-9]{1,2})(000th)", std::regex_constants::icase), "$1$2 thousandth"},
     {std::regex("(^|\\D)([1-9]{1,2})(000)( |-)"), "$1$2 thousand "},
     {std::regex("(^|\\D)([1-9]{1,2})(000)(\\D)"), "$1$2 thousand $4"}}};

const std::array<std::pair<std::regex, std::string>, 4> kHundredFindReplace = {
    {{std::regex("(^|\\D)([1-9]{1,2})(00$)"), "$1$2 hundred"},
     {std::regex("(^|\\D)([1-9]{1,2})(00th)", std::regex_constants::icase), "$1$2 hundredth"},
     {std::regex("(^|\\D)([1-9]{1,2})(00)( |-)"), "$1$2 hundred "},
     {std::regex("(^|\\D)([1-9]{1,2})(00)(\\D)"), "$1$2 hundred $4"}}};

const std::array<std::pair<std::regex, std::string>, 53> kStateRoutes = {
    {{std::regex("(\\bSR)([ -])?(\\d{1,4})", std::regex_constants::icase), "State Route $3"},
     {std::regex("(\\bSH)([ -])?(\\d{1,4})", std::regex_constants::icase), "State Highway $3"},
     {std::regex("(\\bCA)([ -])(\\d{1,3})", std::regex_constants::icase), "California $3"},
     {std::regex("(\\bTX)([ -])(\\d{1,3})", std::regex_constants::icase), "Texas $3"},
     {std::regex("(\\bFL)([ -])(A)?(\\d{1,3})", std::regex_constants::icase), "Florida $3$4"},
     {std::regex("(\\bNY)([ -])(\\d{1,3})", std::regex_constants::icase), "New York $3"},
     {std::regex("(\\bIL)([ -])(\\d{1,3})", std::regex_constants::icase), "Illinois $3"},
     {std::regex("(\\bPA)([ -])(\\d{1,3})", std::regex_constants::icase), "Pennsylvania $3"},
     {std::regex("(\\bOH)([ -])(\\d{1,3})", std::regex_constants::icase), "Ohio $3"},
     {std::regex("(\\bGA)([ -])(\\d{1,3})", std::regex_constants::icase), "Georgia $3"},
     {std::regex("(\\bNC)([ -])(\\d{1,3})", std::regex_constants::icase), "North Carolina $3"},
     {std::regex("(\\bM)([ -])(\\d{1,3})", std::regex_constants::icase), "Michigan $3"},
     {std::regex("(\\bNJ)([ -])(\\d{1,3})", std::regex_constants::icase), "New Jersey $3"},
     {std::regex("(\\bVA)([ -])(\\d{1,3})", std::regex_constants::icase), "Virginia $3"},
     {std::regex("(\\bWA)([ -])(\\d{1,3})", std::regex_constants::icase), "Washington $3"},
     {std::regex("(\\bMA)([ -])(\\d{1,3})", std::regex_constants::icase), "Massachusetts $3"},
     {std::regex("(\\bAZ)([ -])(\\d{1,3})", std::regex_constants::icase), "Arizona $3"},
     {std::regex("(\\bIN)([ -])(\\d{1,3})", std::regex_constants::icase), "Indiana $3"},
     {std::regex("(\\bTN)([ -])(\\d{1,3})", std::regex_constants::icase), "Tennessee $3"},
     {std::regex("(\\bMO)([ -])(\\d{1,3})", std::regex_constants::icase), "Missouri $3"},
     {std::regex("(\\bMO)([ -])([[:alpha:]]{1,2}\\b)", std::regex_constants::icase), "Missouri $3"},
     {std::regex("(\\bMD)([ -])(\\d{1,3})", std::regex_constants::icase), "Maryland $3"},
     {std::regex("(\\bWI)([ -])(\\d{1,3})", std::regex_constants::icase), "Wisconsin $3"},
     {std::regex("(\\bMN)([ -])(\\d{1,3})", std::regex_constants::icase), "Minnesota $3"},
     {std::regex("(\\bAL)([ -])(\\d{1,3})", std::regex_constants::icase), "Alabama $3"},
     {std::regex("(\\bSC)([ -])(\\d{1,3})", std::regex_constants::icase), "South Carolina $3"},
     {std::regex("(\\bLA)([ -])(\\d{1,4})", std::regex_constants::icase), "Louisiana $3"},
     {std::regex("(\\bKY)([ -])(\\d{1,4})", std::regex_constants::icase), "Kentucky $3"},
     {std::regex("(\\bOR)([ -])(\\d{1,3})", std::regex_constants::icase), "Oregon $3"},
     {std::regex("(\\bOK)([ -])(\\d{1,3})", std::regex_constants::icase), "Oklahoma $3"},
     {std::regex("(\\bCT)([ -])(\\d{1,3})", std::regex_constants::icase), "Connecticut $3"},
     {std::regex("(\\bIA)([ -])(\\d{1,3})", std::regex_constants::icase), "Iowa $3"},
     {std::regex("(\\bMS)([ -])(\\d{1,3})", std::regex_constants::icase), "Mississippi $3"},
     {std::regex("(\\bAR)([ -])(\\d{1,3})", std::regex_constants::icase), "Arkansas $3"},
     {std::regex("(\\bUT)([ -])(\\d{1,3})", std::regex_constants::icase), "Utah $3"},
     {std::regex("(\\bKS)([ -])(\\d{1,3})", std::regex_constants::icase), "Kansas $3"},
     {std::regex("(\\bNV)([ -])(\\d{1,3})", std::regex_constants::icase), "Nevada $3"},
     {std::regex("(\\bNM)([ -])(\\d{1,4})", std::regex_constants::icase), "New Mexico $3"},
     {std::regex("(\\bNE)([ -])(\\d{1,3})", std::regex_constants::icase), "Nebraska $3"},
     {std::regex("(\\bWV)([ -])(\\d{1,3})", std::regex_constants::icase), "West Virginia $3"},
     {std::regex("(\\bID)([ -])(\\d{1,3})", std::regex_constants::icase), "Idaho $3"},
     {std::regex("(\\bHI)([ -])(\\d{1,4})", std::regex_constants::icase), "Hawaii $3"},
     {std::regex("(\\bME)([ -])(\\d{1,3})", std::regex_constants::icase), "Maine $3"},
     {std::regex("(\\bNH)([ -])(\\d{1,3})", std::regex_constants::icase), "New Hampshire $3"},
     {std::regex("(\\bRI)([ -])(\\d{1,3})", std::regex_constants::icase), "Rhode Island $3"},
     {std::regex("(\\bMT)([ -])(\\d{1,3})", std::regex_constants::icase), "Montana $3"},
     {std::regex("(\\bDE)([ -])(\\d{1,3})", std::regex_constants::icase), "Delaware $3"},
     {std::regex("(\\bSD)([ -])(\\d{1,4})", std::regex_constants::icase), "South Dakota $3"},
     {std::regex("(\\bND)([ -])(\\d{1,4})", std::regex_constants::icase), "North Dakota $3"},
     {std::regex("(\\bAK)([ -])(\\d{1,3})", std::regex_constants::icase), "Alaska $3"},
     {std::regex("(\\bDC)([ -])(\\d{1,3})", std::regex_constants::icase), "D C $3"},
     {std::regex("(\\bVT)([ -])(\\d{1,3})", std::regex_constants::icase), "Vermont $3"},
     {std::regex("(\\bWY)([ -])(\\d{1,3})", std::regex_constants::icase), "Wyoming $3"}}};

const std::array<std::pair<std::regex, std::string>, 7> kCountyRoutes = {
    {{std::regex("(\\bCR)(\\d{1,4})([[:alpha:]]{1,2})?\\b", std::regex_constants::icase),
      "County Route $2$3"},
     {std::regex("(\\bCR)([ -])([[:alpha:]]{1,2})?(\\d{1,4})([[:alpha:]]{1,2})?\\b",
                 std::regex_constants::icase),
      "County Route $3$4$5"},
     {std::regex("(\\bCR)([ -])([[:alpha:]]{1,2})\\b", std::regex_constants::icase),
      "County Route $3"},
     {std::regex("(\\bC R)(\\d{1,4})([[:alpha:]]{1,2})?\\b", std::regex_constants::icase),
      "County Route $2$3"},
     {std::regex("(\\bC R)([ -])([[:alpha:]]{1,2})?(\\d{1,4})([[:alpha:]]{1,2})?\\b",
                 std::regex_constants::icase),
      "County Route $3$4$5"},
     {std::regex("(\\bC R)([ -])([[:alpha:]]{1,2})\\b", std::regex_constants::icase),
      "County Route $3"},
     {std::regex("(\\bCO)([ -])?(\\d{1,4})([[:alpha:]]{1,2})?\\b", std::regex_constants::icase),
      "County Road $3$4"}}};
} // namespace

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
