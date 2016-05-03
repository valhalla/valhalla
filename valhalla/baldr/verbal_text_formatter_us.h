#ifndef VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_US_H_
#define VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_US_H_

#include <valhalla/baldr/verbal_text_formatter.h>

#include <boost/regex.hpp>
#include <string>
#include <array>
#include <utility>

namespace valhalla {
namespace baldr {

const boost::regex kUsNumberSplitRegex("(\\D*)(\\d+)(st|nd|rd|th)?(\\D*)",
                                     boost::regex_constants::icase);

const boost::regex kInterstateRegex("(\\bI)([ -])(H)?(\\d{1,3})",
                                  boost::regex_constants::icase);
const std::string kInterstateOutPattern = "Interstate $3$4";

const boost::regex kUsHighwayRegex("(\\bUS)([ -])(Highway )?(\\d{1,3})",
                                 boost::regex_constants::icase);
const std::string kUsHighwayOutPattern = "U.S. $3$4";

const boost::regex kLeadingOhRegex("( )(0)([1-9])");
const std::string kLeadingOhOutPattern = "$1o$3";

const std::array<std::pair<boost::regex, std::string>, 4> kThousandFindReplace = {{
    { boost::regex("(^|\\D)([1-9]{1,2})(000$)"), "$1$2 thousand" },
    { boost::regex("(^|\\D)([1-9]{1,2})(000th)", boost::regex_constants::icase), "$1$2 thousandth" },
    { boost::regex("(^|\\D)([1-9]{1,2})(000)( |-)"), "$1$2 thousand " },
    { boost::regex("(^|\\D)([1-9]{1,2})(000)(\\D)"), "$1$2 thousand $4" }
}};

const std::array<std::pair<boost::regex, std::string>, 4> kHundredFindReplace = {{
    { boost::regex("(^|\\D)([1-9]{1,2})(00$)"), "$1$2 hundred" },
    { boost::regex("(^|\\D)([1-9]{1,2})(00th)", boost::regex_constants::icase), "$1$2 hundredth" },
    { boost::regex("(^|\\D)([1-9]{1,2})(00)( |-)"), "$1$2 hundred " },
    { boost::regex("(^|\\D)([1-9]{1,2})(00)(\\D)"), "$1$2 hundred $4" }
}};

const std::array<std::pair<boost::regex, std::string>, 53> kStateRoutes = {{
    { boost::regex("(\\bSR)([ -])?(\\d{1,4})", boost::regex_constants::icase), "State Route $3" },
    { boost::regex("(\\bSH)([ -])?(\\d{1,4})", boost::regex_constants::icase), "State Highway $3" },
    { boost::regex("(\\bCA)([ -])(\\d{1,3})", boost::regex_constants::icase), "California $3" },
    { boost::regex("(\\bTX)([ -])(\\d{1,3})", boost::regex_constants::icase), "Texas $3" },
    { boost::regex("(\\bFL)([ -])(A)?(\\d{1,3})", boost::regex_constants::icase), "Florida $3$4" },
    { boost::regex("(\\bNY)([ -])(\\d{1,3})", boost::regex_constants::icase), "New York $3" },
    { boost::regex("(\\bIL)([ -])(\\d{1,3})", boost::regex_constants::icase), "Illinois $3" },
    { boost::regex("(\\bPA)([ -])(\\d{1,3})", boost::regex_constants::icase), "Pennsylvania $3" },
    { boost::regex("(\\bOH)([ -])(\\d{1,3})", boost::regex_constants::icase), "Ohio $3" },
    { boost::regex("(\\bGA)([ -])(\\d{1,3})", boost::regex_constants::icase), "Georgia $3" },
    { boost::regex("(\\bNC)([ -])(\\d{1,3})", boost::regex_constants::icase), "North Carolina $3" },
    { boost::regex("(\\bM)([ -])(\\d{1,3})", boost::regex_constants::icase), "Michigan $3" },
    { boost::regex("(\\bNJ)([ -])(\\d{1,3})", boost::regex_constants::icase), "New Jersey $3" },
    { boost::regex("(\\bVA)([ -])(\\d{1,3})", boost::regex_constants::icase), "Virginia $3" },
    { boost::regex("(\\bWA)([ -])(\\d{1,3})", boost::regex_constants::icase), "Washington $3" },
    { boost::regex("(\\bMA)([ -])(\\d{1,3})", boost::regex_constants::icase), "Massachusetts $3" },
    { boost::regex("(\\bAZ)([ -])(\\d{1,3})", boost::regex_constants::icase), "Arizona $3" },
    { boost::regex("(\\bIN)([ -])(\\d{1,3})", boost::regex_constants::icase), "Indiana $3" },
    { boost::regex("(\\bTN)([ -])(\\d{1,3})", boost::regex_constants::icase), "Tennessee $3" },
    { boost::regex("(\\bMO)([ -])(\\d{1,3})", boost::regex_constants::icase), "Missouri $3" },
    { boost::regex("(\\bMO)([ -])([[:alpha:]]{1,2}\\b)", boost::regex_constants::icase), "Missouri $3" },
    { boost::regex("(\\bMD)([ -])(\\d{1,3})", boost::regex_constants::icase), "Maryland $3" },
    { boost::regex("(\\bWI)([ -])(\\d{1,3})", boost::regex_constants::icase), "Wisconsin $3" },
    { boost::regex("(\\bMN)([ -])(\\d{1,3})", boost::regex_constants::icase), "Minnesota $3" },
    { boost::regex("(\\bAL)([ -])(\\d{1,3})", boost::regex_constants::icase), "Alabama $3" },
    { boost::regex("(\\bSC)([ -])(\\d{1,3})", boost::regex_constants::icase), "South Carolina $3" },
    { boost::regex("(\\bLA)([ -])(\\d{1,4})", boost::regex_constants::icase), "Louisiana $3" },
    { boost::regex("(\\bKY)([ -])(\\d{1,4})", boost::regex_constants::icase), "Kentucky $3" },
    { boost::regex("(\\bOR)([ -])(\\d{1,3})", boost::regex_constants::icase), "Oregon $3" },
    { boost::regex("(\\bOK)([ -])(\\d{1,3})", boost::regex_constants::icase), "Oklahoma $3" },
    { boost::regex("(\\bCT)([ -])(\\d{1,3})", boost::regex_constants::icase), "Connecticut $3" },
    { boost::regex("(\\bIA)([ -])(\\d{1,3})", boost::regex_constants::icase), "Iowa $3" },
    { boost::regex("(\\bMS)([ -])(\\d{1,3})", boost::regex_constants::icase), "Mississippi $3" },
    { boost::regex("(\\bAR)([ -])(\\d{1,3})", boost::regex_constants::icase), "Arkansas $3" },
    { boost::regex("(\\bUT)([ -])(\\d{1,3})", boost::regex_constants::icase), "Utah $3" },
    { boost::regex("(\\bKS)([ -])(\\d{1,3})", boost::regex_constants::icase), "Kansas $3" },
    { boost::regex("(\\bNV)([ -])(\\d{1,3})", boost::regex_constants::icase), "Nevada $3" },
    { boost::regex("(\\bNM)([ -])(\\d{1,4})", boost::regex_constants::icase), "New Mexico $3" },
    { boost::regex("(\\bNE)([ -])(\\d{1,3})", boost::regex_constants::icase), "Nebraska $3" },
    { boost::regex("(\\bWV)([ -])(\\d{1,3})", boost::regex_constants::icase), "West Virginia $3" },
    { boost::regex("(\\bID)([ -])(\\d{1,3})", boost::regex_constants::icase), "Idaho $3" },
    { boost::regex("(\\bHI)([ -])(\\d{1,4})", boost::regex_constants::icase), "Hawaii $3" },
    { boost::regex("(\\bME)([ -])(\\d{1,3})", boost::regex_constants::icase), "Maine $3" },
    { boost::regex("(\\bNH)([ -])(\\d{1,3})", boost::regex_constants::icase), "New Hampshire $3" },
    { boost::regex("(\\bRI)([ -])(\\d{1,3})", boost::regex_constants::icase), "Rhode Island $3" },
    { boost::regex("(\\bMT)([ -])(\\d{1,3})", boost::regex_constants::icase), "Montana $3" },
    { boost::regex("(\\bDE)([ -])(\\d{1,3})", boost::regex_constants::icase), "Delaware $3" },
    { boost::regex("(\\bSD)([ -])(\\d{1,4})", boost::regex_constants::icase), "South Dakota $3" },
    { boost::regex("(\\bND)([ -])(\\d{1,4})", boost::regex_constants::icase), "North Dakota $3" },
    { boost::regex("(\\bAK)([ -])(\\d{1,3})", boost::regex_constants::icase), "Alaska $3" },
    { boost::regex("(\\bDC)([ -])(\\d{1,3})", boost::regex_constants::icase), "D C $3" },
    { boost::regex("(\\bVT)([ -])(\\d{1,3})", boost::regex_constants::icase), "Vermont $3" },
    { boost::regex("(\\bWY)([ -])(\\d{1,3})", boost::regex_constants::icase), "Wyoming $3" }
}};

const std::array<std::pair<boost::regex, std::string>, 7> kCountyRoutes = {{
    { boost::regex("(\\bCR)(\\d{1,4})([[:alpha:]]{1,2})?\\b", boost::regex_constants::icase), "County Route $2$3" },
    { boost::regex("(\\bCR)([ -])([[:alpha:]]{1,2})?(\\d{1,4})([[:alpha:]]{1,2})?\\b", boost::regex_constants::icase), "County Route $3$4$5" },
    { boost::regex("(\\bCR)([ -])([[:alpha:]]{1,2})\\b", boost::regex_constants::icase), "County Route $3" },
    { boost::regex("(\\bC R)(\\d{1,4})([[:alpha:]]{1,2})?\\b", boost::regex_constants::icase), "County Route $2$3" },
    { boost::regex("(\\bC R)([ -])([[:alpha:]]{1,2})?(\\d{1,4})([[:alpha:]]{1,2})?\\b", boost::regex_constants::icase), "County Route $3$4$5" },
    { boost::regex("(\\bC R)([ -])([[:alpha:]]{1,2})\\b", boost::regex_constants::icase), "County Route $3" },
    { boost::regex("(\\bCO)([ -])?(\\d{1,4})([[:alpha:]]{1,2})?\\b", boost::regex_constants::icase), "County Road $3$4" }
}};

/**
 * The US specific verbal text formatter class that prepares strings for use
 * with a text-to-speech engine.
 */
class VerbalTextFormatterUs : public VerbalTextFormatter {
 public:
  VerbalTextFormatterUs(const std::string& country_code,
                        const std::string& state_code);

  ~VerbalTextFormatterUs();

  /**
   * Returns a US text-to-speech formatted string based on the specified text.
   *
   * @param  text  the source string to transform.
   * @return a US text-to-speech formatted string based on the specified text.
   */
  std::string Format(const std::string& text) const override;

 protected:

  std::string ProcessNumberSplitMatch(const boost::smatch& m) const override;

  std::string FormNumberSplitTts(const std::string& source) const override;

  std::string FormInterstateTts(const std::string& source) const;

  std::string FormUsHighwayTts(const std::string& source) const;

  virtual std::string ProcessStatesTts(const std::string& source) const;

  bool FormStateTts(const std::string& source, const boost::regex& state_regex,
                    const std::string& state_output_pattern,
                    std::string& tts) const;

  std::string ProcessCountysTts(const std::string& source) const;

  bool FormCountyTts(const std::string& source, const boost::regex& county_regex,
                    const std::string& county_output_pattern,
                    std::string& tts) const;

  std::string ProcessThousandTts(const std::string& source) const;

  std::string FormThousandTts(const std::string& source,
                              const boost::regex& thousand_regex,
                              const std::string& thousand_output_pattern) const;

  std::string ProcessHundredTts(const std::string& source) const;

  std::string FormHundredTts(const std::string& source,
                             const boost::regex& hundred_regex,
                             const std::string& hundred_output_pattern) const;

  std::string FormLeadingOhTts(const std::string& source) const;
};

}
}

#endif  // VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_US_H_
