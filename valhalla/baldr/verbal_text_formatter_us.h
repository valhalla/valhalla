#ifndef VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_US_H_
#define VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_US_H_

#include <valhalla/baldr/verbal_text_formatter.h>

#include <string>
#include <array>
#include <utility>

#include <valhalla/baldr/reutil.h>

namespace valhalla {
namespace baldr {

const re::regex kUsNumberSplitRegex("(\\D*)(\\d+)(st|nd|rd|th)?(\\D*)",
                                     re::regex_constants::icase);

const re::regex kInterstateRegex("(\\bI)([ -])(H)?(\\d{1,3})",
                                  re::regex_constants::icase);
const std::string kInterstateOutPattern = "Interstate $3$4";

const re::regex kUsHighwayRegex("(\\bUS)([ -])(Highway )?(\\d{1,3})",
                                 re::regex_constants::icase);
const std::string kUsHighwayOutPattern = "U.S. $3$4";

const re::regex kLeadingOhRegex("( )(0)([1-9])");
const std::string kLeadingOhOutPattern = "$1o$3";

const std::array<std::pair<re::regex, std::string>, 4> kThousandFindReplace = {{
    { re::regex("(^|\\D)([1-9]{1,2})(000$)"), "$1$2 thousand" },
    { re::regex("(^|\\D)([1-9]{1,2})(000th)", re::regex_constants::icase), "$1$2 thousandth" },
    { re::regex("(^|\\D)([1-9]{1,2})(000)( |-)"), "$1$2 thousand " },
    { re::regex("(^|\\D)([1-9]{1,2})(000)(\\D)"), "$1$2 thousand $4" }
}};

const std::array<std::pair<re::regex, std::string>, 4> kHundredFindReplace = {{
    { re::regex("(^|\\D)([1-9]{1,2})(00$)"), "$1$2 hundred" },
    { re::regex("(^|\\D)([1-9]{1,2})(00th)", re::regex_constants::icase), "$1$2 hundredth" },
    { re::regex("(^|\\D)([1-9]{1,2})(00)( |-)"), "$1$2 hundred " },
    { re::regex("(^|\\D)([1-9]{1,2})(00)(\\D)"), "$1$2 hundred $4" }
}};

const std::array<std::pair<re::regex, std::string>, 53> kStateRoutes = {{
    { re::regex("(\\bSR)([ -])?(\\d{1,4})", re::regex_constants::icase), "State Route $3" },
    { re::regex("(\\bSH)([ -])?(\\d{1,4})", re::regex_constants::icase), "State Highway $3" },
    { re::regex("(\\bCA)([ -])(\\d{1,3})", re::regex_constants::icase), "California $3" },
    { re::regex("(\\bTX)([ -])(\\d{1,3})", re::regex_constants::icase), "Texas $3" },
    { re::regex("(\\bFL)([ -])(A)?(\\d{1,3})", re::regex_constants::icase), "Florida $3$4" },
    { re::regex("(\\bNY)([ -])(\\d{1,3})", re::regex_constants::icase), "New York $3" },
    { re::regex("(\\bIL)([ -])(\\d{1,3})", re::regex_constants::icase), "Illinois $3" },
    { re::regex("(\\bPA)([ -])(\\d{1,3})", re::regex_constants::icase), "Pennsylvania $3" },
    { re::regex("(\\bOH)([ -])(\\d{1,3})", re::regex_constants::icase), "Ohio $3" },
    { re::regex("(\\bGA)([ -])(\\d{1,3})", re::regex_constants::icase), "Georgia $3" },
    { re::regex("(\\bNC)([ -])(\\d{1,3})", re::regex_constants::icase), "North Carolina $3" },
    { re::regex("(\\bM)([ -])(\\d{1,3})", re::regex_constants::icase), "Michigan $3" },
    { re::regex("(\\bNJ)([ -])(\\d{1,3})", re::regex_constants::icase), "New Jersey $3" },
    { re::regex("(\\bVA)([ -])(\\d{1,3})", re::regex_constants::icase), "Virginia $3" },
    { re::regex("(\\bWA)([ -])(\\d{1,3})", re::regex_constants::icase), "Washington $3" },
    { re::regex("(\\bMA)([ -])(\\d{1,3})", re::regex_constants::icase), "Massachusetts $3" },
    { re::regex("(\\bAZ)([ -])(\\d{1,3})", re::regex_constants::icase), "Arizona $3" },
    { re::regex("(\\bIN)([ -])(\\d{1,3})", re::regex_constants::icase), "Indiana $3" },
    { re::regex("(\\bTN)([ -])(\\d{1,3})", re::regex_constants::icase), "Tennessee $3" },
    { re::regex("(\\bMO)([ -])(\\d{1,3})", re::regex_constants::icase), "Missouri $3" },
    { re::regex("(\\bMO)([ -])([[:alpha:]]{1,2}\\b)", re::regex_constants::icase), "Missouri $3" },
    { re::regex("(\\bMD)([ -])(\\d{1,3})", re::regex_constants::icase), "Maryland $3" },
    { re::regex("(\\bWI)([ -])(\\d{1,3})", re::regex_constants::icase), "Wisconsin $3" },
    { re::regex("(\\bMN)([ -])(\\d{1,3})", re::regex_constants::icase), "Minnesota $3" },
    { re::regex("(\\bAL)([ -])(\\d{1,3})", re::regex_constants::icase), "Alabama $3" },
    { re::regex("(\\bSC)([ -])(\\d{1,3})", re::regex_constants::icase), "South Carolina $3" },
    { re::regex("(\\bLA)([ -])(\\d{1,4})", re::regex_constants::icase), "Louisiana $3" },
    { re::regex("(\\bKY)([ -])(\\d{1,4})", re::regex_constants::icase), "Kentucky $3" },
    { re::regex("(\\bOR)([ -])(\\d{1,3})", re::regex_constants::icase), "Oregon $3" },
    { re::regex("(\\bOK)([ -])(\\d{1,3})", re::regex_constants::icase), "Oklahoma $3" },
    { re::regex("(\\bCT)([ -])(\\d{1,3})", re::regex_constants::icase), "Connecticut $3" },
    { re::regex("(\\bIA)([ -])(\\d{1,3})", re::regex_constants::icase), "Iowa $3" },
    { re::regex("(\\bMS)([ -])(\\d{1,3})", re::regex_constants::icase), "Mississippi $3" },
    { re::regex("(\\bAR)([ -])(\\d{1,3})", re::regex_constants::icase), "Arkansas $3" },
    { re::regex("(\\bUT)([ -])(\\d{1,3})", re::regex_constants::icase), "Utah $3" },
    { re::regex("(\\bKS)([ -])(\\d{1,3})", re::regex_constants::icase), "Kansas $3" },
    { re::regex("(\\bNV)([ -])(\\d{1,3})", re::regex_constants::icase), "Nevada $3" },
    { re::regex("(\\bNM)([ -])(\\d{1,4})", re::regex_constants::icase), "New Mexico $3" },
    { re::regex("(\\bNE)([ -])(\\d{1,3})", re::regex_constants::icase), "Nebraska $3" },
    { re::regex("(\\bWV)([ -])(\\d{1,3})", re::regex_constants::icase), "West Virginia $3" },
    { re::regex("(\\bID)([ -])(\\d{1,3})", re::regex_constants::icase), "Idaho $3" },
    { re::regex("(\\bHI)([ -])(\\d{1,4})", re::regex_constants::icase), "Hawaii $3" },
    { re::regex("(\\bME)([ -])(\\d{1,3})", re::regex_constants::icase), "Maine $3" },
    { re::regex("(\\bNH)([ -])(\\d{1,3})", re::regex_constants::icase), "New Hampshire $3" },
    { re::regex("(\\bRI)([ -])(\\d{1,3})", re::regex_constants::icase), "Rhode Island $3" },
    { re::regex("(\\bMT)([ -])(\\d{1,3})", re::regex_constants::icase), "Montana $3" },
    { re::regex("(\\bDE)([ -])(\\d{1,3})", re::regex_constants::icase), "Delaware $3" },
    { re::regex("(\\bSD)([ -])(\\d{1,4})", re::regex_constants::icase), "South Dakota $3" },
    { re::regex("(\\bND)([ -])(\\d{1,4})", re::regex_constants::icase), "North Dakota $3" },
    { re::regex("(\\bAK)([ -])(\\d{1,3})", re::regex_constants::icase), "Alaska $3" },
    { re::regex("(\\bDC)([ -])(\\d{1,3})", re::regex_constants::icase), "D C $3" },
    { re::regex("(\\bVT)([ -])(\\d{1,3})", re::regex_constants::icase), "Vermont $3" },
    { re::regex("(\\bWY)([ -])(\\d{1,3})", re::regex_constants::icase), "Wyoming $3" }
}};

const std::array<std::pair<re::regex, std::string>, 7> kCountyRoutes = {{
    { re::regex("(\\bCR)(\\d{1,4})([[:alpha:]]{1,2})?\\b", re::regex_constants::icase), "County Route $2$3" },
    { re::regex("(\\bCR)([ -])([[:alpha:]]{1,2})?(\\d{1,4})([[:alpha:]]{1,2})?\\b", re::regex_constants::icase), "County Route $3$4$5" },
    { re::regex("(\\bCR)([ -])([[:alpha:]]{1,2})\\b", re::regex_constants::icase), "County Route $3" },
    { re::regex("(\\bC R)(\\d{1,4})([[:alpha:]]{1,2})?\\b", re::regex_constants::icase), "County Route $2$3" },
    { re::regex("(\\bC R)([ -])([[:alpha:]]{1,2})?(\\d{1,4})([[:alpha:]]{1,2})?\\b", re::regex_constants::icase), "County Route $3$4$5" },
    { re::regex("(\\bC R)([ -])([[:alpha:]]{1,2})\\b", re::regex_constants::icase), "County Route $3" },
    { re::regex("(\\bCO)([ -])?(\\d{1,4})([[:alpha:]]{1,2})?\\b", re::regex_constants::icase), "County Road $3$4" }
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

  std::string ProcessNumberSplitMatch(const re::smatch& m) const override;

  std::string FormNumberSplitTts(const std::string& source) const override;

  std::string FormInterstateTts(const std::string& source) const;

  std::string FormUsHighwayTts(const std::string& source) const;

  virtual std::string ProcessStatesTts(const std::string& source) const;

  bool FormStateTts(const std::string& source, const re::regex& state_regex,
                    const std::string& state_output_pattern,
                    std::string& tts) const;

  std::string ProcessCountysTts(const std::string& source) const;

  bool FormCountyTts(const std::string& source, const re::regex& county_regex,
                    const std::string& county_output_pattern,
                    std::string& tts) const;

  std::string ProcessThousandTts(const std::string& source) const;

  std::string FormThousandTts(const std::string& source,
                              const re::regex& thousand_regex,
                              const std::string& thousand_output_pattern) const;

  std::string ProcessHundredTts(const std::string& source) const;

  std::string FormHundredTts(const std::string& source,
                             const re::regex& hundred_regex,
                             const std::string& hundred_output_pattern) const;

  std::string FormLeadingOhTts(const std::string& source) const;
};

}
}

#endif  // VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_US_H_
