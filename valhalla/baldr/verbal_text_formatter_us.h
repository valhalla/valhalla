#ifndef VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_US_H_
#define VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_US_H_

#include <valhalla/baldr/verbal_text_formatter.h>

#include <regex>
#include <string>

namespace valhalla {
namespace baldr {

const std::regex kInterstateRegex("(\\bI)([ -])(H)?(\\d{1,3})",
                                  std::regex_constants::icase);
const std::string kInterstateOutPattern = "Interstate $3$4";

const std::regex kUsHighwayRegex("(\\bUS)([ -])(\\d{1,3})",
                                 std::regex_constants::icase);
const std::string kUsHighwayOutPattern = "U.S. $3";

const std::regex kThousandRegex("(^| )([1-9]{1,2})(000)($| )");
const std::string kThousandOutPattern = "$1$2 thousand$4";

const std::regex kHundredRegex("(^| )([1-9]{1,2})(00)($| )");
const std::string kHundredOutPattern = "$1$2 hundred$4";

const std::regex kUsNumberSplitRegex("(\\D*)(\\d+)(st|nd|rd|th)?(\\D*)");

const std::regex kLeadingOhRegex("( )(0)([1-9])");
const std::string kLeadingOhOutPattern = "$1o$3";

class VerbalTextFormatterUs : public VerbalTextFormatter {
 public:
  VerbalTextFormatterUs(const std::string& country_code,
                        const std::string& state_code);

  ~VerbalTextFormatterUs();

  std::string Format(const std::string& text) const override;

 protected:

  std::string ProcessNumberSplitMatch(const std::smatch& m) const override;

  std::string FormNumberSplitTts(const std::string& source) const override;

  std::string FormInterstateTts(const std::string& source) const;

  std::string FormUsHighwayTts(const std::string& source) const;

  std::string FormThousandTts(const std::string& source) const;

  std::string FormHundredTts(const std::string& source) const;

  std::string FormLeadingOhTts(const std::string& source) const;
};

}
}

#endif  // VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_US_H_
