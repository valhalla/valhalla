#ifndef VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_H_
#define VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_H_

#include <regex>
#include <string>

namespace valhalla {
namespace baldr {

const std::string kThousandPattern = "(^| )([1-9]{1,2})(000)($| )";
const std::regex kThousandRegex(kThousandPattern);
const std::string kThousandOutPattern = "$1$2 thousand$4";

const std::string kHundredPattern = "(^| )([1-9]{1,2})(00)($| )";
const std::regex kHundredRegex(kHundredPattern);
const std::string kHundredOutPattern = "$1$2 hundred$4";

const std::string kNumberSplitPattern = "(\\D*)(\\d+)(st|nd|rd|th)?(\\D*)";
const std::regex kNumberSplitRegex(kNumberSplitPattern);

const std::string kLeadingOhPattern = "( )(0)([1-9])";
const std::regex kLeadingOhRegex(kLeadingOhPattern);
const std::string kLeadingOhOutPattern = "$1o$3";

class VerbalTextFormatter {
 public:
  VerbalTextFormatter(const std::string& country_code,
                      const std::string& state_code);

  virtual ~VerbalTextFormatter();

  virtual std::string Format(const std::string& text) const;

 protected:
  std::string FormThousandTts(const std::string& source) const;

  std::string FormHundredTts(const std::string& source) const;

  std::string ProcessNumberSplitMatch(const std::smatch& m) const;

  std::string FormNumberSplitTts(const std::string& source) const;

  std::string FormLeadingOhTts(const std::string& source) const;

  std::string country_code_;
  std::string state_code_;

};

}
}

#endif  // VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_H_
