#ifndef VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_US_H_
#define VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_US_H_

#include <valhalla/baldr/verbal_text_formatter.h>

#include <regex>
#include <string>

namespace valhalla {
namespace baldr {

const std::string kInterstatePattern = "(\\bI)([ -])(H)?(\\d{1,3})";
const std::regex kInterstateRegex(kInterstatePattern,
                                  std::regex_constants::icase);
const std::string kInterstateOutPattern = "Interstate $3$4";

const std::string kUsHighwayPattern = "(\\bUS)([ -])(\\d{1,3})";
const std::regex kUsHighwayRegex(kUsHighwayPattern,
                                 std::regex_constants::icase);
const std::string kUsHighwayOutPattern = "U.S. $3";

class VerbalTextFormatterUs : public VerbalTextFormatter {
 public:
  VerbalTextFormatterUs(const std::string& country_code,
                        const std::string& state_code);

  ~VerbalTextFormatterUs();

  std::string Format(const std::string& text) const;

 protected:

  std::string FormInterstateTts(const std::string& source) const;

  std::string FormUsHighwayTts(const std::string& source) const;

};

}
}

#endif  // VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_US_H_
