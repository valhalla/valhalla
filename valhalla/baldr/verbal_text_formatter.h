#ifndef VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_H_
#define VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_H_

#include <regex>
#include <string>

namespace valhalla {
namespace baldr {

const std::string kNumberSplitPattern = "(\\D*)(\\d+)(\\D*)";
const std::regex kNumberSplitRegex(kNumberSplitPattern);

class VerbalTextFormatter {
 public:
  VerbalTextFormatter(const std::string& country_code,
                      const std::string& state_code);

  virtual ~VerbalTextFormatter();

  virtual std::string Format(const std::string& text) const;

 protected:
  virtual std::string ProcessNumberSplitMatch(const std::smatch& m) const;

  virtual std::string FormNumberSplitTts(const std::string& source) const;

  std::string country_code_;
  std::string state_code_;

};

}
}

#endif  // VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_H_
