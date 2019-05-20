#ifndef VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_H_
#define VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_H_

#include <regex>
#include <string>

namespace valhalla {
namespace baldr {

// Regular expression to find numbers
const std::regex kNumberSplitRegex("(\\D*)(\\d+)(\\D*)");

/**
 * The generic verbal text formatter class that prepares strings for use with
 * a text-to-speech engine.
 */
class VerbalTextFormatter {
public:
  VerbalTextFormatter(const std::string& country_code, const std::string& state_code);

  virtual ~VerbalTextFormatter();

  /**
   * Returns a text-to-speech formatted string based on the specified text.
   *
   * @param  text  the source string to transform.
   * @return a text-to-speech formatted string based on the specified text.
   */
  virtual std::string Format(const std::string& text) const;

protected:
  virtual std::string ProcessNumberSplitMatch(const std::smatch& m) const;

  virtual std::string FormNumberSplitTts(const std::string& source) const;

  // TODO - if not needed for special case logic then remove
  std::string country_code_;
  std::string state_code_;
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_H_
