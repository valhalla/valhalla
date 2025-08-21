#ifndef VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_US_H_
#define VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_US_H_

#include <valhalla/baldr/verbal_text_formatter.h>

#include <regex>
#include <string>

namespace valhalla {
namespace baldr {

/**
 * The US specific verbal text formatter class that prepares strings for use
 * with a text-to-speech engine.
 */
class VerbalTextFormatterUs : public VerbalTextFormatter {
public:
  VerbalTextFormatterUs(const std::string& country_code, const std::string& state_code);

  ~VerbalTextFormatterUs();

  /**
   * Returns a US text-to-speech formatted string based on the specified text.
   *
   * @param  text  the source string to transform.
   *
   * @return a US text-to-speech formatted string based on the specified text.
   */
  std::string Format(const std::string& text) const override;

protected:
  std::string ProcessNumberSplitMatch(const std::smatch& m) const override;

  std::string FormNumberSplitTts(const std::string& source) const override;

  std::string FormInterstateTts(const std::string& source) const;

  std::string FormUsHighwayTts(const std::string& source) const;

  virtual std::string ProcessStatesTts(const std::string& source) const;

  bool FormStateTts(const std::string& source,
                    const std::regex& state_regex,
                    const std::string& state_output_pattern,
                    std::string& tts) const;

  std::string ProcessCountysTts(const std::string& source) const;

  bool FormCountyTts(const std::string& source,
                     const std::regex& county_regex,
                     const std::string& county_output_pattern,
                     std::string& tts) const;

  std::string ProcessThousandTts(const std::string& source) const;

  std::string FormThousandTts(const std::string& source,
                              const std::regex& thousand_regex,
                              const std::string& thousand_output_pattern) const;

  std::string ProcessHundredTts(const std::string& source) const;

  std::string FormHundredTts(const std::string& source,
                             const std::regex& hundred_regex,
                             const std::string& hundred_output_pattern) const;

  std::string FormLeadingOhTts(const std::string& source) const;
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_US_H_
