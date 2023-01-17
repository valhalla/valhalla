#pragma once

#include <optional>
#include <string>

#include <boost/property_tree/ptree.hpp>

#include <valhalla/baldr/streetname.h>
#include <valhalla/odin/sign.h>

namespace valhalla {
namespace odin {

class MarkupFormatter {
public:
  /**
   * Constructor.
   * @param  config  the valhalla odin config values.
   */
  explicit MarkupFormatter(const boost::property_tree::ptree& config = {});

  /**
   * Returns true if markup is enabled.
   * @return true if markup is enabled.
   */
  bool markup_enabled() const;

  /**
   * Sets the markup enabled flag.
   * @param  markup_enabled  bool flag to enable/disable markup.
   */
  void set_markup_enabled(bool markup_enabled);

  /**
   * Return the street name with phoneme markup if it exists.
   *
   * @param  street_name  the street name record to format.
   * @return the street name with phoneme markup if it exists.
   */
  std::optional<std::string>
  FormatPhonemeElement(const std::unique_ptr<baldr::StreetName>& street_name) const;

  /**
   * Return the sign with phoneme markup if it exists.
   *
   * @param  sign  the sign record to format.
   * @return the sign with phoneme markup if it exists.
   */
  std::optional<std::string> FormatPhonemeElement(const Sign& sign) const;

protected:
  /**
   * Returns the phoneme format string.
   * @return  the phoneme format string.
   */
  const std::string& phoneme_format() const;

  bool UseSingleQuotes(valhalla::Pronunciation_Alphabet alphabet) const;

  void FormatQuotes(std::string& markup_string, valhalla::Pronunciation_Alphabet alphabet) const;

  std::string FormatPhonemeElement(const std::string& textual_string,
                                   const std::optional<baldr::Pronunciation>& pronunciation) const;

  bool markup_enabled_;
  std::string phoneme_format_;
};

} // namespace odin
} // namespace valhalla
