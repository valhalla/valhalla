#include <string>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/format.hpp>
#include <boost/optional.hpp>

#include "odin/markup_formatter.h"

namespace {
constexpr auto kQuotesTag = "<QUOTES>";

constexpr auto KSingleQuotes = "&apos;";
constexpr auto KDoubleQuotes = "&quot;";
} // namespace

namespace valhalla {
namespace odin {

// Constructor
MarkupFormatter::MarkupFormatter(const boost::property_tree::ptree& config)
    : markup_enabled_(config.get<bool>("odin.markup_formatter.markup_enabled", false)),
      phoneme_format_(config.get<std::string>("odin.markup_formatter.phoneme_format", "")) {
}

bool MarkupFormatter::markup_enabled() const {
  return markup_enabled_;
}

void MarkupFormatter::set_markup_enabled(bool markup_enabled) {
  markup_enabled_ = markup_enabled;
}

const std::string& MarkupFormatter::phoneme_format() const {
  return phoneme_format_;
}

boost::optional<std::string>
MarkupFormatter::Format(const std::unique_ptr<baldr::StreetName>& street_name) const {
  // Check if markup is enabled
  if (markup_enabled()) {

    // If pronunciation exists then process the phoneme format
    if (street_name->pronunciation()) {
      std::string markup_street_name = phoneme_format();

      // Use the proper quotes depending on the pronunciation alphabet
      FormatQuotes(street_name->pronunciation()->alphabet(), markup_street_name);

      // If the markup string exists then return the street name with the phoneme
      return boost::make_optional(!markup_street_name.empty(),
                                  (boost::format(markup_street_name) % street_name->value() %
                                   street_name->pronunciation()->value())
                                      .str());
    }
  }
  return boost::none;
}

boost::optional<std::string> MarkupFormatter::Format(const Sign& sign) const {
  // Check if markup is enabled
  if (markup_enabled()) {

    // If pronunciation exists then process the phoneme format
    if (sign.pronunciation()) {
      std::string markup_sign = phoneme_format();

      // Use the proper quotes depending on the pronunciation alphabet
      FormatQuotes(sign.pronunciation()->alphabet(), markup_sign);

      // If the markup string exists then return the sign with the phoneme
      return boost::make_optional(!markup_sign.empty(), (boost::format(markup_sign) % sign.text() %
                                                         sign.pronunciation()->value())
                                                            .str());
    }
  }
  return boost::none;
}

bool MarkupFormatter::UseSingleQuotes(valhalla::Pronunciation_Alphabet alphabet) const {
  if (alphabet == valhalla::Pronunciation_Alphabet_kNtSampa) {
    return true;
  }
  return false;
}

void MarkupFormatter::FormatQuotes(valhalla::Pronunciation_Alphabet alphabet,
                                   std::string& markup_string) const {
  // Use the proper quotes depending on the pronunciation alphabet
  UseSingleQuotes(alphabet) ? boost::replace_all(markup_string, kQuotesTag, KSingleQuotes)
                            : boost::replace_all(markup_string, kQuotesTag, KDoubleQuotes);
}

} // namespace odin
} // namespace valhalla
