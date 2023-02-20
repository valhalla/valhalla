#include <optional>
#include <string>
#include <unordered_map>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/format.hpp>

#include "odin/markup_formatter.h"
#include "proto/common.pb.h"

namespace {
constexpr auto kQuotesTag = "<QUOTES>";
constexpr auto kPhoneticAlphabetTag = "<PHONETIC_ALPHABET>";
constexpr auto kTextualStringTag = "<TEXTUAL_STRING>";
constexpr auto kVerbalStringTag = "<VERBAL_STRING>";

constexpr auto KSingleQuotes = "'";
constexpr auto KDoubleQuotes = "\"";

const std::string& PronunciationAlphabetToString(valhalla::Pronunciation_Alphabet alphabet) {
  static const std::unordered_map<valhalla::Pronunciation_Alphabet, std::string>
      values{{valhalla::Pronunciation_Alphabet::Pronunciation_Alphabet_kIpa, "ipa"},
             {valhalla::Pronunciation_Alphabet::Pronunciation_Alphabet_kXKatakana, "x-katakana"},
             {valhalla::Pronunciation_Alphabet::Pronunciation_Alphabet_kXJeita, "x-jeita"},
             {valhalla::Pronunciation_Alphabet::Pronunciation_Alphabet_kNtSampa, "nt-sampa"}};
  auto f = values.find(alphabet);
  if (f == values.cend())
    throw std::runtime_error("Missing value in protobuf Pronunciation_Alphabet enum to string");
  return f->second;
}
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

std::optional<std::string>
MarkupFormatter::FormatPhonemeElement(const std::unique_ptr<baldr::StreetName>& street_name) const {
  // Check if markup is enabled
  if (markup_enabled()) {

    // If pronunciation exists then process the phoneme format
    if (street_name->pronunciation()) {
      // Populate the phoneme markup string
      std::string phoneme_markup_string =
          FormatPhonemeElement(street_name->value(), street_name->pronunciation());

      // If the markup string exists then return the street name with the phoneme
      return phoneme_markup_string.empty() ? std::nullopt : std::make_optional(phoneme_markup_string);
    }
  }
  return std::nullopt;
}

std::optional<std::string> MarkupFormatter::FormatPhonemeElement(const Sign& sign) const {
  // Check if markup is enabled
  if (markup_enabled()) {

    // If pronunciation exists then process the phoneme format
    if (sign.pronunciation()) {
      // Populate the phoneme markup string
      std::string phoneme_markup_string = FormatPhonemeElement(sign.text(), sign.pronunciation());

      // If the markup string exists then return the sign with the phoneme
      if (!phoneme_markup_string.empty()) {
        return std::make_optional(phoneme_markup_string);
      }
    }
  }

  return std::nullopt;
}

const std::string& MarkupFormatter::phoneme_format() const {
  return phoneme_format_;
}

bool MarkupFormatter::UseSingleQuotes(valhalla::Pronunciation_Alphabet alphabet) const {
  if (alphabet == valhalla::Pronunciation_Alphabet_kNtSampa) {
    return true;
  }
  return false;
}

void MarkupFormatter::FormatQuotes(std::string& markup_string,
                                   valhalla::Pronunciation_Alphabet alphabet) const {
  // Use the proper quotes depending on the pronunciation alphabet
  UseSingleQuotes(alphabet) ? boost::replace_all(markup_string, kQuotesTag, KSingleQuotes)
                            : boost::replace_all(markup_string, kQuotesTag, KDoubleQuotes);
}

std::string MarkupFormatter::FormatPhonemeElement(
    const std::string& textual_string,
    const std::optional<baldr::Pronunciation>& pronunciation) const {
  std::string phoneme_markup_string = phoneme_format();

  // Use the proper quotes depending on the pronunciation alphabet
  FormatQuotes(phoneme_markup_string, pronunciation->alphabet);

  // Replace phrase tags with values
  boost::replace_all(phoneme_markup_string, kPhoneticAlphabetTag,
                     PronunciationAlphabetToString(pronunciation->alphabet));
  boost::replace_all(phoneme_markup_string, kTextualStringTag, textual_string);
  boost::replace_all(phoneme_markup_string, kVerbalStringTag, pronunciation->value);

  return phoneme_markup_string;
}

} // namespace odin
} // namespace valhalla
