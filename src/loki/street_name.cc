#include "loki/street_name.h"
#include "street_name_extra.h"

#include <utf8proc.h>

#include <algorithm>
#include <iterator>

namespace valhalla {
namespace loki {
namespace street_name {

namespace {

// case folds and decomposes, compatibility forms included
// pure diacritics are dropped in normalize
constexpr auto kDecomposeOpts =
    static_cast<utf8proc_option_t>(UTF8PROC_COMPAT | UTF8PROC_DECOMPOSE | UTF8PROC_CASEFOLD);

constexpr std::pair<std::u32string_view, char32_t> kDigraphs[] = {
    {U"ue", U'u'},
    {U"oe", U'o'},
    {U"ae", U'a'},
};

// only the diacritics of Latin, Greek and Cyrillic
// the marks of other scripts carry their vowels
bool is_diacritic(utf8proc_int32_t codepoint) {
  return codepoint >= 0x0300 && codepoint <= 0x036F;
}

// anything but letters, marks and numbers
bool is_separator(utf8proc_int32_t codepoint) {
  const auto category = utf8proc_category(codepoint);
  return category < UTF8PROC_CATEGORY_LU || category > UTF8PROC_CATEGORY_NO;
}

} // namespace

normalized_t normalize(std::string_view name) {
  normalized_t normalized;
  std::u32string token;
  auto close_token = [&normalized, &token]() {
    if (token.empty())
      return;
    normalized.stripped += token;
    normalized.tokens.push_back(std::move(token));
    token.clear();
  };

  const auto* name_bytes = reinterpret_cast<const utf8proc_uint8_t*>(name.data());
  const auto name_size = static_cast<utf8proc_ssize_t>(name.size());

  // turn the utf-8 into utf-32 to hold raw unicode codepoints
  utf8proc_int32_t decomposed[32];
  for (utf8proc_ssize_t i = 0; i < name_size;) {
    utf8proc_int32_t codepoint;
    // see if we can read a unicode codepoint
    const auto name_bytes_read = utf8proc_iterate(name_bytes + i, name_size - i, &codepoint);
    // a malformed byte separates tokens
    // we require utf-8 as input, but who knows what clients send..
    if (name_bytes_read < 0) {
      close_token();
      ++i;
      continue;
    }

    // decompose the utf-8 codepoint(s) to unicode codepoint(s)
    const auto decomposed_count =
        utf8proc_decompose_char(codepoint, decomposed, std::size(decomposed), kDecomposeOpts,
                                nullptr);
    if (decomposed_count < 0 ||
        decomposed_count > static_cast<utf8proc_ssize_t>(std::size(decomposed))) {
      close_token();
      i += name_bytes_read;
      continue;
    }

    for (utf8proc_ssize_t j = 0; j < decomposed_count; ++j) {
      if (is_separator(decomposed[j]))
        close_token();
      else if (!is_diacritic(decomposed[j])) {
        // binary search to find a matching codepoint in the extra table which utf8proc doesn't
        // decompose
        const auto* entry = std::lower_bound(std::begin(kDecomposeExtra), std::end(kDecomposeExtra),
                                             static_cast<char32_t>(decomposed[j]),
                                             [](const auto& e, char32_t c) { return e.first < c; });

        if (entry == std::end(kDecomposeExtra) ||
            entry->first != static_cast<char32_t>(decomposed[j]))
          token.push_back(static_cast<char32_t>(decomposed[j]));
        else
          token.append(entry->second.begin(), entry->second.end());
      }
    }

    i += name_bytes_read;
  }
  close_token();
  return normalized;
}

normalized_t collapse_digraphs(const normalized_t& light) {
  normalized_t collapsed;
  collapsed.tokens = light.tokens;
  for (auto& token : collapsed.tokens) {
    for (const auto& [digraph, single] : kDigraphs) {
      for (auto pos = token.find(digraph); pos != token.npos; pos = token.find(digraph, pos + 1))
        token.replace(pos, digraph.size(), 1, single);
    }
    collapsed.stripped += token;
  }
  return collapsed;
}

// TODO: idf weighted trigram containment, ref regime, retries
std::vector<float> score(std::string_view hint, const std::vector<names_t>& candidates) {
  const auto normalized_hint = normalize(hint).stripped;
  std::vector<float> scores;
  scores.reserve(candidates.size());
  for (const auto& names : candidates) {
    float best = 0.f;
    for (const auto& [name, is_ref] : names) {
      if (normalize(name).stripped == normalized_hint) {
        best = 1.f;
        break;
      }
    }
    scores.push_back(best);
  }
  return scores;
}

std::vector<size_t>
best_matches(std::string_view hint, const std::vector<names_t>& candidates, float threshold) {
  const auto scores = score(hint, candidates);
  std::vector<size_t> matches;
  float best = threshold;
  for (size_t i = 0; i < scores.size(); ++i) {
    if (scores[i] < best)
      continue;
    if (scores[i] > best) {
      best = scores[i];
      matches.clear();
    }
    matches.push_back(i);
  }
  return matches;
}

} // namespace street_name
} // namespace loki
} // namespace valhalla
