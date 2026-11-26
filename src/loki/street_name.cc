#include "loki/street_name.h"

#include <ankerl/unordered_dense.h>
#include <utf8proc.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <string_view>
#include <utility>

namespace valhalla {
namespace loki {
namespace street_name {

namespace {

// case folds and decomposes, compatibility forms included
// pure diacritics are dropped in normalize
constexpr auto kDecomposeOpts =
    static_cast<utf8proc_option_t>(UTF8PROC_COMPAT | UTF8PROC_DECOMPOSE | UTF8PROC_CASEFOLD);

// only the diacritics of Latin, Greek and Cyrillic
// the marks of other scripts carry their vowels
bool is_diacritic(utf8proc_int32_t codepoint) {
  return codepoint >= 0x0300 && codepoint <= 0x036F;
}

// letters, marks and numbers
bool is_alphanumeric(utf8proc_int32_t codepoint) {
  const auto category = utf8proc_category(codepoint);
  return category >= UTF8PROC_CATEGORY_LU && category <= UTF8PROC_CATEGORY_NO;
}

// lowercase ASCII form of the Latin letters that don't decompose in utf8proc, sorted by codepoint
// only the lowercase forms are listed, it assumes case folding has been done before searching here
// clang-format off
constexpr std::pair<char32_t, std::string_view> kDecomposeExtra[] = {
    {0x00E6, "ae"}, // æ
    {0x00F0, "d"}, // ð
    {0x00F8, "o"}, // ø
    {0x00FE, "th"}, // þ
    {0x0111, "d"}, // đ
    {0x0127, "h"}, // ħ
    {0x0131, "i"}, // ı
    {0x0138, "k"}, // ĸ
    {0x0142, "l"}, // ł
    {0x014B, "ng"}, // ŋ
    {0x0153, "oe"}, // œ
    {0x0167, "t"}, // ŧ
    {0x0180, "b"}, // ƀ
    {0x0183, "b"}, // ƃ
    {0x0188, "c"}, // ƈ
    {0x018C, "d"}, // ƌ
    {0x018D, "d"}, // ƍ
    {0x0192, "f"}, // ƒ
    {0x0195, "hv"}, // ƕ
    {0x0199, "k"}, // ƙ
    {0x019A, "l"}, // ƚ
    {0x019B, "l"}, // ƛ
    {0x019E, "n"}, // ƞ
    {0x01A3, "oi"}, // ƣ
    {0x01A5, "p"}, // ƥ
    {0x01AA, "s"}, // ƪ
    {0x01AB, "t"}, // ƫ
    {0x01AD, "t"}, // ƭ
    {0x01B4, "y"}, // ƴ
    {0x01B6, "z"}, // ƶ
    {0x01B9, "z"}, // ƹ
    {0x01BA, "z"}, // ƺ
    {0x01BF, "w"}, // ƿ
    {0x01DD, "e"}, // ǝ
    {0x01E5, "g"}, // ǥ
    {0x021D, "gh"}, // ȝ
    {0x0221, "d"}, // ȡ
    {0x0223, "ou"}, // ȣ
    {0x0225, "z"}, // ȥ
    {0x0234, "l"}, // ȴ
    {0x0235, "n"}, // ȵ
    {0x0236, "t"}, // ȶ
    {0x0237, "j"}, // ȷ
    {0x0238, "db"}, // ȸ
    {0x0239, "qp"}, // ȹ
    {0x023C, "c"}, // ȼ
    {0x023F, "s"}, // ȿ
    {0x0240, "z"}, // ɀ
    {0x0247, "e"}, // ɇ
    {0x0249, "j"}, // ɉ
    {0x024B, "q"}, // ɋ
    {0x024D, "r"}, // ɍ
    {0x024F, "y"}, // ɏ
    {0x0253, "b"}, // ɓ
    {0x0254, "o"}, // ɔ
    {0x0256, "d"}, // ɖ
    {0x0257, "d"}, // ɗ
    {0x0259, "e"}, // ə
    {0x025B, "e"}, // ɛ
    {0x0260, "g"}, // ɠ
    {0x0263, "g"}, // ɣ
    {0x0268, "i"}, // ɨ
    {0x0269, "i"}, // ɩ
    {0x026F, "m"}, // ɯ
    {0x0272, "n"}, // ɲ
    {0x0275, "o"}, // ɵ
    {0x0280, "r"}, // ʀ
    {0x0283, "s"}, // ʃ
    {0x0288, "t"}, // ʈ
    {0x0289, "u"}, // ʉ
    {0x028A, "u"}, // ʊ
    {0x028B, "v"}, // ʋ
    {0x028C, "v"}, // ʌ
    {0x0292, "z"}, // ʒ
    {0x2C65, "a"}, // ⱥ
    {0x2C66, "t"}, // ⱦ
};
// clang-format on
static_assert(std::is_sorted(std::begin(kDecomposeExtra), std::end(kDecomposeExtra)));

// three codepoints of 21 bits each (top 11 bits of a valid codepoint are empty)
using gram_t = uint64_t;
constexpr size_t kGramSize = 3;

// sorted gram bag after normalization, each uint64_t representing one 3-char gram
// normalized strings less than kGramSize long have none (e.g. refs)
std::vector<gram_t> get_trigrams(const std::u32string& text) {
  std::vector<gram_t> grams;
  for (size_t i = 0; i + kGramSize <= text.size(); ++i)
    grams.push_back(gram_t{text[i]} << 42 | gram_t{text[i + 1]} << 21 | gram_t{text[i + 2]});
  std::sort(grams.begin(), grams.end());
  return grams;
}

// inverse document frequency in log space to smoothen impact of very rare grams
// ln((N + 1) / max(df, 1))
class gram_weights_t {
public:
  explicit gram_weights_t(const std::vector<std::vector<gram_t>>& pool_grams) {
    size_t count = 0;
    for (const auto& grams : pool_grams) {
      // ignore names too short for a gram
      if (grams.empty())
        continue;
      ++count;
      for (size_t i = 0; i < grams.size(); ++i) {
        // dedupe grams within a single document
        if (i == 0 || grams[i] != grams[i - 1])
          ++pool_gram_frequency_[grams[i]];
      }
    }
    pool_count_ln_ = std::log(static_cast<float>(count + 1));
  }

  // returns the log-smoothed IDF for grams which are found among the candidates
  // hint grams not found get the max weight, which dilutes the total hint's weight
  float operator()(gram_t hint_gram) const {
    const auto it = pool_gram_frequency_.find(hint_gram);
    if (it == pool_gram_frequency_.end())
      return pool_count_ln_;

    // ln((N + 1) / max(df, 1))
    return pool_count_ln_ - std::log(static_cast<float>(it->second));
  }

private:
  float pool_count_ln_;
  ankerl::unordered_dense::map<gram_t, uint32_t> pool_gram_frequency_;
};

// the share of the hint's weight the name covers. we only add the hint grams' weight if it's in
// the candidates grams, so abbreviations like str work without penalty
// TODO(nils): however not yet e.g. calle -> cl
float score_candidate(const std::vector<gram_t>& hint_grams,
                      const std::vector<float>& hint_weights,
                      const std::vector<gram_t>& name_grams) {
  float total = 0.f;
  float shared = 0.f;
  auto it = name_grams.begin();
  for (size_t i = 0; i < hint_grams.size(); ++i) {
    total += hint_weights[i];
    // both are sorted, we're only penalizing for grams which are in the hint
    // but not in the street name, not vice versa
    it = std::lower_bound(it, name_grams.end(), hint_grams[i]);
    if (it != name_grams.end() && *it == hint_grams[i]) {
      shared += hint_weights[i];
      ++it;
    }
  }

  // total can be 0 if no valid grams in the whole pool
  return total > 0.f ? shared / total : 0.f;
}

} // namespace

std::u32string normalize_name(std::string_view name) {
  std::u32string normalized;
  const auto* name_bytes = reinterpret_cast<const utf8proc_uint8_t*>(name.data());
  const auto name_size = static_cast<utf8proc_ssize_t>(name.size());

  // turn the utf-8 into utf-32 to hold raw unicode codepoints
  utf8proc_int32_t decomposed[32];
  for (utf8proc_ssize_t i = 0; i < name_size;) {
    utf8proc_int32_t codepoint;
    // see if we can read a unicode codepoint
    const auto name_bytes_read = utf8proc_iterate(name_bytes + i, name_size - i, &codepoint);
    // a malformed byte is dropped
    // we require utf-8 as input, but who knows what clients send..
    if (name_bytes_read < 0) {
      ++i;
      continue;
    }

    // decompose the utf-8 codepoint(s) to unicode codepoint(s)
    const auto decomposed_count =
        utf8proc_decompose_char(codepoint, decomposed, std::size(decomposed), kDecomposeOpts,
                                nullptr);
    if (decomposed_count < 0 ||
        decomposed_count > static_cast<utf8proc_ssize_t>(std::size(decomposed))) {
      i += name_bytes_read;
      continue;
    }

    // drop anything non-alphanumeric and the relevant diacritics
    for (utf8proc_ssize_t j = 0; j < decomposed_count; ++j) {
      if (!is_alphanumeric(decomposed[j]) || is_diacritic(decomposed[j]))
        continue;

      // binary search to find a matching codepoint in the extra table which utf8proc doesn't
      // decompose
      const auto* found = std::lower_bound(std::begin(kDecomposeExtra), std::end(kDecomposeExtra),
                                           static_cast<char32_t>(decomposed[j]),
                                           [](const auto& e, char32_t c) { return e.first < c; });

      if (found == std::end(kDecomposeExtra) || found->first != static_cast<char32_t>(decomposed[j]))
        normalized.push_back(static_cast<char32_t>(decomposed[j]));
      else
        normalized.append(found->second.begin(), found->second.end());
    }

    i += name_bytes_read;
  }
  return normalized;
}

std::vector<float> score_candidates(std::string_view hint, const std::vector<names_t>& pool) {
  std::vector<float> scores(pool.size(), 0.f);
  const auto hint_grams = get_trigrams(normalize_name(hint));
  if (hint_grams.empty())
    return scores;

  // dedupe the names of all candidate edges before building the weights
  ankerl::unordered_dense::map<std::string_view, uint32_t> document_by_name;
  std::vector<std::vector<gram_t>> pool_grams_unique;
  std::vector<std::pair<size_t, uint32_t>> cands_to_grams;
  for (size_t i = 0; i < pool.size(); ++i) {
    for (const auto& [name, is_ref] : pool[i]) {
      // TODO(nils): refs like A1 need extra logic
      if (is_ref)
        continue;
      auto [it, added] = document_by_name.try_emplace(name, pool_grams_unique.size());
      if (added)
        pool_grams_unique.push_back(get_trigrams(normalize_name(name)));
      cands_to_grams.emplace_back(i, it->second);
    }
  }

  // define the hint's weights wrt the whole names pool
  const gram_weights_t gram_weights(pool_grams_unique);
  std::vector<float> hint_weights;
  hint_weights.reserve(hint_grams.size());
  for (const auto gram : hint_grams)
    hint_weights.push_back(gram_weights(gram));

  // for each candidate name in the pool, determine its hint score
  std::vector<float> pool_grams_scores;
  pool_grams_scores.reserve(pool_grams_unique.size());
  for (const auto& grams : pool_grams_unique)
    pool_grams_scores.push_back(score_candidate(hint_grams, hint_weights, grams));

  // a candidate scores its best name
  for (const auto [cand_idx, grams_idx] : cands_to_grams)
    scores[cand_idx] = std::max(scores[cand_idx], pool_grams_scores[grams_idx]);

  return scores;
}

} // namespace street_name
} // namespace loki
} // namespace valhalla
