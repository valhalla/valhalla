#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace valhalla {
namespace loki {
namespace street_name {

// (name, is route number) as returned by EdgeInfo::GetNames(bool)
using names_t = std::vector<std::pair<std::string, bool>>;

// a name reduced to comparable form. tokens keeps the word boundaries for the token-level rules,
// stripped drops them, which is what makes `Hermannstrasse` and `Hermann Strasse` the same string
struct normalized_t {
  std::vector<std::u32string> tokens;
  std::u32string stripped;
};

// lowercases, transliterates and splits on anything that isn't alphanumeric, used on both the hint
// and the candidate names
normalized_t normalize(std::string_view name);

// the retry pass: collapses the digraphs people swap for the plain vowel, which loses information
normalized_t collapse_digraphs(const normalized_t& light);

// one score in [0, 1] per candidate, the max over its names. All candidates' names form the rarity
// corpus, so a pool has to be scored as a whole.
std::vector<float> score(std::string_view hint, const std::vector<names_t>& candidates);

// indices of the candidates sharing the best score in candidates' order, none if that score is
// below threshold
std::vector<size_t>
best_matches(std::string_view hint, const std::vector<names_t>& candidates, float threshold);

} // namespace street_name
} // namespace loki
} // namespace valhalla
