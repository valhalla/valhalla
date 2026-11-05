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

// lowercases, transliterates and drops anything that isn't alphanumeric, which is what makes
// `Hermannstraße` and `Hermann-Straße` the same string. Used on both the hint and the pool names
std::u32string normalize_name(std::string_view name);

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
