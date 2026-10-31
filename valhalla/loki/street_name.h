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

// a hint is used iff some candidate within the hint radius scores at least this
constexpr float kTau = 0.6f;

// (name, is route number) as returned by EdgeInfo::GetNames(bool)
using names_t = std::vector<std::pair<std::string, bool>>;

// hint radius in meters for an edge density in [0, kMaxDensity]
double hint_radius(uint32_t density);

std::string normalize(std::string_view name);

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
