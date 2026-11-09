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

/**
 * Normalizes a street name: lowercases, decomposes and strips non-alphanumeric characters.
 * Used on both the hint and the pool names.
 *
 * @param name the string to be normalized
 * @returns the utf-32 representation of the normalized string
 */
std::u32string normalize_name(std::string_view name);

/**
 * Calculates the hint score for each candidate using inverse document frequency weighting on trigrams
 * over the whole candidate pool. A candidate's score is the max over its names.
 *
 * @param hint the request's street name hint
 * @param pool the collection of all candidates' names
 * @returns the score of each candidate in pool order. A score of 0 means the candidate shares
 *          nothing with the hint
 */
std::vector<float> score_candidates(std::string_view hint, const std::vector<names_t>& pool);

} // namespace street_name
} // namespace loki
} // namespace valhalla
