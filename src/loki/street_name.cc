#include "loki/street_name.h"

#include <algorithm>
#include <cctype>

namespace valhalla {
namespace loki {
namespace street_name {

// TODO: derive from density
double hint_radius(uint32_t /*density*/) {
  return 100.;
}

// TODO: transliteration table, punctuation, whitespace
std::string normalize(std::string_view name) {
  std::string normalized(name);
  std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return normalized;
}

// TODO: idf weighted trigram containment, ref regime, retries
std::vector<float> score(std::string_view hint, const std::vector<names_t>& candidates) {
  const auto normalized_hint = normalize(hint);
  std::vector<float> scores;
  scores.reserve(candidates.size());
  for (const auto& names : candidates) {
    float best = 0.f;
    for (const auto& [name, is_ref] : names) {
      if (normalize(name) == normalized_hint) {
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
