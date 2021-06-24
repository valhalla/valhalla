#pragma once

#include <string>

namespace valhalla {
namespace tyr {

// Returns the 3-char equivalent of the 2-char country code (iso_3166_1_alpha2) or an empty string
// if the 2-char code is unknown
std::string get_iso_3166_1_alpha3(const std::string& iso_3166_1_alpha2);

} // namespace tyr
} // namespace valhalla
