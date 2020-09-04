#include "baldr/admin.h"

namespace valhalla {
namespace baldr {

// Constructor given parameters.
Admin::Admin(const uint32_t country_offset,
             const uint32_t state_offset,
             const std::string& country_iso,
             const std::string& state_iso)
    : country_offset_(country_offset), state_offset_(state_offset) {

  std::size_t length = 0;
  // Example:  GB or US
  if (country_iso.size() == kCountryIso) {
    length = country_iso.copy(country_iso_, kCountryIso);
  } else {
    country_iso_[0] = '\0';
  }

  // Example:  PA
  if (state_iso.size() == kStateIso - 1) {
    length = state_iso.copy(state_iso_, kStateIso - 1);
    state_iso_[length] = '\0';
  }
  // Example:  WLS
  else if (state_iso.size() == kStateIso) {
    length = state_iso.copy(state_iso_, kStateIso);
  } else {
    state_iso_[0] = '\0';
  }
}

// Get the offset within the text/names list for the state text.
uint32_t Admin::state_offset() const {
  return state_offset_;
}

// Get the offset within the text/names list for the country text.
uint32_t Admin::country_offset() const {
  return country_offset_;
}

// country ISO3166-1
std::string Admin::country_iso() const {
  std::string str;
  for (int i = 0; i < kCountryIso; i++) {
    if (country_iso_[i] == '\0') {
      break;
    }
    str.append(1, country_iso_[i]);
  }
  return str;
}

// country ISO + dash + state ISO will give you ISO3166-2 for state.
std::string Admin::state_iso() const {
  std::string str;
  for (int i = 0; i < kStateIso; i++) {
    if (state_iso_[i] == '\0') {
      break;
    }
    str.append(1, state_iso_[i]);
  }
  return str;
}

} // namespace baldr
} // namespace valhalla
