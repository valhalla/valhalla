#include "baldr/admin.h"

namespace valhalla {
namespace baldr {

// Constructor given parameters.
Admin::Admin(const uint32_t country_offset, const uint32_t state_offset,
             const std::string& country_iso, const std::string& state_iso,
             const std::string& start_dst, const std::string& end_dst)
    : country_offset_(country_offset), state_offset_(state_offset){

  std::size_t length = 0;
  // Example:  GB or US
  if (country_iso.size() == kCountryIso-1) {
    length = country_iso.copy(country_iso_,kCountryIso-1);
    country_iso_[length]='\0';
  }
  else country_iso_[0]= '\0';

  // Example:  PA
  if (state_iso.size() == kStateIso-2) {
    length = state_iso.copy(state_iso_,kStateIso-2);
    state_iso_[length]='\0';
  }
  // Example:  WLS
  else if (state_iso.size() == kStateIso-1) {
    length = state_iso.copy(state_iso_,kStateIso-1);
    state_iso_[length]='\0';
  }
  else state_iso_[0]= '\0';

  // YYYYMMDD
  end_dst.copy(end_dst_,kDst);
  // YYYYMMDD
  start_dst.copy(start_dst_,kDst);
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
const std::string Admin::country_iso() const {
  return std::string(country_iso_);
}

// country ISO + dash + state ISO will give you ISO3166-2 for state.
const std::string Admin::state_iso() const {
  return std::string(state_iso_);
}

// When does daylight saving time start?
const char* Admin::start_dst() const {
  return start_dst_ + '\0';
}

// When does daylight saving time end?
const char* Admin::end_dst() const {
  return end_dst_ + '\0';
}

}
}
