#include "baldr/admininfo.h"

namespace valhalla {
namespace baldr {

// Returns true if the specified object is equal to this object.
bool AdminInfo::operator ==(const AdminInfo& rhs) const {
  return (country_iso_ == rhs.country_iso_
      && country_text_ == rhs.country_text_
      && state_iso_ == rhs.state_iso_
      && state_text_ == rhs.state_text_);
}

// Get the country text
const std::string& AdminInfo::country_text() const {
  return country_text_;
}

// Get the state text
const std::string& AdminInfo::state_text() const {
  return state_text_;
}

// Get the country iso
const std::string& AdminInfo::country_iso() const {
  return country_iso_;
}

// Get the state iso
const std::string& AdminInfo::state_iso() const {
  return state_iso_;
}

// Constructor
AdminInfo::AdminInfo(const std::string& country_text, const std::string& state_text,
                     const std::string& country_iso, const std::string& state_iso)
    : country_text_(country_text),
      state_text_(state_text),
      country_iso_(country_iso),
      state_iso_(state_iso) {
}

}
}
