#include "baldr/accessrestriction.h"
#include "baldr/timedomain.h"
#include <string.h>

namespace vb = valhalla::baldr;

namespace {
const std::unordered_map<vb::AccessType, std::string> type_to_string = {
    {vb::AccessType::kHazmat, "hazmat"},
    {vb::AccessType::kMaxHeight, "max_height"},
    {vb::AccessType::kMaxWidth, "max_width"},
    {vb::AccessType::kMaxLength, "max_length"},
    {vb::AccessType::kMaxWeight, "max_weight"},
    {vb::AccessType::kMaxAxleLoad, "max_axle_load"},
    {vb::AccessType::kTimedAllowed, "timed_allowed"},
    {vb::AccessType::kTimedDenied, "timed_denied"},
    {vb::AccessType::kDestinationAllowed, "destination_allowed"},
    {vb::AccessType::kMaxAxles, "max_axles"},
};
}

namespace valhalla {
namespace baldr {

// Constructor with arguments
AccessRestriction::AccessRestriction(const uint32_t edgeindex,
                                     const AccessType type,
                                     const uint32_t modes,
                                     const uint64_t value)
    : edgeindex_(edgeindex), type_(static_cast<uint32_t>(type)), modes_(modes), spare_(0),
      value_(value) {
}

// Get the internal edge Id.
uint32_t AccessRestriction::edgeindex() const {
  return edgeindex_;
}

// Set the internal edge index to which this access restriction applies.
void AccessRestriction::set_edgeindex(const uint32_t edgeindex) {
  edgeindex_ = edgeindex;
}

// Get the type
AccessType AccessRestriction::type() const {
  return static_cast<AccessType>(type_);
}

// Get the modes impacted by access restriction.
uint32_t AccessRestriction::modes() const {
  return modes_;
}

// Get the value
uint64_t AccessRestriction::value() const {
  return value_;
}

// Set the value
void AccessRestriction::set_value(const uint64_t v) {
  value_ = v;
}

const json::MapPtr AccessRestriction::json() const {
  auto maybe_found = type_to_string.find(type());
  std::string restriction_type = "unsupported";
  if (maybe_found != type_to_string.cend()) {
    restriction_type = maybe_found->second;
  }
  auto map = json::map({{"type", restriction_type},
                        {"edge_index", static_cast<uint64_t>(edgeindex())},
                        {"bus", static_cast<bool>(modes_ & kBusAccess)},
                        {"car", static_cast<bool>(modes_ & kAutoAccess)},
                        {"emergency", static_cast<bool>(modes_ & kEmergencyAccess)},
                        {"HOV", static_cast<bool>(modes_ & kHOVAccess)},
                        {"pedestrian", static_cast<bool>(modes_ & kPedestrianAccess)},
                        {"taxi", static_cast<bool>(modes_ & kTaxiAccess)},
                        {"truck", static_cast<bool>(modes_ & kTruckAccess)},
                        {"wheelchair", static_cast<bool>(modes_ & kWheelchairAccess)},
                        {"moped", static_cast<bool>(modes_ & kMopedAccess)},
                        {"motorcycle", static_cast<bool>(modes_ & kMotorcycleAccess)}});

  switch (type()) {
    case AccessType::kTimedAllowed:
    case AccessType::kTimedDenied:
    case AccessType::kDestinationAllowed:
      // TODO(nils): turn the time domain into a proper map
      map->emplace("value", json::map({{std::string("time_domain"), value()}}));
      break;
    case AccessType::kMaxAxles:
      map->emplace("value", value());
      break;
    default:
      map->emplace("value", json::fixed_t{static_cast<double>(value()) * 0.01, 2});
  }

  return map;
}

// operator < - for sorting. Sort by route Id.
bool AccessRestriction::operator<(const AccessRestriction& other) const {
  if (edgeindex() == other.edgeindex()) {
    if (modes() == other.modes()) {
      if (type() == other.type()) {
        return value() < other.value();
      }
      return type() < other.type();
    }
    return modes() < other.modes();
  }
  return edgeindex() < other.edgeindex();
}

} // namespace baldr
} // namespace valhalla
