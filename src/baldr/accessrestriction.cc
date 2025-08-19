#include "baldr/accessrestriction.h"
#include "baldr/rapidjson_utils.h"

#include <cstring>

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
                                     const uint64_t value,
                                     const bool except_destination)
    : edgeindex_(edgeindex), type_(static_cast<uint32_t>(type)), modes_(modes),
      except_destination_(except_destination), spare_(0), value_(value) {
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

bool AccessRestriction::except_destination() const {
  return static_cast<bool>(except_destination_);
}

void AccessRestriction::set_except_destination(const bool except_destination) {
  except_destination_ = static_cast<uint64_t>(except_destination);
}

// Get the value
uint64_t AccessRestriction::value() const {
  return value_;
}

// Set the value
void AccessRestriction::set_value(const uint64_t v) {
  value_ = v;
}

void AccessRestriction::json(rapidjson::writer_wrapper_t& writer) const {
  auto maybe_found = type_to_string.find(type());
  std::string restriction_type = "unsupported";
  if (maybe_found != type_to_string.cend()) {
    restriction_type = maybe_found->second;
  }

  writer.start_object();
  writer("type", restriction_type);
  writer("edge_index", static_cast<uint64_t>(edgeindex()));
  writer("bus", static_cast<bool>(modes_ & kBusAccess));
  writer("car", static_cast<bool>(modes_ & kAutoAccess));
  writer("emergency", static_cast<bool>(modes_ & kEmergencyAccess));
  writer("HOV", static_cast<bool>(modes_ & kHOVAccess));
  writer("pedestrian", static_cast<bool>(modes_ & kPedestrianAccess));
  writer("taxi", static_cast<bool>(modes_ & kTaxiAccess));
  writer("truck", static_cast<bool>(modes_ & kTruckAccess));
  writer("wheelchair", static_cast<bool>(modes_ & kWheelchairAccess));
  writer("moped", static_cast<bool>(modes_ & kMopedAccess));
  writer("motorcycle", static_cast<bool>(modes_ & kMotorcycleAccess));

  switch (type()) {
    case AccessType::kTimedAllowed:
    case AccessType::kTimedDenied:
    case AccessType::kDestinationAllowed:
      // TODO(nils): turn the time domain into a proper map
      writer.start_object("value");
      writer("time_domain", value());
      writer.end_object();
      break;
    case AccessType::kMaxAxles:
      writer("value", value());
      break;
    default:
      writer.set_precision(2);
      writer("value", static_cast<double>(value()) * 0.01);
      writer.set_precision(3);
  }
  writer.end_object();
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
