#include "mjolnir/countryaccess.h"

namespace valhalla {
namespace mjolnir {

// Removes or adds access.
uint32_t ProcessAccess(const uint32_t current_access, const uint32_t country_access,
                       const uint32_t type) {

  uint32_t new_access = current_access;

  auto current = ((type & current_access) == type);
  auto country = ((type & country_access) == type);

  if (current && !country)
    new_access &= ~(type);
  else if (!current && country)
    new_access |= type;

  return new_access;
}

// Get the new access for a DE.  If a user entered tags then we will not update
// the access.  Also, we have to take into account if the DE is oneway or not.
uint32_t GetAccess(const uint32_t current_access, const uint32_t country_access,
                   const bool oneway_vehicle, const bool oneway_bicycle,
                   const OSMAccess& user_access) {

  uint32_t new_access = current_access;

  if (!user_access.foot_tag())
    new_access = ProcessAccess(new_access,country_access,(kPedestrianAccess | kWheelchairAccess));

  if (!oneway_bicycle && !user_access.bike_tag())
    new_access = ProcessAccess(new_access,country_access,kBicycleAccess);

  // if the reverse direction is oneway then do not add access for vehicles in the current direction.
  if (oneway_vehicle)
    return new_access;

  if (!user_access.auto_tag())
    new_access = ProcessAccess(new_access,country_access,kAutoAccess);

  if (!user_access.bus_tag())
    new_access = ProcessAccess(new_access,country_access,kBusAccess);

  if (!user_access.truck_tag())
    new_access = ProcessAccess(new_access,country_access,kTruckAccess);

  if (!user_access.hov_tag())
    new_access = ProcessAccess(new_access,country_access,kHOVAccess);

  return new_access;
}

// Set the country access for a DE.
void SetCountryAccess(DirectedEdge& directededge, const std::vector<int>& country_access,
                      const OSMAccess& user_access) {

  uint32_t forward = directededge.forwardaccess();
  uint32_t reverse = directededge.reverseaccess();

  bool f_oneway_vehicle = (((forward & kAutoAccess) && !(reverse & kAutoAccess)) ||
      ((forward & kTruckAccess) && !(reverse & kTruckAccess)) ||
      ((forward & kEmergencyAccess) && !(reverse & kEmergencyAccess)) ||
      ((forward & kTaxiAccess) && !(reverse & kTaxiAccess)) ||
      ((forward & kHOVAccess) && !(reverse & kHOVAccess)) ||
      ((forward & kBusAccess) && !(reverse & kBusAccess)));

  bool r_oneway_vehicle = ((!(forward & kAutoAccess) && (reverse & kAutoAccess)) ||
      (!(forward & kTruckAccess) && (reverse & kTruckAccess)) ||
      (!(forward & kEmergencyAccess) && (reverse & kEmergencyAccess)) ||
      (!(forward & kTaxiAccess) && (reverse & kTaxiAccess)) ||
      (!(forward & kHOVAccess) && (reverse & kHOVAccess)) ||
      (!(forward & kBusAccess) && (reverse & kBusAccess)));

  bool f_oneway_bicycle = ((forward & kBicycleAccess) && !(reverse & kBicycleAccess));
  bool r_oneway_bicycle = (!(forward & kBicycleAccess) && (reverse & kBicycleAccess));

  // country_access.at(X) = -1 means that no default country overrides are needed.
  // user_access.<type>_tag() == true means that a user set the <type> tag.

  // trunk and trunk_link
  if (directededge.classification() == RoadClass::kTrunk) {
    if (directededge.link() && country_access.at(static_cast<uint32_t>(AccessTypes::kTrunkLink)) != -1) {
      forward = GetAccess(forward, country_access.at(static_cast<uint32_t>(AccessTypes::kTrunkLink)),
                          r_oneway_vehicle, r_oneway_bicycle, user_access);
      reverse = GetAccess(reverse, country_access.at(static_cast<uint32_t>(AccessTypes::kTrunkLink)),
                          f_oneway_vehicle, f_oneway_bicycle, user_access);
    } else if (country_access.at(static_cast<uint32_t>(AccessTypes::kTrunk)) != -1) {
      forward = GetAccess(forward, country_access.at(static_cast<uint32_t>(AccessTypes::kTrunk)),
                          r_oneway_vehicle, r_oneway_bicycle, user_access);
      reverse = GetAccess(reverse, country_access.at(static_cast<uint32_t>(AccessTypes::kTrunk)),
                          f_oneway_vehicle, f_oneway_bicycle, user_access);
    }
  }

  // track, footway, pedestrian, bridleway, cycleway, and path
  if (directededge.use() == Use::kTrack && country_access.at(static_cast<uint32_t>(AccessTypes::kTrack)) != -1) {
    forward = GetAccess(forward, country_access.at(static_cast<uint32_t>(AccessTypes::kTrack)),
                        r_oneway_vehicle, r_oneway_bicycle, user_access);
    reverse = GetAccess(reverse, country_access.at(static_cast<uint32_t>(AccessTypes::kTrack)),
                        f_oneway_vehicle, f_oneway_bicycle, user_access);
  } else if (directededge.use() == Use::kFootway && country_access.at(static_cast<uint32_t>(AccessTypes::kFootway)) != -1) {
    forward = GetAccess(forward, country_access.at(static_cast<uint32_t>(AccessTypes::kFootway)),
                        r_oneway_vehicle, r_oneway_bicycle, user_access);
    reverse = GetAccess(reverse, country_access.at(static_cast<uint32_t>(AccessTypes::kFootway)),
                        f_oneway_vehicle, f_oneway_bicycle, user_access);

  } else if (directededge.use() == Use::kPedestrian && country_access.at(static_cast<uint32_t>(AccessTypes::kPedestrian)) != -1) {

    forward = GetAccess(forward, country_access.at(static_cast<uint32_t>(AccessTypes::kPedestrian)),
                        r_oneway_vehicle, r_oneway_bicycle, user_access);
    reverse = GetAccess(reverse, country_access.at(static_cast<uint32_t>(AccessTypes::kPedestrian)),
                        f_oneway_vehicle, f_oneway_bicycle, user_access);
  } else if (directededge.use() == Use::kBridleway && country_access.at(static_cast<uint32_t>(AccessTypes::kBridleway)) != -1) {
    forward = GetAccess(forward, country_access.at(static_cast<uint32_t>(AccessTypes::kBridleway)),
                        r_oneway_vehicle, r_oneway_bicycle, user_access);
    reverse = GetAccess(reverse, country_access.at(static_cast<uint32_t>(AccessTypes::kBridleway)),
                        f_oneway_vehicle, f_oneway_bicycle, user_access);
  } else if (directededge.use() == Use::kCycleway && country_access.at(static_cast<uint32_t>(AccessTypes::kCycleway)) != -1) {
    forward = GetAccess(forward, country_access.at(static_cast<uint32_t>(AccessTypes::kCycleway)),
                        r_oneway_vehicle, r_oneway_bicycle, user_access);
    reverse = GetAccess(reverse, country_access.at(static_cast<uint32_t>(AccessTypes::kCycleway)),
                        f_oneway_vehicle, f_oneway_bicycle, user_access);

  } else if (directededge.use() == Use::kPath && country_access.at(static_cast<uint32_t>(AccessTypes::kPath)) != -1) {
    forward = GetAccess(forward, country_access.at(static_cast<uint32_t>(AccessTypes::kPath)),
                        r_oneway_vehicle, r_oneway_bicycle, user_access);
    reverse = GetAccess(reverse, country_access.at(static_cast<uint32_t>(AccessTypes::kPath)),
                        f_oneway_vehicle, f_oneway_bicycle, user_access);
  }

  directededge.set_forwardaccess(forward);
  directededge.set_reverseaccess(reverse);
}

}
}
