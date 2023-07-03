#include "mjolnir/countryaccess.h"

namespace valhalla {
namespace mjolnir {

// Removes or adds access.
uint32_t
ProcessAccess(const uint32_t current_access, const uint32_t country_access, const uint32_t type) {

  uint32_t new_access = current_access;

  auto current = ((type & current_access) == type);
  auto country = ((type & country_access) == type);

  if (current && !country) {
    new_access &= ~(type);
  } else if (!current && country) {
    new_access |= type;
  }

  return new_access;
}

// Get the new access for a DE.  If a user entered tags then we will not update
// the access.  Also, we have to take into account if the DE is oneway or not.
uint32_t GetAccess(const uint32_t current_access,
                   const uint32_t country_access,
                   const bool oneway_vehicle,
                   const bool oneway_bicycle,
                   const bool oneway_pedestrian,
                   const OSMAccess& user_access) {

  uint32_t new_access = current_access;

  if (!oneway_pedestrian && !user_access.foot_tag()) {
    new_access = ProcessAccess(new_access, country_access, (kPedestrianAccess | kWheelchairAccess));
  }

  if (!oneway_bicycle && !user_access.bike_tag()) {
    new_access = ProcessAccess(new_access, country_access, kBicycleAccess);
  }

  // if the reverse direction is oneway then do not add access for vehicles in the current
  // direction.
  if (oneway_vehicle) {
    return new_access;
  }

  if (!user_access.auto_tag()) {
    new_access = ProcessAccess(new_access, country_access, kAutoAccess);
  }

  if (!user_access.bus_tag()) {
    new_access = ProcessAccess(new_access, country_access, kBusAccess);
  }

  if (!user_access.truck_tag()) {
    new_access = ProcessAccess(new_access, country_access, kTruckAccess);
  }

  if (!user_access.hov_tag()) {
    new_access = ProcessAccess(new_access, country_access, kHOVAccess);
  }

  if (!user_access.taxi_tag()) {
    new_access = ProcessAccess(new_access, country_access, kTaxiAccess);
  }

  if (!user_access.moped_tag()) {
    new_access = ProcessAccess(new_access, country_access, kMopedAccess);
  }

  if (!user_access.motorcycle_tag())
    new_access = ProcessAccess(new_access, country_access, kMotorcycleAccess);

  return new_access;
}

// Set the country access for a DE.
void SetCountryAccess(DirectedEdge& directededge,
                      const std::vector<int>& country_access,
                      const OSMAccess& user_access) {

  uint32_t forward = directededge.forwardaccess();
  uint32_t reverse = directededge.reverseaccess();

  bool f_oneway_vehicle = (((forward & kAutoAccess) && !(reverse & kAutoAccess)) ||
                           ((forward & kTruckAccess) && !(reverse & kTruckAccess)) ||
                           ((forward & kEmergencyAccess) && !(reverse & kEmergencyAccess)) ||
                           ((forward & kTaxiAccess) && !(reverse & kTaxiAccess)) ||
                           ((forward & kHOVAccess) && !(reverse & kHOVAccess)) ||
                           ((forward & kMopedAccess) && !(reverse & kMopedAccess)) ||
                           ((forward & kMotorcycleAccess) && !(reverse & kMotorcycleAccess)) ||
                           ((forward & kBusAccess) && !(reverse & kBusAccess)));

  bool r_oneway_vehicle = ((!(forward & kAutoAccess) && (reverse & kAutoAccess)) ||
                           (!(forward & kTruckAccess) && (reverse & kTruckAccess)) ||
                           (!(forward & kEmergencyAccess) && (reverse & kEmergencyAccess)) ||
                           (!(forward & kTaxiAccess) && (reverse & kTaxiAccess)) ||
                           (!(forward & kHOVAccess) && (reverse & kHOVAccess)) ||
                           (!(forward & kMopedAccess) && (reverse & kMopedAccess)) ||
                           (!(forward & kMotorcycleAccess) && (reverse & kMotorcycleAccess)) ||
                           (!(forward & kBusAccess) && (reverse & kBusAccess)));

  bool f_oneway_bicycle = ((forward & kBicycleAccess) && !(reverse & kBicycleAccess));
  bool r_oneway_bicycle = (!(forward & kBicycleAccess) && (reverse & kBicycleAccess));

  bool f_oneway_pedestrian = ((forward & kPedestrianAccess) && !(reverse & kPedestrianAccess));
  bool r_oneway_pedestrian = (!(forward & kPedestrianAccess) && (reverse & kPedestrianAccess));

  // country_access.at(X) = -1 means that no default country overrides are needed.
  // user_access.<type>_tag() == true means that a user set the <type> tag.

  // motorroad override.  Only applies to RC <= kPrimary.  If no override is found in the
  // country access, just use the defaults which is no bicycles, mopeds or pedestrians.
  if (directededge.classification() <= RoadClass::kPrimary && user_access.motorroad_tag()) {
    if (country_access.at(static_cast<uint32_t>(AccessTypes::kMotorroad)) != -1) {

      forward = GetAccess(forward, country_access.at(static_cast<uint32_t>(AccessTypes::kMotorroad)),
                          r_oneway_vehicle, r_oneway_bicycle, r_oneway_pedestrian, user_access);
      reverse = GetAccess(reverse, country_access.at(static_cast<uint32_t>(AccessTypes::kMotorroad)),
                          f_oneway_vehicle, f_oneway_bicycle, f_oneway_pedestrian, user_access);
    } else {
      // do the trunk and trunk_link logic first
      if (directededge.classification() == RoadClass::kTrunk) {
        if (directededge.link() &&
            country_access.at(static_cast<uint32_t>(AccessTypes::kTrunkLink)) != -1) {

          forward =
              GetAccess(forward, country_access.at(static_cast<uint32_t>(AccessTypes::kTrunkLink)),
                        r_oneway_vehicle, r_oneway_bicycle, r_oneway_pedestrian, user_access);
          reverse =
              GetAccess(reverse, country_access.at(static_cast<uint32_t>(AccessTypes::kTrunkLink)),
                        f_oneway_vehicle, f_oneway_bicycle, f_oneway_pedestrian, user_access);
        } else if (country_access.at(static_cast<uint32_t>(AccessTypes::kTrunk)) != -1) {

          forward = GetAccess(forward, country_access.at(static_cast<uint32_t>(AccessTypes::kTrunk)),
                              r_oneway_vehicle, r_oneway_bicycle, r_oneway_pedestrian, user_access);
          reverse = GetAccess(reverse, country_access.at(static_cast<uint32_t>(AccessTypes::kTrunk)),
                              f_oneway_vehicle, f_oneway_bicycle, f_oneway_pedestrian, user_access);
        }
      }
      // now remove pedestian, moped and bike access if it is set.  This is the default for
      // motorroad_tag.
      forward = GetAccess(forward,
                          (forward &
                           ~(kPedestrianAccess | kWheelchairAccess | kMopedAccess | kBicycleAccess)),
                          r_oneway_vehicle, r_oneway_bicycle, r_oneway_pedestrian, user_access);
      reverse = GetAccess(reverse,
                          (reverse &
                           ~(kPedestrianAccess | kWheelchairAccess | kMopedAccess | kBicycleAccess)),
                          f_oneway_vehicle, f_oneway_bicycle, f_oneway_pedestrian, user_access);
    }
  } // trunk and trunk_link
  else if (directededge.classification() == RoadClass::kTrunk) {
    if (directededge.link() &&
        country_access.at(static_cast<uint32_t>(AccessTypes::kTrunkLink)) != -1) {
      forward = GetAccess(forward, country_access.at(static_cast<uint32_t>(AccessTypes::kTrunkLink)),
                          r_oneway_vehicle, r_oneway_bicycle, r_oneway_pedestrian, user_access);
      reverse = GetAccess(reverse, country_access.at(static_cast<uint32_t>(AccessTypes::kTrunkLink)),
                          f_oneway_vehicle, f_oneway_bicycle, f_oneway_pedestrian, user_access);
    } else if (country_access.at(static_cast<uint32_t>(AccessTypes::kTrunk)) != -1) {
      forward = GetAccess(forward, country_access.at(static_cast<uint32_t>(AccessTypes::kTrunk)),
                          r_oneway_vehicle, r_oneway_bicycle, r_oneway_pedestrian, user_access);
      reverse = GetAccess(reverse, country_access.at(static_cast<uint32_t>(AccessTypes::kTrunk)),
                          f_oneway_vehicle, f_oneway_bicycle, f_oneway_pedestrian, user_access);
    }
  }
  // track, footway, pedestrian, bridleway, cycleway, and path
  if (directededge.use() == Use::kTrack &&
      country_access.at(static_cast<uint32_t>(AccessTypes::kTrack)) != -1) {
    forward = GetAccess(forward, country_access.at(static_cast<uint32_t>(AccessTypes::kTrack)),
                        r_oneway_vehicle, r_oneway_bicycle, r_oneway_pedestrian, user_access);
    reverse = GetAccess(reverse, country_access.at(static_cast<uint32_t>(AccessTypes::kTrack)),
                        f_oneway_vehicle, f_oneway_bicycle, f_oneway_pedestrian, user_access);
  } else if ((directededge.use() == Use::kFootway || directededge.use() == Use::kSidewalk ||
              directededge.use() == Use::kPedestrianCrossing) &&
             country_access.at(static_cast<uint32_t>(AccessTypes::kFootway)) != -1) {
    forward = GetAccess(forward, country_access.at(static_cast<uint32_t>(AccessTypes::kFootway)),
                        r_oneway_vehicle, r_oneway_bicycle, r_oneway_pedestrian, user_access);
    reverse = GetAccess(reverse, country_access.at(static_cast<uint32_t>(AccessTypes::kFootway)),
                        f_oneway_vehicle, f_oneway_bicycle, f_oneway_pedestrian, user_access);

  } else if (directededge.use() == Use::kPedestrian &&
             country_access.at(static_cast<uint32_t>(AccessTypes::kPedestrian)) != -1) {

    forward = GetAccess(forward, country_access.at(static_cast<uint32_t>(AccessTypes::kPedestrian)),
                        r_oneway_vehicle, r_oneway_bicycle, r_oneway_pedestrian, user_access);
    reverse = GetAccess(reverse, country_access.at(static_cast<uint32_t>(AccessTypes::kPedestrian)),
                        f_oneway_vehicle, f_oneway_bicycle, f_oneway_pedestrian, user_access);
  } else if (directededge.use() == Use::kBridleway &&
             country_access.at(static_cast<uint32_t>(AccessTypes::kBridleway)) != -1) {
    forward = GetAccess(forward, country_access.at(static_cast<uint32_t>(AccessTypes::kBridleway)),
                        r_oneway_vehicle, r_oneway_bicycle, r_oneway_pedestrian, user_access);
    reverse = GetAccess(reverse, country_access.at(static_cast<uint32_t>(AccessTypes::kBridleway)),
                        f_oneway_vehicle, f_oneway_bicycle, f_oneway_pedestrian, user_access);
  } else if (directededge.use() == Use::kCycleway &&
             country_access.at(static_cast<uint32_t>(AccessTypes::kCycleway)) != -1) {
    forward = GetAccess(forward, country_access.at(static_cast<uint32_t>(AccessTypes::kCycleway)),
                        r_oneway_vehicle, r_oneway_bicycle, r_oneway_pedestrian, user_access);
    reverse = GetAccess(reverse, country_access.at(static_cast<uint32_t>(AccessTypes::kCycleway)),
                        f_oneway_vehicle, f_oneway_bicycle, f_oneway_pedestrian, user_access);

  } else if (directededge.use() == Use::kPath &&
             country_access.at(static_cast<uint32_t>(AccessTypes::kPath)) != -1) {
    forward = GetAccess(forward, country_access.at(static_cast<uint32_t>(AccessTypes::kPath)),
                        r_oneway_vehicle, r_oneway_bicycle, r_oneway_pedestrian, user_access);
    reverse = GetAccess(reverse, country_access.at(static_cast<uint32_t>(AccessTypes::kPath)),
                        f_oneway_vehicle, f_oneway_bicycle, f_oneway_pedestrian, user_access);
  }

  directededge.set_forwardaccess(forward);
  directededge.set_reverseaccess(reverse);
}

} // namespace mjolnir
} // namespace valhalla
