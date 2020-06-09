#ifndef VALHALLA_MJOLNIR_ADMINCONSTANTS_H_
#define VALHALLA_MJOLNIR_ADMINCONSTANTS_H_

#include <string>
#include <unordered_map>
#include <vector>

#include <valhalla/baldr/graphconstants.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

enum class AccessTypes : uint16_t {
  kTrunk = 0,
  kTrunkLink = 1,
  kTrack = 2,
  kFootway = 3,
  kPedestrian = 4,
  kBridleway = 5,
  kCycleway = 6,
  kPath = 7,
  kMotorroad = 8
};

// Based on logic at http://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Access-Restrictions
// vector = trunk, trunk_link, track, footway, pedestrian, bridleway, cycleway, path, and motorroad
//-1 indicates a null value will be set which means no change from default access.
// 0 indicates no access.
const std::unordered_map<std::string, std::vector<int>>
    kCountryAccess{{"Australia",
                    {-1, -1, -1, (kPedestrianAccess | kWheelchairAccess | kBicycleAccess),
                     (kPedestrianAccess | kWheelchairAccess | kBicycleAccess),
                     (kPedestrianAccess | kWheelchairAccess | kBicycleAccess),
                     (kPedestrianAccess | kWheelchairAccess | kBicycleAccess), -1, -1}},
                   {"Austria",
                    {(kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess | kTaxiAccess |
                      kMotorcycleAccess),
                     (kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess | kTaxiAccess |
                      kMotorcycleAccess),
                     -1, -1, -1, -1, -1, (kPedestrianAccess | kWheelchairAccess), -1}},
                   {"Belarus",
                    {-1, -1, -1, (kPedestrianAccess | kWheelchairAccess | kBicycleAccess),
                     (kPedestrianAccess | kWheelchairAccess | kBicycleAccess), -1,
                     (kPedestrianAccess | kWheelchairAccess | kBicycleAccess), -1, -1}},
                   {"Belgium",
                    {(kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess | kTaxiAccess |
                      kMotorcycleAccess),
                     (kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess | kTaxiAccess |
                      kMotorcycleAccess),
                     -1, -1, (kPedestrianAccess | kWheelchairAccess | kBicycleAccess),
                     (kPedestrianAccess | kWheelchairAccess),
                     (kPedestrianAccess | kWheelchairAccess | kBicycleAccess | kMopedAccess),
                     (kPedestrianAccess | kWheelchairAccess | kBicycleAccess | kMopedAccess), -1}},
                   {"Brazil",
                    {-1, -1, -1,
                     (kPedestrianAccess | kWheelchairAccess | kBicycleAccess | kMopedAccess), -1,
                     (kPedestrianAccess | kWheelchairAccess | kBicycleAccess | kMopedAccess),
                     (kPedestrianAccess | kWheelchairAccess | kBicycleAccess | kMopedAccess),
                     (kPedestrianAccess | kWheelchairAccess | kBicycleAccess | kMopedAccess), -1}},
                   {"Denmark",
                    {(kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess | kTaxiAccess |
                      kMotorcycleAccess),
                     (kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess | kTaxiAccess |
                      kMotorcycleAccess),
                     (kPedestrianAccess | kWheelchairAccess | kBicycleAccess), -1, -1, -1,
                     (kPedestrianAccess | kWheelchairAccess | kBicycleAccess), -1, -1}},
                   {"France",
                    {(kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess | kTaxiAccess |
                      kMotorcycleAccess),
                     (kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess | kTaxiAccess |
                      kMotorcycleAccess),
                     -1, -1, (kPedestrianAccess | kWheelchairAccess | kBicycleAccess), -1, -1,
                     (kPedestrianAccess | kWheelchairAccess | kBicycleAccess | kMopedAccess), -1}},
                   {"Finland",
                    {-1, -1, -1, -1, (kPedestrianAccess | kWheelchairAccess | kBicycleAccess), -1,
                     (kPedestrianAccess | kWheelchairAccess | kBicycleAccess), -1, -1}},
                   {"Hungary",
                    {(kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess | kTaxiAccess |
                      kMotorcycleAccess),
                     (kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess | kTaxiAccess |
                      kMotorcycleAccess),
                     -1, -1, -1, -1, -1, -1, -1}},
                   {"Netherlands",
                    {(kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess | kTaxiAccess |
                      kMotorcycleAccess),
                     (kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess | kTaxiAccess |
                      kMotorcycleAccess),
                     -1, -1, -1, -1, (kPedestrianAccess | kWheelchairAccess | kBicycleAccess),
                     (kPedestrianAccess | kWheelchairAccess | kBicycleAccess | kMopedAccess), -1}},
                   {"Norway",
                    {-1, -1, -1, (kPedestrianAccess | kWheelchairAccess | kBicycleAccess),
                     (kPedestrianAccess | kWheelchairAccess | kBicycleAccess), -1,
                     (kPedestrianAccess | kWheelchairAccess | kBicycleAccess), -1, -1}},
                   {"Poland",
                    {(kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess | kTaxiAccess |
                      kMotorcycleAccess),
                     (kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess | kTaxiAccess |
                      kMotorcycleAccess),
                     -1, -1, -1, -1, -1, -1, -1}},
                   {"Romania",
                    {(kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess | kTaxiAccess |
                      kMotorcycleAccess),
                     (kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess | kTaxiAccess |
                      kMotorcycleAccess),
                     -1, -1, -1, -1, -1,
                     (kPedestrianAccess | kWheelchairAccess | kBicycleAccess | kMopedAccess), -1}},
                   {"Russia", {-1, -1, -1, -1, -1, -1, (kMopedAccess | kBicycleAccess), -1, -1}},
                   {"Slovakia",
                    {(kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess | kTaxiAccess |
                      kMotorcycleAccess),
                     (kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess | kTaxiAccess |
                      kMotorcycleAccess),
                     -1, -1, -1, -1, -1, -1, -1}},
                   {"Spain",
                    {-1, -1, -1, -1, -1, -1, -1,
                     (kPedestrianAccess | kWheelchairAccess | kBicycleAccess | kMopedAccess),
                     (kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess | kTaxiAccess |
                      kMotorcycleAccess | kBicycleAccess)}},
                   {"Switzerland",
                    {(kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess | kTaxiAccess |
                      kMotorcycleAccess),
                     (kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess | kTaxiAccess |
                      kMotorcycleAccess),
                     -1, -1, -1, -1, (kBicycleAccess | kMopedAccess), -1, -1}},
                   {"Turkey",
                    {-1, -1, -1, -1, -1, (kPedestrianAccess | kWheelchairAccess), -1, -1, -1}},
                   {"United Kingdom",
                    {-1, -1, -1, -1, -1, (kPedestrianAccess | kWheelchairAccess | kBicycleAccess),
                     (kPedestrianAccess | kWheelchairAccess | kBicycleAccess), -1, -1}},
                   {"United States",
                    {-1, -1, -1, -1, (kPedestrianAccess | kWheelchairAccess | kBicycleAccess),
                     (kPedestrianAccess | kWheelchairAccess | kBicycleAccess),
                     (kPedestrianAccess | kWheelchairAccess | kBicycleAccess),
                     (kPedestrianAccess | kWheelchairAccess | kBicycleAccess | kMopedAccess), -1}}};
} // namespace mjolnir
} // namespace valhalla
#endif // VALHALLA_MJOLNIR_ADMINCONSTANTS_H_
