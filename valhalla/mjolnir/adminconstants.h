#ifndef VALHALLA_MJOLNIR_ADMINCONSTANTS_H_
#define VALHALLA_MJOLNIR_ADMINCONSTANTS_H_

#include <unordered_map>
#include <string>

#include <valhalla/baldr/graphconstants.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

enum class AccessTypes : uint8_t {
  kTrunk = 0,
  kTrunkLink = 1,
  kTrack = 2,
  kFootway = 3,
  kPedestrian = 4,
  kBridleway = 5,
  kCycleway = 6,
  kPath = 7
};

//Based on logic at http://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Access-Restrictions
//vector = trunk, trunk_link, track, footway, pedestrian, bridleway, cycleway, and path
//-1 indicates a null value will be set which means no change from default access.
//0 indicates no access.
const std::unordered_map<std::string, std::vector<int>> kCountryAccess {
    {"Australia",                 {-1, -1, -1, (kPedestrianAccess | kWheelchairAccess | kBicycleAccess),
                                   (kPedestrianAccess | kWheelchairAccess | kBicycleAccess),
                                   (kPedestrianAccess | kWheelchairAccess | kBicycleAccess),
                                   (kPedestrianAccess | kWheelchairAccess | kBicycleAccess), -1}},
    {"Austria",                   {(kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess),
                                   (kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess),
                                   -1, -1, -1, -1, -1,  (kPedestrianAccess | kWheelchairAccess)}},
    {"Belarus",                   {-1, -1, -1, (kPedestrianAccess | kWheelchairAccess | kBicycleAccess),
                                   (kPedestrianAccess | kWheelchairAccess | kBicycleAccess), -1,
                                   (kPedestrianAccess | kWheelchairAccess | kBicycleAccess), -1}},
    {"Belgium",                   {(kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess),
                                   (kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess),
                                   -1, -1, (kPedestrianAccess | kWheelchairAccess | kBicycleAccess),
                                   (kPedestrianAccess | kWheelchairAccess), (kPedestrianAccess | kWheelchairAccess | kBicycleAccess), -1}},
    {"Brazil",                    {-1, -1, -1, (kPedestrianAccess | kWheelchairAccess | kBicycleAccess),
                                    -1, (kPedestrianAccess | kWheelchairAccess | kBicycleAccess),
                                    (kPedestrianAccess | kWheelchairAccess | kBicycleAccess), -1}},
    {"Denmark",                   {(kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess),
                                   (kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess),
                                   -1, -1, -1, -1, (kPedestrianAccess | kWheelchairAccess | kBicycleAccess), -1}},
    {"France",                    {(kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess),
                                   (kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess), -1, -1,
                                   (kPedestrianAccess | kWheelchairAccess | kBicycleAccess), -1, -1, -1}},
    {"Finland",                   {-1, -1, -1, -1, (kPedestrianAccess | kWheelchairAccess | kBicycleAccess),
                                   -1, (kPedestrianAccess | kWheelchairAccess | kBicycleAccess), -1}},
    {"Hungary",                   {(kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess),
                                   (kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess),
                                   -1, -1, -1, -1, -1, -1}},
    {"The Netherlands",           {(kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess),
                                   (kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess),
                                   -1, -1, -1, -1, (kPedestrianAccess | kWheelchairAccess | kBicycleAccess), -1}},
    {"Norway",                    {-1,-1, -1,
                                   (kPedestrianAccess | kWheelchairAccess | kBicycleAccess), (kPedestrianAccess | kWheelchairAccess | kBicycleAccess),
                                   -1, (kPedestrianAccess | kWheelchairAccess | kBicycleAccess), -1}},
    {"Poland",                    {(kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess),
                                   (kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess),
                                   -1, -1, -1, -1, -1, -1}},
    {"Romania",                   {(kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess),
                                   (kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess),
                                   -1, -1, -1, -1, -1, -1}},
    {"Slovakia",                  {(kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess),
                                   (kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess),
                                   -1, -1, -1, -1, -1, -1}},
    {"Switzerland",               {(kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess),
                                   (kAutoAccess | kTruckAccess | kBusAccess | kHOVAccess),
                                   -1, -1, -1, -1, -1, -1}},
    {"Turkey",                    {-1, -1, -1, -1, -1, (kPedestrianAccess | kWheelchairAccess), -1, -1}},
    {"United Kingdom",            {-1, -1, -1, -1, -1, (kPedestrianAccess | kWheelchairAccess | kBicycleAccess),
                                   (kPedestrianAccess | kWheelchairAccess | kBicycleAccess), -1}},
    {"United States of America",  {-1, -1, -1, -1,
                                   (kPedestrianAccess | kWheelchairAccess | kBicycleAccess),
                                   (kPedestrianAccess | kWheelchairAccess | kBicycleAccess),
                                   (kPedestrianAccess | kWheelchairAccess | kBicycleAccess), -1}}
  };
}
}
#endif  // VALHALLA_MJOLNIR_ADMINCONSTANTS_H_
