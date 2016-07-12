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
    {"Australia",                 {-1, -1, -1, (kPedestrianAccess | kBicycleAccess),
                                   (kPedestrianAccess | kBicycleAccess),
                                   (kPedestrianAccess | kBicycleAccess),
                                   (kPedestrianAccess | kBicycleAccess), -1}},
    {"Austria",                   {kAllAccess & ~(kPedestrianAccess | kBicycleAccess | kEmergencyAccess | kTaxiAccess | kHOVAccess),
                                   kAllAccess & ~(kPedestrianAccess | kBicycleAccess | kEmergencyAccess | kTaxiAccess | kHOVAccess),
                                   -1, -1, -1, -1, -1,  kPedestrianAccess}},
    {"Belarus",                   {-1, -1, -1, (kPedestrianAccess | kBicycleAccess),
                                   (kPedestrianAccess | kBicycleAccess), -1,
                                   (kPedestrianAccess | kBicycleAccess), -1}},
    {"Belgium",                   {kAllAccess & ~(kPedestrianAccess | kBicycleAccess | kEmergencyAccess | kTaxiAccess | kHOVAccess),
                                   kAllAccess & ~(kPedestrianAccess | kBicycleAccess | kEmergencyAccess | kTaxiAccess | kHOVAccess),
                                   -1, -1, (kPedestrianAccess | kBicycleAccess),
                                   kPedestrianAccess, (kPedestrianAccess | kBicycleAccess), -1}},
    {"Brazil",                    {-1, -1, -1, (kPedestrianAccess | kBicycleAccess),
                                    -1, (kPedestrianAccess | kBicycleAccess),
                                    (kPedestrianAccess | kBicycleAccess), -1}},
    {"Denmark",                   {kAllAccess & ~(kPedestrianAccess | kBicycleAccess | kEmergencyAccess | kTaxiAccess | kHOVAccess),
                                   kAllAccess & ~(kPedestrianAccess | kBicycleAccess | kEmergencyAccess | kTaxiAccess | kHOVAccess),
                                   -1, -1, -1, -1, (kPedestrianAccess | kBicycleAccess), -1}},
    {"France",                    {kAllAccess & ~(kPedestrianAccess | kBicycleAccess | kEmergencyAccess | kTaxiAccess | kHOVAccess),
                                   kAllAccess & ~(kPedestrianAccess | kBicycleAccess | kEmergencyAccess | kTaxiAccess | kHOVAccess), -1, -1,
                                   (kPedestrianAccess | kBicycleAccess), -1, -1, -1}},
    {"Finland",                   {-1, -1, -1, -1, (kPedestrianAccess | kBicycleAccess),
                                   -1, (kPedestrianAccess | kBicycleAccess), -1}},
    {"Hungary",                   {kAllAccess & ~(kPedestrianAccess | kBicycleAccess | kEmergencyAccess | kTaxiAccess | kHOVAccess),
                                   kAllAccess & ~(kPedestrianAccess | kBicycleAccess | kEmergencyAccess | kTaxiAccess | kHOVAccess),
                                   -1, -1, -1, -1, -1, -1}},
    {"The Netherlands",           {kAllAccess & ~(kPedestrianAccess | kBicycleAccess | kEmergencyAccess | kTaxiAccess | kHOVAccess),
                                   kAllAccess & ~(kPedestrianAccess | kBicycleAccess | kEmergencyAccess | kTaxiAccess | kHOVAccess),
                                   -1, -1, -1, -1, (kPedestrianAccess | kBicycleAccess), -1}},
    {"Norway",                    {-1,-1, -1,
                                   (kPedestrianAccess | kBicycleAccess), (kPedestrianAccess | kBicycleAccess),
                                   -1, (kPedestrianAccess | kBicycleAccess), -1}},
    {"Poland",                    {kAllAccess & ~(kPedestrianAccess | kBicycleAccess | kEmergencyAccess | kTaxiAccess | kHOVAccess),
                                   kAllAccess & ~(kPedestrianAccess | kBicycleAccess | kEmergencyAccess | kTaxiAccess | kHOVAccess),
                                   -1, -1, -1, -1, -1, -1}},
    {"Romania",                   {kAllAccess & ~(kPedestrianAccess | kBicycleAccess | kEmergencyAccess | kTaxiAccess | kHOVAccess),
                                   kAllAccess & ~(kPedestrianAccess | kBicycleAccess | kEmergencyAccess | kTaxiAccess | kHOVAccess),
                                   -1, -1, -1, -1, -1, -1}},
    {"Slovakia",                  {kAllAccess & ~(kPedestrianAccess | kBicycleAccess | kEmergencyAccess | kTaxiAccess | kHOVAccess),
                                   kAllAccess & ~(kPedestrianAccess | kBicycleAccess | kEmergencyAccess | kTaxiAccess | kHOVAccess),
                                   -1, -1, -1, -1, -1, -1}},
    {"Switzerland",               {kAllAccess & ~(kPedestrianAccess | kBicycleAccess | kEmergencyAccess | kTaxiAccess | kHOVAccess),
                                   kAllAccess & ~(kPedestrianAccess | kBicycleAccess | kEmergencyAccess | kTaxiAccess | kHOVAccess),
                                   -1, -1, -1, -1, -1, -1}},
    {"Turkey",                    {-1, -1, -1, -1, -1, kPedestrianAccess, -1, -1}},
    {"United Kingdom",            {-1, -1, -1, -1, -1, (kPedestrianAccess | kBicycleAccess),
                                   (kPedestrianAccess | kBicycleAccess), -1}},
    {"United States of America",  {-1, -1, -1, -1,
                                   (kPedestrianAccess | kBicycleAccess),
                                   (kPedestrianAccess | kBicycleAccess),
                                   (kPedestrianAccess | kBicycleAccess), -1}}
  };


}
}
#endif  // VALHALLA_MJOLNIR_ADMINCONSTANTS_H_
