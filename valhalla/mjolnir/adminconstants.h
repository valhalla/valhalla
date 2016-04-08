#ifndef VALHALLA_MJOLNIR_ADMINCONSTANTS_H_
#define VALHALLA_MJOLNIR_ADMINCONSTANTS_H_

#include <unordered_map>
#include <string>

#include <valhalla/baldr/graphconstants.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

enum class Columns : uint8_t {
  kAdminId = 1,
  kTrunk = 2,
  kTrunkLink = 3,
  kTrack = 4,
  kFootway = 5,
  kPedestrian = 6,
  kBridleway = 7,
  kCycleway = 8,
  kPath = 9
};

const std::unordered_map<uint8_t, std::string> ColumnStrings = {
  {static_cast<uint8_t>(Columns::kAdminId),"admin_id"},
  {static_cast<uint8_t>(Columns::kTrunk),"trunk"},
  {static_cast<uint8_t>(Columns::kTrunkLink),"trunk_link"},
  {static_cast<uint8_t>(Columns::kTrack),"track"},
  {static_cast<uint8_t>(Columns::kFootway),"footway"},
  {static_cast<uint8_t>(Columns::kPedestrian),"pedestrian"},
  {static_cast<uint8_t>(Columns::kBridleway),"bridleway"},
  {static_cast<uint8_t>(Columns::kCycleway),"cycleway"},
  {static_cast<uint8_t>(Columns::kPath),"path"},
};
inline std::string to_string(Columns col) {
  auto i = ColumnStrings.find(static_cast<uint8_t>(col));
  if(i == ColumnStrings.cend())
    return "null";
  return i->second;
}

//Based on logic at http://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Access-Restrictions
//vector = trunk, trunk_link, track, footway, pedestrian, bridleway, cycleway, and path
//-1 indicates a null value will be set which means no change from default access.
//0 indicates no access.
const std::unordered_map<std::string, std::vector<int>> kCountryAccess {
    {"Australia",                 {-1, -1, -1, (kPedestrianAccess | kBicycleAccess),
                                   (kPedestrianAccess | kBicycleAccess),
                                   (kPedestrianAccess | kBicycleAccess),
                                   (kPedestrianAccess | kBicycleAccess), -1}},
    {"Austria",                   {kAllAccess & ~(kPedestrianAccess | kBicycleAccess),
                                   kAllAccess & ~(kPedestrianAccess | kBicycleAccess),
                                   kPedestrianAccess, -1, -1, -1, -1,  kPedestrianAccess}},
    {"Belarus",                   {-1, -1, -1, (kPedestrianAccess | kBicycleAccess),
                                   (kPedestrianAccess | kBicycleAccess), -1,
                                   (kPedestrianAccess | kBicycleAccess), -1}},
    {"Belgium",                   {kAllAccess & ~(kPedestrianAccess | kBicycleAccess),
                                   kAllAccess & ~(kPedestrianAccess | kBicycleAccess),
                                   -1, -1, (kPedestrianAccess | kBicycleAccess),
                                   kPedestrianAccess, (kPedestrianAccess | kBicycleAccess), -1}},
    {"Brazil",                    {-1, -1, -1, (kPedestrianAccess | kBicycleAccess),
                                    -1, (kPedestrianAccess | kBicycleAccess),
                                    (kPedestrianAccess | kBicycleAccess), -1}},
    {"Denmark",                   {kAllAccess & ~(kPedestrianAccess | kBicycleAccess),
                                   kAllAccess & ~(kPedestrianAccess | kBicycleAccess),
                                   kAllAccess & ~(kPedestrianAccess | kBicycleAccess),
                                   -1, -1, -1, kAllAccess & ~(kPedestrianAccess | kBicycleAccess), -1}},
    {"France",                    {kAllAccess & ~(kPedestrianAccess | kBicycleAccess),
                                   kAllAccess & ~(kPedestrianAccess | kBicycleAccess), -1, -1,
                                   (kPedestrianAccess | kBicycleAccess), -1, -1, -1}},
    {"Finland",                   {-1, -1, -1, -1, (kPedestrianAccess | kBicycleAccess),
                                   -1, (kPedestrianAccess | kBicycleAccess), -1}},
    {"The Netherlands",           {kAllAccess & ~(kPedestrianAccess | kBicycleAccess),
                                   kAllAccess & ~(kPedestrianAccess | kBicycleAccess),
                                   -1, -1, -1, -1, (kPedestrianAccess | kBicycleAccess), -1}},
    {"Romania",                   {kAllAccess & ~(kPedestrianAccess | kBicycleAccess),
                                   kAllAccess & ~(kPedestrianAccess | kBicycleAccess),
                                   -1, -1, -1, -1, -1, -1}},
    {"Slovakia",                  {kAllAccess & ~(kPedestrianAccess | kBicycleAccess),
                                   kAllAccess & ~(kPedestrianAccess | kBicycleAccess), -1, -1, -1, -1, -1, -1}},
    {"Switzerland",               {kAllAccess & ~(kPedestrianAccess | kBicycleAccess),
                                   kAllAccess & ~(kPedestrianAccess | kBicycleAccess), -1, -1, -1, -1, -1, -1}},
    {"United Kingdom",            {-1, -1, -1, -1, -1, kPedestrianAccess, -1, -1}},
    {"United States of America",  {-1, -1, -1, kAllAccess & ~(kPedestrianAccess | kBicycleAccess),
                                   kAllAccess & ~(kPedestrianAccess | kBicycleAccess),
                                   kAllAccess & ~(kPedestrianAccess | kBicycleAccess),
                                   kAllAccess & ~(kPedestrianAccess | kBicycleAccess), -1}}
  };


}
}
#endif  // VALHALLA_MJOLNIR_ADMINCONSTANTS_H_
