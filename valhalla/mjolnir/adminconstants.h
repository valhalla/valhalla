#ifndef VALHALLA_MJOLNIR_ADMINCONSTANTS_H_
#define VALHALLA_MJOLNIR_ADMINCONSTANTS_H_

#include <valhalla/baldr/graphconstants.h>

#include <string>
#include <unordered_map>
#include <vector>

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
                    {-1, -1, -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     -1, -1, -1, -1, -1}},
                   {"Austria",
                    {(baldr::kAutoAccess | baldr::kTruckAccess | baldr::kBusAccess |
                      baldr::kHOVAccess | baldr::kTaxiAccess | baldr::kMotorcycleAccess),
                     (baldr::kAutoAccess | baldr::kTruckAccess | baldr::kBusAccess |
                      baldr::kHOVAccess | baldr::kTaxiAccess | baldr::kMotorcycleAccess),
                     -1, -1, -1, -1, -1, (baldr::kPedestrianAccess | baldr::kWheelchairAccess), -1}},
                   {"Belarus",
                    {-1, -1, -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     -1, -1}},
                   {"Belgium",
                    {(baldr::kAutoAccess | baldr::kTruckAccess | baldr::kBusAccess |
                      baldr::kHOVAccess | baldr::kTaxiAccess | baldr::kMotorcycleAccess),
                     (baldr::kAutoAccess | baldr::kTruckAccess | baldr::kBusAccess |
                      baldr::kHOVAccess | baldr::kTaxiAccess | baldr::kMotorcycleAccess),
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess |
                      baldr::kMopedAccess),
                     -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess),
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess |
                      baldr::kMopedAccess),
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess |
                      baldr::kMopedAccess),
                     -1}},
                   {"Brazil",
                    {-1, -1, -1, -1, -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     -1, -1, -1}},
                   {"China",
                    {-1, -1, -1, -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     -1, -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess |
                      baldr::kMopedAccess),
                     -1}},
                   {"Denmark",
                    {(baldr::kAutoAccess | baldr::kTruckAccess | baldr::kBusAccess |
                      baldr::kHOVAccess | baldr::kTaxiAccess | baldr::kMotorcycleAccess),
                     (baldr::kAutoAccess | baldr::kTruckAccess | baldr::kBusAccess |
                      baldr::kHOVAccess | baldr::kTaxiAccess | baldr::kMotorcycleAccess),
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     -1, -1, -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     -1, -1}},
                   {"England",
                    {-1, -1, -1, -1, -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     -1, -1}},
                   {"France",
                    {(baldr::kAutoAccess | baldr::kTruckAccess | baldr::kBusAccess |
                      baldr::kHOVAccess | baldr::kTaxiAccess | baldr::kMotorcycleAccess),
                     (baldr::kAutoAccess | baldr::kTruckAccess | baldr::kBusAccess |
                      baldr::kHOVAccess | baldr::kTaxiAccess | baldr::kMotorcycleAccess),
                     -1, -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     -1, -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess |
                      baldr::kMopedAccess),
                     -1}},
                   {"Finland",
                    {-1, -1, -1, -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     -1, -1}},
                   // Germany - no overrides
                   {"Greece",
                    {-1, -1, -1, -1, -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     -1, -1}},
                   {"Hungary",
                    {(baldr::kAutoAccess | baldr::kTruckAccess | baldr::kBusAccess |
                      baldr::kHOVAccess | baldr::kTaxiAccess | baldr::kMotorcycleAccess),
                     (baldr::kAutoAccess | baldr::kTruckAccess | baldr::kBusAccess |
                      baldr::kHOVAccess | baldr::kTaxiAccess | baldr::kMotorcycleAccess),
                     -1, -1, -1, -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     -1, -1}},
                   {"Iceland",
                    {-1, -1, -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     -1, -1}},
                   {"Ireland",
                    {-1, -1, -1, -1, -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     -1, -1, -1}},
                   {"Italy",
                    {-1, -1, -1, -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     -1, -1, -1, -1}},
                   // Netherlands allows bicycle access on trunk and trunk link without motorroad =
                   // yes
                   {"Netherlands",
                    {-1, -1, -1, -1, -1, -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess |
                      baldr::kMopedAccess),
                     -1}},
                   {"Norway",
                    {-1, -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess |
                      baldr::kMopedAccess),
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     -1, -1}},
                   {"Northern Ireland",
                    {-1, -1, -1, -1, -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     -1, -1, -1}},
                   {"Oman",
                    {-1, -1, -1, -1, -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     -1, -1}},
                   {"Philippines",
                    {-1, -1, -1, -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess |
                      baldr::kMopedAccess),
                     -1}},
                   {"Poland",
                    {-1, -1, -1, -1, -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess |
                      baldr::kMopedAccess),
                     -1}},
                   {"Romania",
                    {-1, -1, -1, -1, -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess |
                      baldr::kMopedAccess),
                     (baldr::kBicycleAccess | baldr::kMopedAccess),
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess |
                      baldr::kMopedAccess),
                     -1}},
                   {"Russia",
                    {-1, -1, -1, -1, -1, -1, (baldr::kMopedAccess | baldr::kBicycleAccess), -1, -1}},
                   {"Alba / Scotland",
                    {-1, -1, -1, -1, -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     -1, -1}},
                   {"Slovakia",
                    {(baldr::kAutoAccess | baldr::kTruckAccess | baldr::kBusAccess |
                      baldr::kHOVAccess | baldr::kTaxiAccess | baldr::kMotorcycleAccess),
                     (baldr::kAutoAccess | baldr::kTruckAccess | baldr::kBusAccess |
                      baldr::kHOVAccess | baldr::kTaxiAccess | baldr::kMotorcycleAccess),
                     -1, -1, -1, -1, -1, -1, -1}},
                   // Spain allows bicycle access on trunk and trunk link without motorroad = yes
                   {"Spain",
                    {-1, -1, -1, -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     -1, -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess |
                      baldr::kMopedAccess),
                     -1}},
                   {"Sweden",
                    {-1, -1, -1, -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess),
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess |
                      baldr::kMopedAccess),
                     -1, -1}},
                   {"Switzerland",
                    {(baldr::kAutoAccess | baldr::kTruckAccess | baldr::kBusAccess |
                      baldr::kHOVAccess | baldr::kTaxiAccess | baldr::kMotorcycleAccess),
                     (baldr::kAutoAccess | baldr::kTruckAccess | baldr::kBusAccess |
                      baldr::kHOVAccess | baldr::kTaxiAccess | baldr::kMotorcycleAccess),
                     -1, -1, -1, -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess |
                      baldr::kMopedAccess),
                     -1, -1}},
                   {"ประเทศไทย",
                    {-1, -1, -1, -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess |
                      baldr::kMopedAccess),
                     -1}},
                   {"Türkiye",
                    {-1, -1, -1, -1, -1, (baldr::kPedestrianAccess | baldr::kWheelchairAccess),
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     -1, -1}},
                   // Ukraine - no overrides
                   {"United States",
                    {-1, -1, -1, -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess |
                      baldr::kMopedAccess),
                     -1}},
                   {"Cymru / Wales",
                    {-1, -1, -1, -1, -1,
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     (baldr::kPedestrianAccess | baldr::kWheelchairAccess | baldr::kBicycleAccess),
                     -1, -1}}};

const std::unordered_map<std::string, std::pair<uint8_t, std::string>>
    kSupportedLanguages{{"Cymru / Wales", {2, "cy;en"}},
                        {"United Kingdom", {2, "en"}},
                        {"Ireland", {4, "ga"}},
                        {"Northern Ireland", {4, "ga"}},
                        {"Japan", {2, "ja;en"}},
                        {"Canada", {2, "en;fr"}},
                        {"Belarus", {2, "ru;be"}},
                        {"Singapore", {2, "en;zh;ms;ta"}},
                        {"Saudi Arabia", {2, "ar;en"}},
                        {"Catalonia", {4, "ca"}},
                        {"Valencian Community", {4, "ca"}},
                        {"Spain", {2, "es"}}};

} // namespace mjolnir
} // namespace valhalla
#endif // VALHALLA_MJOLNIR_ADMINCONSTANTS_H_
