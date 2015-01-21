#ifndef VALHALLA_BALDR_GRAPHCONSTANTS_H_
#define VALHALLA_BALDR_GRAPHCONSTANTS_H_

#include <unordered_map>
#include <string>

namespace valhalla {
namespace baldr {


enum class RoadClass : uint8_t {
  kMotorway = 0,
  kTrunk = 1,
  kPrimary = 2,
  kTertiaryUnclassified = 3,
  kResidential = 4,
  kService = 5,
  kTrack = 6,
  kOther = 7
};
const std::unordered_map<std::string, RoadClass> stringToRoadClass =
  { {"Motorway", RoadClass::kMotorway}, {"Trunk", RoadClass::kTrunk}, {"Primary", RoadClass::kPrimary},
    {"TertiaryUnclassified", RoadClass::kTertiaryUnclassified}, {"Residential", RoadClass::kResidential},
    {"Service", RoadClass::kService}, {"Track", RoadClass::kTrack}, {"Other", RoadClass::kOther}
  };

enum class Use : uint8_t {
  kNone,
  kCycleway,
  kFootway,
  kParkingAisle,
  kDriveway,
  kAlley,
  kEmergencyAccess,
  kDriveThru,
  kSteps,
  kOther
};

enum class CycleLane : uint8_t {
  kNone,
  kShared,
  kDedicated,
  kSeparated
};

enum class Surface : uint8_t {
  kPavedSmooth,
  kPaved,
  kPavedRough,
  kCompacted,
  kDirt,
  kGravel,
  kPath,
  kImpassable
};
}
}

#endif  // VALHALLA_BALDR_GRAPHCONSTANTS_H_
