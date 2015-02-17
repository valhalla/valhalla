#ifndef VALHALLA_BALDR_GRAPHCONSTANTS_H_
#define VALHALLA_BALDR_GRAPHCONSTANTS_H_

#include <unordered_map>
#include <string>

namespace valhalla {
namespace baldr {

// Road class or importance of an edge
enum class RoadClass : uint8_t {
  kMotorway = 0,
  kTrunk = 1,
  kPrimary = 2,
  kSecondary = 3,
  kTertiaryUnclassified = 4,
  kResidential = 5,
  kService = 6,
  kOther = 7
};
const std::unordered_map<std::string, RoadClass> stringToRoadClass =
  { {"Motorway", RoadClass::kMotorway}, {"Trunk", RoadClass::kTrunk}, {"Primary", RoadClass::kPrimary},
    {"Secondary", RoadClass::kSecondary}, {"TertiaryUnclassified", RoadClass::kTertiaryUnclassified},
    {"Residential", RoadClass::kResidential}, {"Service", RoadClass::kService}, {"Other", RoadClass::kOther}
  };

// Maximum length in meters of an internal intersection edge
constexpr float kMaxInternalLength = 60.0f;

// Maximum length in meters of a "link" that can be assigned use=kTurnChannel
// (vs. kRamp)
constexpr float kMaxTurnChannelLength = 150.0f;

// Edge use
enum class Use : uint8_t {
  kRoad,
  kCycleway,
  kFootway,
  kParkingAisle,
  kDriveway,
  kAlley,
  kEmergencyAccess,
  kDriveThru,
  kSteps,
  kRamp,
  kTurnChannel,
  kTrack,
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

enum class DOW : uint8_t {
  kNone,
  kSunday,
  kMonday,
  kTuesday,
  kWednesday,
  kThursday,
  kFriday,
  kSaturday,
};


enum class RestrictionType : uint8_t {
  kNoLeftTurn,
  kNoRightTurn,
  kNoStraightOn,
  kNoUTurn,
  kOnlyRightTurn,
  kOnlyLeftTurn,
  kOnlyStraightOn
};

}
}

#endif  // VALHALLA_BALDR_GRAPHCONSTANTS_H_
