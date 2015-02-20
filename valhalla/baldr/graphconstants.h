#ifndef VALHALLA_BALDR_GRAPHCONSTANTS_H_
#define VALHALLA_BALDR_GRAPHCONSTANTS_H_

#include <unordered_map>
#include <string>

namespace valhalla {
namespace baldr {

// Access constants. Bit constants.
constexpr uint8_t kAutoAccess       = 1;
constexpr uint8_t kPedestrianAccess = 2;
constexpr uint8_t kBicycleAccess    = 4;
constexpr uint8_t kTruckAccess      = 8;
constexpr uint8_t kEmergencyAccess  = 16;
constexpr uint8_t kTaxiAccess       = 32;
constexpr uint8_t kHorseAccess      = 64;  // ??

// Access structure used by NodeInfo and DirectedEdge
// TODO - should HOV and/or transponder only access be part of this?!
union Access {
  struct Fields {
    uint8_t car          : 1;
    uint8_t pedestrian   : 1;
    uint8_t bicycle      : 1;
    uint8_t truck        : 1;
    uint8_t emergency    : 1;
    uint8_t taxi         : 1;
    uint8_t horse        : 1;  // ???
    uint8_t spare        : 1;
  } fields;
  uint8_t v;
};

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

// Node types.
enum class NodeType : uint8_t {
  kStreetIntersection = 0,
  kGate = 1,
  kBollard = 2,
  kRailStop = 3,
  kBusStop = 4,
  kMultiUseTransitStop = 5,
  kBikeShare = 6,
  kParking = 7
};

// Intersection types. Classifications of various intersections.
// TODO - enumerate and assign!
enum class IntersectionType : uint8_t {
  kFalse = 0       // False intersection. Only 2 edges connect. Typically
                   // where 2 ways are split or where attributes force a split.
};

// Edge use
// TODO - add values for each so we are explicit
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

// TODO - add comment and explicit values
enum class CycleLane : uint8_t {
  kNone,
  kShared,
  kDedicated,
  kSeparated
};

// TODO - add comment and explicit values
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

// TODO - add comment and explicit values
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

// TODO - add comment and explicit values
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
