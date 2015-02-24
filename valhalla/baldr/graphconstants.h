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
constexpr uint8_t kHorseAccess      = 64;
constexpr uint8_t kHOVAccess        = 128;
constexpr uint8_t kAllAccess        = 255;

// Payment constants. Bit constants.
constexpr uint8_t kCoins  = 1; // Coins
constexpr uint8_t kNotes  = 2; // Bills
constexpr uint8_t kETC    = 4; // Electronic Toll Collector

// Access structure used by NodeInfo and DirectedEdge
union Access {
  struct Fields {
    uint8_t car          : 1; // Auto and light vehicle access
    uint8_t pedestrian   : 1; // Pedestrian access
    uint8_t bicycle      : 1; // Bicycle access
    uint8_t truck        : 1; // Truck / heavy good vehicle access
    uint8_t emergency    : 1; // Emergency vehicle access
    uint8_t taxi         : 1; // Taxi access
    uint8_t horse        : 1; // Horse access
    uint8_t hov          : 1; // High occupancy vehicle access
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
  kStreetIntersection = 0,  // Regular intersection of 2 roads
  kGate = 1,                // Gate or rising bollard
  kBollard = 2,             // Bollard (fixed obstruction)
  kTollBooth = 3,           // Toll booth / fare collection
  kRailStop = 4,            // Rail/metro/subway stop
  kBusStop = 5,             // Bus stop
  kMultiUseTransitStop = 6, // Multi-use transit stop (rail and bus)
  kBikeShare = 7,           // Bike share location
  kParking = 8              // Parking location
};

// Intersection types. Classifications of various intersections.
// TODO - enumerate and assign!
enum class IntersectionType : uint8_t {
  kFalse = 0       // False intersection. Only 2 edges connect. Typically
                   // where 2 ways are split or where attributes force a split.
};

// Edge use. Indicates specialized uses.
enum class Use : uint8_t {
  // Road specific uses
  kRoad = 0,
  kRamp = 1,              // Link - exits/entrance ramps.
  kTurnChannel = 2,       // Link - turn lane.
  kTrack = 3,             // Agricultural use, forest tracks
  kDriveway = 4,          // Driveway/private service
  kAlley = 5,             // Service road - limited route use
  kParkingAisle = 6,      // Access roads in parking areas
  kEmergencyAccess = 7,   // Emergency vehicles only
  kDriveThru = 8,         // Commerical drive-thru (banks/fast-food)
  kCuldesac = 9,          // Cul-de-sac (edge that forms a loop and is only
                          // connected at one node to another edge.

  // Pedestrian specific uses
  kFootway = 25,
  kSteps   = 26,           // Stairs

  // Bicycle specific uses
  kCycleway = 20,          // Dedicated bicycle path
  kMountainBike = 21,      // Mountain bike trail

  // Transit specific uses
  kRail = 30,              // Rail line
  kBus = 31,               // Bus line
  kRailConnection = 32,    // Connection to a rail stop
  kBusConnection = 33,     // Connection to a bus stop
  kTransitConnection = 34, // Connection to multi-use transit stop

  // Other...
  kOther = 63
};

// Speed type
enum class SpeedType : uint8_t {
  kTagged = 0,            // Tagged maximum speed
  kClassified = 1,        // Speed assigned based on highway classification
  kClassifiedUrban = 2,   // Classified speed in urban area
  kClassifiedRural = 3    // Classified speed in rural area
};

// Indication of the type of cycle lane (if any) present along an edge.
// Higher values are more favorable to safe bicycling.
enum class CycleLane : uint8_t {
  kNone = 0,      // No specified bicycle lane
  kShared = 1,    // Shared use lane (could be shared with pedestrians)
  kDedicated = 2, // Dedicated cycle lane
  kSeparated = 3  // A separate cycle lane (physical separation from the
                  // main carriageway)
};

// Generalized representation of surface types. Lower values indicate smoother
// surfaces. Vehicle or bicycle type can use this to avoid or disallow edges
// that are "too rough" or inappropriate for the vehicle to travel on.
enum class Surface : uint8_t {
  kPavedSmooth = 0,
  kPaved = 1,
  kPavedRough = 2,
  kCompacted = 3,
  kDirt = 4,
  kGravel = 5,
  kPath = 6,
  kImpassable = 7
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
