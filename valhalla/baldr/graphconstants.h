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

// Edge driveability (auto)
enum class Driveability {
  kNone = 0,        // Edge is not driveable in either direction
  kForward = 1,     // Edge is driveable in the forward direction
  kBackward = 2,    // Edge is driveable in the backward direction
  kBoth = 3         // Edge is driveable in both directions
};

// Road class or importance of an edge
enum class RoadClass : uint8_t {
  kMotorway = 0,
  kTrunk = 1,
  kPrimary = 2,
  kSecondary = 3,
  kTertiary = 4,
  kUnclassified = 5,
  kResidential = 6,
  kServiceOther = 7
};
const std::unordered_map<std::string, RoadClass> stringToRoadClass =
  { {"Motorway", RoadClass::kMotorway}, {"Trunk", RoadClass::kTrunk}, {"Primary", RoadClass::kPrimary},
    {"Secondary", RoadClass::kSecondary}, {"Tertiary", RoadClass::kTertiary},
    {"Unclassified", RoadClass::kUnclassified},{"Residential", RoadClass::kResidential},
    {"ServiceOther", RoadClass::kServiceOther}
  };

// Maximum length in meters of an internal intersection edge
constexpr float kMaxInternalLength = 60.0f;

// Maximum length in meters of a "link" that can be assigned use=kTurnChannel
// (vs. kRamp)
constexpr float kMaxTurnChannelLength = 200.0f;

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
  kRegular = 0,    // Regular, unclassified intersection
  kFalse = 1,      // False intersection. Only 2 edges connect. Typically
                   // where 2 ways are split or where attributes force a split.
  kDeadEnd = 2     // Node only connects to one edge ("dead-end").
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
  kDriveThru = 8,         // Commercial drive-thru (banks/fast-food)
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

// Restriction types. If a restriction exists this value will be set.
// Restrictions with "Only" will restrict all turns not adhering to the
// only "to" road of the restriction.
enum class RestrictionType : uint8_t {
  kNoLeftTurn = 0,
  kNoRightTurn = 1,
  kNoStraightOn = 2,
  kNoUTurn = 3,
  kOnlyRightTurn = 4,
  kOnlyLeftTurn = 5,
  kOnlyStraightOn = 6
};

// ------------------------------- Transit information --------------------- //

constexpr uint32_t kOneStopIdSize = 256;

// Transit transfer types
enum class TransferType : uint8_t {
  kRecommended = 0,   // Recommended transfer point between 2 routes
  kTimed       = 1,   // Timed transfer between 2 routes. Departing vehicle
                      // is expected to wait, allowing sufficient time for
                      // passengers to transfer.
  kMinTime     = 2,   // Transfer is expected to take the time specified.
  kNotPossible = 3    // Transfers not possible between routes
};

enum class CalendarExceptionType : uint8_t {
  kAdded       = 1,   // Service added for the specified date
  kRemoved     = 2    // Service removed for the specified date
};

}
}

#endif  // VALHALLA_BALDR_GRAPHCONSTANTS_H_
