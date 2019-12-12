#ifndef VALHALLA_BALDR_GRAPHCONSTANTS_H_
#define VALHALLA_BALDR_GRAPHCONSTANTS_H_

#include <string>
#include <unordered_map>

namespace valhalla {
namespace baldr {

// OSM Ids can exceed 32 bits, but these are currently only Node Ids. Way Ids should still have
// room to grow before exceeding an unsigned 32 bit word.
constexpr uint32_t kMaxOSMWayId = 4294967295;

// Maximum tile id/index supported. 22 bits
constexpr uint32_t kMaxGraphTileId = 4194303;
// Maximum id/index within a tile. 21 bits
constexpr uint32_t kMaxGraphId = 2097151;

// Access bit field constants. Access in directed edge allows 12 bits.
constexpr uint16_t kAutoAccess = 1;
constexpr uint16_t kPedestrianAccess = 2;
constexpr uint16_t kBicycleAccess = 4;
constexpr uint16_t kTruckAccess = 8;
constexpr uint16_t kEmergencyAccess = 16;
constexpr uint16_t kTaxiAccess = 32;
constexpr uint16_t kBusAccess = 64;
constexpr uint16_t kHOVAccess = 128;
constexpr uint16_t kWheelchairAccess = 256;
constexpr uint16_t kMopedAccess = 512;
constexpr uint16_t kMotorcycleAccess = 1024;
constexpr uint16_t kSpareAccess = 2048; // Unused so far
constexpr uint16_t kAllAccess = 4095;

// Constant representing vehicular access types
constexpr uint32_t kVehicularAccess = kAutoAccess | kTruckAccess | kMopedAccess | kMotorcycleAccess |
                                      kTaxiAccess | kBusAccess | kHOVAccess;

// Maximum number of transit records per tile and other max. transit
// field values.
constexpr uint32_t kMaxTransitDepartures = 16777215;
constexpr uint32_t kMaxTransitStops = 65535;
constexpr uint32_t kMaxTransitRoutes = 4095;
constexpr uint32_t kMaxTransitSchedules = 4095;
constexpr uint32_t kMaxTransitBlockId = 1048575;
constexpr uint32_t kMaxTransitLineId = 1048575;
constexpr uint32_t kMaxTransitDepartureTime = 131071;
constexpr uint32_t kMaxTransitElapsedTime = 131071;
constexpr uint32_t kMaxStartTime = 131071;
constexpr uint32_t kMaxEndTime = 131071;
constexpr uint32_t kMaxEndDay = 63;
constexpr uint32_t kScheduleEndDay = 60;
constexpr uint32_t kMaxFrequency = 8191;
constexpr uint32_t kMaxTransfers = 65535;
constexpr uint32_t kMaxTransferTime = 65535;
constexpr uint32_t kMaxTripId = 536870912; // 29 bits

// Maximum offset into the text/name list
constexpr uint32_t kMaxNameOffset = 16777215; // 24 bits

// Payment constants. Bit constants.
constexpr uint8_t kCoins = 1; // Coins
constexpr uint8_t kNotes = 2; // Bills
constexpr uint8_t kETC = 4;   // Electronic Toll Collector

// Edge traversability
enum class Traversability {
  kNone = 0,     // Edge is not traversable in either direction
  kForward = 1,  // Edge is traversable in the forward direction
  kBackward = 2, // Edge is traversable in the backward direction
  kBoth = 3      // Edge is traversable in both directions
};

// Maximum relative density at a node or within a tile
constexpr uint32_t kMaxDensity = 15;

// Maximum speed. This impacts the effectiveness of A* for driving routes
// so it should be set as low as is reasonable. Speeds above this in OSM are
// clamped to this maximum value.
constexpr uint32_t kMaxSpeedKph = 140; // ~85 MPH

// Minimum speed. This is a stop gap for dubious traffic data. While its possible
// to measure a probe going this slow via stop and go traffic over a long enough
// stretch, its unlikely to be good signal below this value
constexpr uint32_t kMinSpeedKph = 5; // ~3 MPH

inline bool valid_speed(float speed) {
  return speed > kMinSpeedKph && speed < kMaxSpeedKph;
}

// Maximum ferry speed
constexpr uint32_t kMaxFerrySpeedKph = 40; // 21 knots

// Special speeds for use with parking aisles, driveways, and drive thrus
constexpr uint32_t kParkingAisleSpeed = 15; // 15 KPH (10MPH)
constexpr uint32_t kDriveThruSpeed = 10;    // 10 KPH
constexpr uint32_t kDrivewaySpeed = 10;     // 10 KPH

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
inline RoadClass stringToRoadClass(const std::string& s) {
  static const std::unordered_map<std::string, RoadClass> stringToRoadClass =
      {{"Motorway", RoadClass::kMotorway},       {"Trunk", RoadClass::kTrunk},
       {"Primary", RoadClass::kPrimary},         {"Secondary", RoadClass::kSecondary},
       {"Tertiary", RoadClass::kTertiary},       {"Unclassified", RoadClass::kUnclassified},
       {"Residential", RoadClass::kResidential}, {"ServiceOther", RoadClass::kServiceOther}};

  return stringToRoadClass.find(s)->second;
}
inline std::string to_string(RoadClass r) {
  static const std::unordered_map<uint8_t, std::string> RoadClassStrings = {
      {static_cast<uint8_t>(RoadClass::kMotorway), "motorway"},
      {static_cast<uint8_t>(RoadClass::kTrunk), "trunk"},
      {static_cast<uint8_t>(RoadClass::kPrimary), "primary"},
      {static_cast<uint8_t>(RoadClass::kSecondary), "secondary"},
      {static_cast<uint8_t>(RoadClass::kTertiary), "tertiary"},
      {static_cast<uint8_t>(RoadClass::kUnclassified), "unclassified"},
      {static_cast<uint8_t>(RoadClass::kResidential), "residential"},
      {static_cast<uint8_t>(RoadClass::kServiceOther), "service_other"},
  };

  auto i = RoadClassStrings.find(static_cast<uint8_t>(r));
  if (i == RoadClassStrings.cend()) {
    return "null";
  }
  return i->second;
}

// Maximum length in meters of an internal intersection edge
constexpr float kMaxInternalLength = 32.0f;

// Maximum length in meters of a "link" that can be assigned use=kTurnChannel
// (vs. kRamp)
constexpr float kMaxTurnChannelLength = 200.0f;

// Bicycle Network constants. Bit constants.
constexpr uint8_t kNcn = 1; // Part of national bicycle network
constexpr uint8_t kRcn = 2; // Part of regional bicycle network
constexpr uint8_t kLcn = 4; // Part of local bicycle network
constexpr uint8_t kMcn = 8; // Part of mountain bicycle network
constexpr uint8_t kMaxBicycleNetwork = 15;

// Maximum offset to edge information
constexpr uint32_t kMaxEdgeInfoOffset = 33554431; // 2^25 bytes

// Maximum length of an edge
constexpr uint32_t kMaxEdgeLength = 16777215; // 2^24 meters

// Maximum number of edges allowed in a turn restriction mask
constexpr uint32_t kMaxTurnRestrictionEdges = 8;

// Maximum lane count
constexpr uint32_t kMaxLaneCount = 15;

// Number of edges considered for edge transitions
constexpr uint32_t kNumberOfEdgeTransitions = 8;

// Maximum shortcuts edges from a node. More than this can be
// added but this is the max. that can supersede an edge
constexpr uint32_t kMaxShortcutsFromNode = 7;

// Maximum stop impact
constexpr uint32_t kMaxStopImpact = 7;

// Maximum grade and curvature factors.
constexpr uint32_t kMaxGradeFactor = 15;
constexpr uint32_t kMaxCurvatureFactor = 15;

// Maximum added time along shortcuts to approximate transition costs
constexpr uint32_t kMaxAddedTime = 255;

// Elevation constants
constexpr float kNoElevationData = 32768.0f;

// Node types.
enum class NodeType : uint8_t {
  kStreetIntersection = 0, // Regular intersection of 2 roads
  kGate = 1,               // Gate or rising bollard
  kBollard = 2,            // Bollard (fixed obstruction)
  kTollBooth = 3,          // Toll booth / fare collection
  // TODO - for now there is no differentiation between bus and rail stops...
  kTransitEgress = 4,           // Transit egress
  kTransitStation = 5,          // Transit station
  kMultiUseTransitPlatform = 6, // Multi-use transit platform (rail and bus)
  kBikeShare = 7,               // Bike share location
  kParking = 8,                 // Parking location
  kMotorWayJunction = 9,        // Highway = motorway_junction
  kBorderControl = 10           // Border control
};
inline std::string to_string(NodeType n) {
  static const std::unordered_map<uint8_t, std::string> NodeTypeStrings = {
      {static_cast<uint8_t>(NodeType::kStreetIntersection), "street_intersection"},
      {static_cast<uint8_t>(NodeType::kGate), "gate"},
      {static_cast<uint8_t>(NodeType::kBollard), "bollard"},
      {static_cast<uint8_t>(NodeType::kTollBooth), "toll_booth"},
      {static_cast<uint8_t>(NodeType::kTransitEgress), "transit_egress"},
      {static_cast<uint8_t>(NodeType::kTransitStation), "transit_station"},
      {static_cast<uint8_t>(NodeType::kMultiUseTransitPlatform), "multi_use_transit_platform"},
      {static_cast<uint8_t>(NodeType::kBikeShare), "bike_share"},
      {static_cast<uint8_t>(NodeType::kParking), "parking"},
      {static_cast<uint8_t>(NodeType::kMotorWayJunction), "motor_way_junction"},
      {static_cast<uint8_t>(NodeType::kBorderControl), "border_control"},
  };

  auto i = NodeTypeStrings.find(static_cast<uint8_t>(n));
  if (i == NodeTypeStrings.cend()) {
    return "null";
  }
  return i->second;
}

// Intersection types. Classifications of various intersections.
// Maximum value = 15 (DO NOT EXCEED!)
enum class IntersectionType : uint8_t {
  kRegular = 0, // Regular, unclassified intersection
  kFalse = 1,   // False intersection. Only 2 edges connect. Typically
                // where 2 ways are split or where attributes force a split.
  kDeadEnd = 2, // Node only connects to one edge ("dead-end").
  kFork = 3     // All edges are links OR all edges are not links
                // and node is a motorway_junction.
};
inline std::string to_string(IntersectionType x) {
  static const std::unordered_map<uint8_t, std::string> IntersectionTypeStrings = {
      {static_cast<uint8_t>(IntersectionType::kRegular), "regular"},
      {static_cast<uint8_t>(IntersectionType::kFalse), "false"},
      {static_cast<uint8_t>(IntersectionType::kDeadEnd), "dead-end"},
      {static_cast<uint8_t>(IntersectionType::kFork), "fork"},
  };

  auto i = IntersectionTypeStrings.find(static_cast<uint8_t>(x));
  if (i == IntersectionTypeStrings.cend()) {
    return "null";
  }
  return i->second;
}

// Edge use. Indicates specialized uses.
// Maximum value that can be stored for a directed edge is 63 - DO NOT EXCEED!
enum class Use : uint8_t {
  // Road specific uses
  kRoad = 0,
  kRamp = 1,            // Link - exits/entrance ramps.
  kTurnChannel = 2,     // Link - turn lane.
  kTrack = 3,           // Agricultural use, forest tracks
  kDriveway = 4,        // Driveway/private service
  kAlley = 5,           // Service road - limited route use
  kParkingAisle = 6,    // Access roads in parking areas
  kEmergencyAccess = 7, // Emergency vehicles only
  kDriveThru = 8,       // Commercial drive-thru (banks/fast-food)
  kCuldesac = 9,        // Cul-de-sac (edge that forms a loop and is only
                        // connected at one node to another edge.
  kLivingStreet = 10,   // Streets with preference towards bicyclists and pedestrians

  // Bicycle specific uses
  kCycleway = 20,     // Dedicated bicycle path
  kMountainBike = 21, // Mountain bike trail

  kSidewalk = 24,

  // Pedestrian specific uses
  kFootway = 25,
  kSteps = 26, // Stairs
  kPath = 27,
  kPedestrian = 28,
  kBridleway = 29,

  // Other...
  kOther = 40,

  // Ferry and rail ferry
  kFerry = 41,
  kRailFerry = 42,

  // Transit specific uses. Must be last in the list
  kRail = 50,               // Rail line
  kBus = 51,                // Bus line
  kEgressConnection = 52,   // Connection to a egress node
  kPlatformConnection = 53, // Connection to a platform node
  kTransitConnection = 54,  // Connection to multi-use transit stop
  kBikeShareConnection = 55 // Connection to multi-use transit stop

};
inline std::string to_string(Use u) {
  static const std::unordered_map<uint8_t, std::string> UseStrings = {
      {static_cast<uint8_t>(Use::kRoad), "road"},
      {static_cast<uint8_t>(Use::kRamp), "ramp"},
      {static_cast<uint8_t>(Use::kTurnChannel), "turn_channel"},
      {static_cast<uint8_t>(Use::kTrack), "track"},
      {static_cast<uint8_t>(Use::kDriveway), "driveway"},
      {static_cast<uint8_t>(Use::kAlley), "alley"},
      {static_cast<uint8_t>(Use::kParkingAisle), "parking_aisle"},
      {static_cast<uint8_t>(Use::kEmergencyAccess), "emergency_access"},
      {static_cast<uint8_t>(Use::kDriveThru), "drive_through"},
      {static_cast<uint8_t>(Use::kCuldesac), "culdesac"},
      {static_cast<uint8_t>(Use::kLivingStreet), "living_street"},
      {static_cast<uint8_t>(Use::kCycleway), "cycleway"},
      {static_cast<uint8_t>(Use::kMountainBike), "mountain_bike"},
      {static_cast<uint8_t>(Use::kSidewalk), "sidewalk"},
      {static_cast<uint8_t>(Use::kFootway), "footway"},
      {static_cast<uint8_t>(Use::kSteps), "steps"},
      {static_cast<uint8_t>(Use::kPath), "path"},
      {static_cast<uint8_t>(Use::kPedestrian), "pedestrian"},
      {static_cast<uint8_t>(Use::kBridleway), "bridleway"},
      {static_cast<uint8_t>(Use::kOther), "other"},
      {static_cast<uint8_t>(Use::kRailFerry), "rail-ferry"},
      {static_cast<uint8_t>(Use::kFerry), "ferry"},
      {static_cast<uint8_t>(Use::kRail), "rail"},
      {static_cast<uint8_t>(Use::kBus), "bus"},
      {static_cast<uint8_t>(Use::kEgressConnection), "egress_connection"},
      {static_cast<uint8_t>(Use::kPlatformConnection), "platform_connnection"},
      {static_cast<uint8_t>(Use::kTransitConnection), "transit_connection"},
      {static_cast<uint8_t>(Use::kBikeShareConnection), "bike_share_connection"},
  };

  auto i = UseStrings.find(static_cast<uint8_t>(u));
  if (i == UseStrings.cend()) {
    return "null";
  }
  return i->second;
}

// Speed type
enum class SpeedType : uint8_t {
  kTagged = 0,    // Tagged maximum speed
  kClassified = 1 // Speed assigned based on highway classification
};
inline std::string to_string(SpeedType s) {
  static const std::unordered_map<uint8_t, std::string> SpeedTypeStrings = {
      {static_cast<uint8_t>(SpeedType::kTagged), "tagged"},
      {static_cast<uint8_t>(SpeedType::kClassified), "classified"},
  };

  auto i = SpeedTypeStrings.find(static_cast<uint8_t>(s));
  if (i == SpeedTypeStrings.cend()) {
    return "null";
  }
  return i->second;
}

// Indication of the type of cycle lane (if any) present along an edge.
// Higher values are more favorable to safe bicycling.
// If edge is a cycleway, footway, or path, then there is an alternate meaning
enum class CycleLane : uint8_t {
  kNone = 0,      // No specified bicycle lane
  kShared = 1,    // Shared use lane (could be shared with pedestrians)
                  // Alternative: Shared path with pedestrians
  kDedicated = 2, // Dedicated cycle lane
                  // Alternative: Path with segregated lanes
  kSeparated = 3  // A separate cycle lane (physical separation from the
                  // main carriageway)
                  // Alternative: Path with no pedestrians on it
};
inline std::string to_string(CycleLane c) {
  static const std::unordered_map<uint8_t, std::string> CycleLaneStrings = {
      {static_cast<uint8_t>(CycleLane::kNone), "none"},
      {static_cast<uint8_t>(CycleLane::kShared), "shared"},
      {static_cast<uint8_t>(CycleLane::kDedicated), "dedicated"},
      {static_cast<uint8_t>(CycleLane::kSeparated), "separated"},
  };
  auto i = CycleLaneStrings.find(static_cast<uint8_t>(c));
  if (i == CycleLaneStrings.cend()) {
    return "null";
  }
  return i->second;
}

enum class SacScale : uint8_t {
  kNone = 0,
  kHiking = 1,
  kMountainHiking = 2,
  kDemandingMountainHiking = 3,
  kAlpineHiking = 4,
  kDemandingAlpineHiking = 5,
  kDifficultAlpineHiking = 6
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
inline std::string to_string(Surface s) {
  static const std::unordered_map<uint8_t, std::string> SurfaceStrings = {
      {static_cast<uint8_t>(Surface::kPavedSmooth), "paved_smooth"},
      {static_cast<uint8_t>(Surface::kPaved), "paved"},
      {static_cast<uint8_t>(Surface::kPavedRough), "paved_rough"},
      {static_cast<uint8_t>(Surface::kCompacted), "compacted"},
      {static_cast<uint8_t>(Surface::kDirt), "dirt"},
      {static_cast<uint8_t>(Surface::kGravel), "gravel"},
      {static_cast<uint8_t>(Surface::kPath), "path"},
      {static_cast<uint8_t>(Surface::kImpassable), "impassable"},
  };

  auto i = SurfaceStrings.find(static_cast<uint8_t>(s));
  if (i == SurfaceStrings.cend()) {
    return "null";
  }
  return i->second;
}

// Used for restrictions.  A restriction can start and end on a particular day
enum class DOW : uint8_t {
  kNone = 0,
  kSunday = 1,
  kMonday = 2,
  kTuesday = 3,
  kWednesday = 4,
  kThursday = 5,
  kFriday = 6,
  kSaturday = 7
};

// Used for restrictions.  A restriction can start and end on a particular month
enum class MONTH : uint8_t {
  kNone = 0,
  kJan = 1,
  kFeb = 2,
  kMar = 3,
  kApr = 4,
  kMay = 5,
  kJun = 6,
  kJul = 7,
  kAug = 8,
  kSep = 9,
  kOct = 10,
  kNov = 11,
  kDec = 12
};

// Used for transit. Types of transit currently supported.
enum class TransitType : uint8_t {
  kTram = 0,
  kMetro = 1,
  kRail = 2,
  kBus = 3,
  kFerry = 4,
  kCableCar = 5,
  kGondola = 6,
  kFunicular = 7
};

// This is our pivot date for transit.  No dates will be older than this date.
const std::string kPivotDate = "2014-01-01"; // January 1, 2014

// Used for day of week mask.
constexpr uint8_t kDOWNone = 0;
constexpr uint8_t kSunday = 1;
constexpr uint8_t kMonday = 2;
constexpr uint8_t kTuesday = 4;
constexpr uint8_t kWednesday = 8;
constexpr uint8_t kThursday = 16;
constexpr uint8_t kFriday = 32;
constexpr uint8_t kSaturday = 64;
constexpr uint8_t kAllDaysOfWeek = 127;

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
  kOnlyStraightOn = 6,
  kNoEntry = 7,
  kNoExit = 8,
  kNoTurn = 9
};

// Access Restriction types. Maximum value supported is 31. DO NOT EXCEED.
enum class AccessType : uint8_t {
  kHazmat = 0,
  kMaxHeight = 1,
  kMaxWidth = 2,
  kMaxLength = 3,
  kMaxWeight = 4,
  kMaxAxleLoad = 5,
  kTimedAllowed = 6,
  kTimedDenied = 7
};

// Minimum meters offset from start/end of shape for finding heading
constexpr float kMinMetersOffsetForHeading = 15.0f;
inline float GetOffsetForHeading(RoadClass road_class, Use use) {
  uint8_t rc = static_cast<uint8_t>(road_class);
  float offset = kMinMetersOffsetForHeading;
  // Adjust offset based on road class
  if (rc < 2) {
    offset *= 1.6f;
  } else if (rc < 5) {
    offset *= 1.4f;
  }

  // Adjust offset based on use
  switch (use) {
    case Use::kCycleway:
    case Use::kMountainBike:
    case Use::kFootway:
    case Use::kSteps:
    case Use::kPath:
    case Use::kPedestrian:
    case Use::kBridleway: {
      offset *= 0.5f;
    }
    default:
      break;
  }

  return offset;
}

// ------------------------------- Transit information --------------------- //

constexpr uint32_t kOneStopIdSize = 256;

// Transit transfer types
enum class TransferType : uint8_t {
  kRecommended = 0, // Recommended transfer point between 2 routes
  kTimed = 1,       // Timed transfer between 2 routes. Departing vehicle
                    // is expected to wait, allowing sufficient time for
                    // passengers to transfer.
  kMinTime = 2,     // Transfer is expected to take the time specified.
  kNotPossible = 3  // Transfers not possible between routes
};

enum class CalendarExceptionType : uint8_t {
  kAdded = 1,  // Service added for the specified date
  kRemoved = 2 // Service removed for the specified date
};

// --------------------- Traffic information ------------------------ //

// Traffic type constants
constexpr uint8_t kNoFlowMask = 0;
constexpr uint8_t kFreeFlowMask = 1;
constexpr uint8_t kConstrainedFlowMask = 2;
constexpr uint8_t kPredictedFlowMask = 4;
constexpr uint8_t kCurrentFlowMask = 8;
constexpr uint8_t kDefaultFlowMask =
    kFreeFlowMask | kConstrainedFlowMask | kPredictedFlowMask | kCurrentFlowMask;
constexpr uint32_t kFreeFlowSecondOfDay = 60 * 60 * 0;         // midnight
constexpr uint32_t kConstrainedFlowSecondOfDay = 60 * 60 * 12; // noon
constexpr uint32_t kInvalidSecondsOfWeek = -1;                 // invalid

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_GRAPHCONSTANTS_H_
