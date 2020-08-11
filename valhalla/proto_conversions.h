#pragma once

#include <valhalla/baldr/graphconstants.h>
#include <valhalla/proto/api.pb.h>
#include <valhalla/sif/costconstants.h>

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace valhalla {
// Associate RoadClass values to TripLeg proto
constexpr valhalla::RoadClass kTripLegRoadClass[] = {valhalla::RoadClass::kMotorway,
                                                     valhalla::RoadClass::kTrunk,
                                                     valhalla::RoadClass::kPrimary,
                                                     valhalla::RoadClass::kSecondary,
                                                     valhalla::RoadClass::kTertiary,
                                                     valhalla::RoadClass::kUnclassified,
                                                     valhalla::RoadClass::kResidential,
                                                     valhalla::RoadClass::kServiceOther};
inline valhalla::RoadClass GetRoadClass(const baldr::RoadClass road_class) {
  return kTripLegRoadClass[static_cast<int>(road_class)];
}

// Associate Surface values to TripLeg proto
constexpr TripLeg_Surface kTripLegSurface[] =
    {TripLeg_Surface_kPavedSmooth, TripLeg_Surface_kPaved,     TripLeg_Surface_kPavedRough,
     TripLeg_Surface_kCompacted,   TripLeg_Surface_kDirt,      TripLeg_Surface_kGravel,
     TripLeg_Surface_kPath,        TripLeg_Surface_kImpassable};
inline TripLeg_Surface GetTripLegSurface(const baldr::Surface surface) {
  return kTripLegSurface[static_cast<int>(surface)];
}

// Associate vehicle types to TripLeg proto
// TODO - why doesn't these use an enum input?
constexpr TripLeg_VehicleType kTripLegVehicleType[] =
    {TripLeg_VehicleType::TripLeg_VehicleType_kCar,
     TripLeg_VehicleType::TripLeg_VehicleType_kMotorcycle,
     TripLeg_VehicleType::TripLeg_VehicleType_kAutoBus,
     TripLeg_VehicleType::TripLeg_VehicleType_kTractorTrailer,
     TripLeg_VehicleType::TripLeg_VehicleType_kMotorScooter};
inline TripLeg_VehicleType GetTripLegVehicleType(const uint8_t type) {
  return (type <= static_cast<uint8_t>(VehicleType::kMotorScooter)) ? kTripLegVehicleType[type]
                                                                    : kTripLegVehicleType[0];
}

// Associate pedestrian types to TripLeg proto
constexpr TripLeg_PedestrianType kTripLegPedestrianType[] =
    {TripLeg_PedestrianType::TripLeg_PedestrianType_kFoot,
     TripLeg_PedestrianType::TripLeg_PedestrianType_kWheelchair,
     TripLeg_PedestrianType::TripLeg_PedestrianType_kSegway};
inline TripLeg_PedestrianType GetTripLegPedestrianType(const uint8_t type) {
  return (type <= static_cast<uint8_t>(PedestrianType::kSegway)) ? kTripLegPedestrianType[type]
                                                                 : kTripLegPedestrianType[0];
}

// Associate bicycle types to TripLeg proto
constexpr TripLeg_BicycleType kTripLegBicycleType[] =
    {TripLeg_BicycleType::TripLeg_BicycleType_kRoad, TripLeg_BicycleType::TripLeg_BicycleType_kCross,
     TripLeg_BicycleType::TripLeg_BicycleType_kHybrid,
     TripLeg_BicycleType::TripLeg_BicycleType_kMountain};
inline TripLeg_BicycleType GetTripLegBicycleType(const uint8_t type) {
  return (type <= static_cast<uint8_t>(BicycleType::kMountain)) ? kTripLegBicycleType[type]
                                                                : kTripLegBicycleType[0];
}

// Associate transit types to TripLeg proto
constexpr TripLeg_TransitType kTripLegTransitType[] =
    {TripLeg_TransitType::TripLeg_TransitType_kTram,
     TripLeg_TransitType::TripLeg_TransitType_kMetro,
     TripLeg_TransitType::TripLeg_TransitType_kRail,
     TripLeg_TransitType::TripLeg_TransitType_kBus,
     TripLeg_TransitType::TripLeg_TransitType_kFerry,
     TripLeg_TransitType::TripLeg_TransitType_kCableCar,
     TripLeg_TransitType::TripLeg_TransitType_kGondola,
     TripLeg_TransitType::TripLeg_TransitType_kFunicular};
inline TripLeg_TransitType GetTripLegTransitType(const TransitType transit_type) {
  return kTripLegTransitType[static_cast<uint32_t>(transit_type)];
}

// Associate traversability values to TripLeg proto
constexpr TripLeg_Traversability kTripLegTraversability[] = {TripLeg_Traversability_kNone,
                                                             TripLeg_Traversability_kForward,
                                                             TripLeg_Traversability_kBackward,
                                                             TripLeg_Traversability_kBoth};
inline TripLeg_Traversability GetTripLegTraversability(const Traversability traversability) {
  return kTripLegTraversability[static_cast<uint32_t>(traversability)];
}

// Associate side of street to TripLeg proto
constexpr valhalla::Location::SideOfStreet kTripLegSideOfStreet[] = {valhalla::Location::kNone,
                                                                     valhalla::Location::kLeft,
                                                                     valhalla::Location::kRight};
inline valhalla::Location::SideOfStreet
GetTripLegSideOfStreet(const valhalla::Location::SideOfStreet sos) {
  return kTripLegSideOfStreet[static_cast<uint32_t>(sos)];
}

inline TripLeg_Node_Type GetTripLegNodeType(const NodeType node_type) {
  switch (node_type) {
    case NodeType::kStreetIntersection:
      return TripLeg_Node_Type_kStreetIntersection;
    case NodeType::kGate:
      return TripLeg_Node_Type_kGate;
    case NodeType::kBollard:
      return TripLeg_Node_Type_kBollard;
    case NodeType::kTollBooth:
      return TripLeg_Node_Type_kTollBooth;
    case NodeType::kTransitEgress:
      return TripLeg_Node_Type_kTransitEgress;
    case NodeType::kTransitStation:
      return TripLeg_Node_Type_kTransitStation;
    case NodeType::kMultiUseTransitPlatform:
      return TripLeg_Node_Type_kTransitPlatform;
    case NodeType::kBikeShare:
      return TripLeg_Node_Type_kBikeShare;
    case NodeType::kParking:
      return TripLeg_Node_Type_kParking;
    case NodeType::kMotorWayJunction:
      return TripLeg_Node_Type_kMotorwayJunction;
    case NodeType::kBorderControl:
      return TripLeg_Node_Type_kBorderControl;
  }
  auto num = static_cast<uint8_t>(node_type);
  throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__) +
                           " Unhandled NodeType: " + std::to_string(num));
}

// Associate cycle lane values to TripLeg proto
constexpr TripLeg_CycleLane kTripLegCycleLane[] = {TripLeg_CycleLane_kNoCycleLane,
                                                   TripLeg_CycleLane_kShared,
                                                   TripLeg_CycleLane_kDedicated,
                                                   TripLeg_CycleLane_kSeparated};
inline TripLeg_CycleLane GetTripLegCycleLane(const CycleLane cyclelane) {
  return kTripLegCycleLane[static_cast<uint32_t>(cyclelane)];
}

// Associate Use to TripLeg proto
inline TripLeg_Use GetTripLegUse(const Use use) {
  switch (use) {
    case Use::kRoad:
      return TripLeg_Use_kRoadUse;
    case Use::kRamp:
      return TripLeg_Use_kRampUse;
    case Use::kTurnChannel:
      return TripLeg_Use_kTurnChannelUse;
    case Use::kTrack:
      return TripLeg_Use_kTrackUse;
    case Use::kDriveway:
      return TripLeg_Use_kDrivewayUse;
    case Use::kAlley:
      return TripLeg_Use_kAlleyUse;
    case Use::kParkingAisle:
      return TripLeg_Use_kParkingAisleUse;
    case Use::kEmergencyAccess:
      return TripLeg_Use_kEmergencyAccessUse;
    case Use::kDriveThru:
      return TripLeg_Use_kDriveThruUse;
    case Use::kCuldesac:
      return TripLeg_Use_kCuldesacUse;
    case Use::kLivingStreet:
      return TripLeg_Use_kLivingStreetUse;
    case Use::kCycleway:
      return TripLeg_Use_kCyclewayUse;
    case Use::kMountainBike:
      return TripLeg_Use_kMountainBikeUse;
    case Use::kSidewalk:
      // return TripLeg_Use_kSidewalkUse;
      return TripLeg_Use_kFootwayUse; // TODO: update when odin has been updated
    case Use::kFootway:
      return TripLeg_Use_kFootwayUse;
    case Use::kSteps:
      return TripLeg_Use_kStepsUse;
    case Use::kPath:
      return TripLeg_Use_kPathUse;
    case Use::kPedestrian:
      return TripLeg_Use_kPedestrianUse;
    case Use::kBridleway:
      return TripLeg_Use_kBridlewayUse;
    case Use::kOther:
      return TripLeg_Use_kOtherUse;
    case Use::kFerry:
      return TripLeg_Use_kFerryUse;
    case Use::kRailFerry:
      return TripLeg_Use_kRailFerryUse;
    case Use::kRail:
      return TripLeg_Use_kRailUse;
    case Use::kBus:
      return TripLeg_Use_kBusUse;
    case Use::kEgressConnection:
      return TripLeg_Use_kEgressConnectionUse;
    case Use::kPlatformConnection:
      return TripLeg_Use_kPlatformConnectionUse;
    case Use::kTransitConnection:
      return TripLeg_Use_kTransitConnectionUse;
      // Should not see other values
    default:
      // TODO should we throw a runtime error?
      return TripLeg_Use_kRoadUse;
  }
}
} // namespace valhalla
