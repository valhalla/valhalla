#pragma once
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/proto/api.pb.h>
#include <valhalla/proto/incidents.pb.h>
#include <valhalla/sif/costconstants.h>

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
  return (type <= static_cast<uint8_t>(sif::VehicleType::kMotorScooter)) ? kTripLegVehicleType[type]
                                                                         : kTripLegVehicleType[0];
}

// Associate pedestrian types to TripLeg proto
constexpr TripLeg_PedestrianType kTripLegPedestrianType[] =
    {TripLeg_PedestrianType::TripLeg_PedestrianType_kFoot,
     TripLeg_PedestrianType::TripLeg_PedestrianType_kWheelchair,
     TripLeg_PedestrianType::TripLeg_PedestrianType_kSegway};
inline TripLeg_PedestrianType GetTripLegPedestrianType(const uint8_t type) {
  return (type <= static_cast<uint8_t>(sif::PedestrianType::kSegway)) ? kTripLegPedestrianType[type]
                                                                      : kTripLegPedestrianType[0];
}

// Associate bicycle types to TripLeg proto
constexpr TripLeg_BicycleType kTripLegBicycleType[] =
    {TripLeg_BicycleType::TripLeg_BicycleType_kRoad, TripLeg_BicycleType::TripLeg_BicycleType_kCross,
     TripLeg_BicycleType::TripLeg_BicycleType_kHybrid,
     TripLeg_BicycleType::TripLeg_BicycleType_kMountain};
inline TripLeg_BicycleType GetTripLegBicycleType(const uint8_t type) {
  return (type <= static_cast<uint8_t>(sif::BicycleType::kMountain)) ? kTripLegBicycleType[type]
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
inline TripLeg_TransitType GetTripLegTransitType(const baldr::TransitType transit_type) {
  return kTripLegTransitType[static_cast<uint32_t>(transit_type)];
}

// Associate traversability values to TripLeg proto
constexpr TripLeg_Traversability kTripLegTraversability[] = {TripLeg_Traversability_kNone,
                                                             TripLeg_Traversability_kForward,
                                                             TripLeg_Traversability_kBackward,
                                                             TripLeg_Traversability_kBoth};
inline TripLeg_Traversability GetTripLegTraversability(const baldr::Traversability traversability) {
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

inline TripLeg_Node_Type GetTripLegNodeType(const baldr::NodeType node_type) {
  switch (node_type) {
    case baldr::NodeType::kStreetIntersection:
      return TripLeg_Node_Type_kStreetIntersection;
    case baldr::NodeType::kGate:
      return TripLeg_Node_Type_kGate;
    case baldr::NodeType::kBollard:
      return TripLeg_Node_Type_kBollard;
    case baldr::NodeType::kTollBooth:
      return TripLeg_Node_Type_kTollBooth;
    case baldr::NodeType::kTransitEgress:
      return TripLeg_Node_Type_kTransitEgress;
    case baldr::NodeType::kTransitStation:
      return TripLeg_Node_Type_kTransitStation;
    case baldr::NodeType::kMultiUseTransitPlatform:
      return TripLeg_Node_Type_kTransitPlatform;
    case baldr::NodeType::kBikeShare:
      return TripLeg_Node_Type_kBikeShare;
    case baldr::NodeType::kParking:
      return TripLeg_Node_Type_kParking;
    case baldr::NodeType::kMotorWayJunction:
      return TripLeg_Node_Type_kMotorwayJunction;
    case baldr::NodeType::kBorderControl:
      return TripLeg_Node_Type_kBorderControl;
    case baldr::NodeType::kTollGantry:
      return TripLeg_Node_Type_kTollGantry;
    case baldr::NodeType::kSumpBuster:
      return TripLeg_Node_Type_kSumpBuster;
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
inline TripLeg_CycleLane GetTripLegCycleLane(const baldr::CycleLane cyclelane) {
  return kTripLegCycleLane[static_cast<uint32_t>(cyclelane)];
}

// Associate Use to TripLeg proto
inline TripLeg_Use GetTripLegUse(const baldr::Use use) {
  switch (use) {
    case baldr::Use::kRoad:
      return TripLeg_Use_kRoadUse;
    case baldr::Use::kRamp:
      return TripLeg_Use_kRampUse;
    case baldr::Use::kTurnChannel:
      return TripLeg_Use_kTurnChannelUse;
    case baldr::Use::kTrack:
      return TripLeg_Use_kTrackUse;
    case baldr::Use::kDriveway:
      return TripLeg_Use_kDrivewayUse;
    case baldr::Use::kAlley:
      return TripLeg_Use_kAlleyUse;
    case baldr::Use::kParkingAisle:
      return TripLeg_Use_kParkingAisleUse;
    case baldr::Use::kEmergencyAccess:
      return TripLeg_Use_kEmergencyAccessUse;
    case baldr::Use::kDriveThru:
      return TripLeg_Use_kDriveThruUse;
    case baldr::Use::kCuldesac:
      return TripLeg_Use_kCuldesacUse;
    case baldr::Use::kLivingStreet:
      return TripLeg_Use_kLivingStreetUse;
    case baldr::Use::kCycleway:
      return TripLeg_Use_kCyclewayUse;
    case baldr::Use::kMountainBike:
      return TripLeg_Use_kMountainBikeUse;
    case baldr::Use::kSidewalk:
      // return TripLeg_Use_kSidewalkUse;
      return TripLeg_Use_kFootwayUse; // TODO: update when odin has been updated
    case baldr::Use::kFootway:
      return TripLeg_Use_kFootwayUse;
    case baldr::Use::kSteps:
      return TripLeg_Use_kStepsUse;
    case baldr::Use::kPath:
      return TripLeg_Use_kPathUse;
    case baldr::Use::kPedestrian:
      return TripLeg_Use_kPedestrianUse;
    case baldr::Use::kBridleway:
      return TripLeg_Use_kBridlewayUse;
    case baldr::Use::kRestArea:
      return TripLeg_Use_kRestAreaUse;
    case baldr::Use::kServiceArea:
      return TripLeg_Use_kServiceAreaUse;
    case baldr::Use::kOther:
      return TripLeg_Use_kOtherUse;
    case baldr::Use::kFerry:
      return TripLeg_Use_kFerryUse;
    case baldr::Use::kRailFerry:
      return TripLeg_Use_kRailFerryUse;
    case baldr::Use::kRail:
      return TripLeg_Use_kRailUse;
    case baldr::Use::kBus:
      return TripLeg_Use_kBusUse;
    case baldr::Use::kEgressConnection:
      return TripLeg_Use_kEgressConnectionUse;
    case baldr::Use::kPlatformConnection:
      return TripLeg_Use_kPlatformConnectionUse;
    case baldr::Use::kTransitConnection:
      return TripLeg_Use_kTransitConnectionUse;
      // Should not see other values
    default:
      // TODO should we throw a runtime error?
      return TripLeg_Use_kRoadUse;
  }
}

// Get the string representing the incident-type
std::string incidentTypeToString(const valhalla::IncidentsTile::Metadata::Type& incident_type);
// Get the string representing the incident-Impact
const char* incidentImpactToString(const valhalla::IncidentsTile::Metadata::Impact& impact);
// Get the string representing the guidance view type
const std::string& GuidanceViewTypeToString(const valhalla::DirectionsLeg_GuidanceView_Type type);

// to use protobuflite we cant use descriptors which means we cant translate enums to strings
// and so we reimplement the ones we use here. newer versions of protobuf provide these even
// for the lite bindings so we avoid collisions by picking somewhat goofy names. an alternative
// which would allow us to delete this completely would be to target a newer protobuf version
bool Options_Action_Enum_Parse(const std::string& action, Options::Action* a);
const std::string& Options_Action_Enum_Name(const Options::Action action);
bool Costing_Enum_Parse(const std::string& costing, Costing* c);
const std::string& Costing_Enum_Name(const Costing costing);
bool ShapeMatch_Enum_Parse(const std::string& match, ShapeMatch* s);
const std::string& ShapeMatch_Enum_Name(const ShapeMatch match);
bool Options_Format_Enum_Parse(const std::string& format, Options::Format* f);
const std::string& Options_Format_Enum_Name(const Options::Format match);
const std::string& Options_Units_Enum_Name(const Options::Units unit);
bool FilterAction_Enum_Parse(const std::string& action, FilterAction* a);
const std::string& FilterAction_Enum_Name(const FilterAction action);
bool DirectionsType_Enum_Parse(const std::string& dtype, DirectionsType* t);
bool PreferredSide_Enum_Parse(const std::string& pside, valhalla::Location::PreferredSide* p);
bool RoadClass_Enum_Parse(const std::string& rc_name, valhalla::RoadClass* rc);
bool Location_Type_Enum_Parse(const std::string& type, Location::Type* t);
const std::string& Location_Type_Enum_Name(const Location::Type t);
const std::string& Location_SideOfStreet_Enum_Name(const Location::SideOfStreet s);

std::pair<std::string, std::string>
travel_mode_type(const valhalla::DirectionsLeg_Maneuver& maneuver);

inline midgard::PointLL to_ll(const LatLng& ll) {
  return midgard::PointLL{ll.lng(), ll.lat()};
}

inline void from_ll(valhalla::Location* l, const midgard::PointLL& p) {
  l->mutable_ll()->set_lat(p.lat());
  l->mutable_ll()->set_lng(p.lng());
}

} // namespace valhalla
