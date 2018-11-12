#include <cmath>
#include <cstdlib>
#include <iostream>

#include "midgard/constants.h"
#include "midgard/util.h"
#include "worker.h"

#include "odin/enhancedtrippath.h"
#include "odin/util.h"

#include <valhalla/proto/trippath.pb.h>

using namespace valhalla::midgard;

namespace {
const std::string& TripPath_RoadClass_Name(int v) {
  static const std::unordered_map<int, std::string> values{
      {0, "kMotorway"}, {1, "kTrunk"},        {2, "kPrimary"},     {3, "kSecondary"},
      {4, "kTertiary"}, {5, "kUnclassified"}, {6, "kResidential"}, {7, "kServiceOther"},
  };
  auto f = values.find(v);
  if (f == values.cend())
    throw std::runtime_error("Missing value in protobuf enum to string");
  return f->second;
}

const std::string& TripPath_Traversability_Name(int v) {
  static const std::unordered_map<int, std::string> values{
      {0, "kNone"},
      {1, "kForward"},
      {2, "kward"},
      {3, "kBoth"},
  };
  auto f = values.find(v);
  if (f == values.cend())
    throw std::runtime_error("Missing value in protobuf enum to string");
  return f->second;
}

const std::string& TripPath_Use_Name(int v) {
  static const std::unordered_map<int, std::string> values{
      {0, "kRoadUse"},
      {1, "kRampUse"},
      {2, "kTurnChannelUse"},
      {3, "kUse"},
      {4, "kDrivewayUse"},
      {5, "kAlleyUse"},
      {6, "kingAisleUse"},
      {7, "kEmergencyAccessUse"},
      {8, "kDriveThruUse"},
      {9, "kCuldesacUse"},
      {20, "kCyclewayUse"},
      {21, "keUse"},
      {24, "kUse"},
      {25, "kFootwayUse"},
      {26, "kStepsUse"},
      {27, "kPathUse"},
      {28, "kPedestrianUse"},
      {29, "kBridlewayUse"},
      {40, "kOtherUse"},
      {41, "kFerryUse"},
      {42, "kRailFerryUse"},
      {50, "kRailUse"},
      {51, "kBusUse"},
      {52, "kEgressConnectionUse"},
      {53, "kPlatformConnectionUse"},
      {54, "kTransitConnectionUse"},
  };
  auto f = values.find(v);
  if (f == values.cend())
    throw std::runtime_error("Missing value in protobuf enum to string");
  return f->second;
}

const std::string& TripPath_TravelMode_Name(int v) {
  static const std::unordered_map<int, std::string> values{
      {0, "kDrive"},
      {1, "kPedestrian"},
      {2, "kBicycle"},
      {3, "kTransit"},
  };
  auto f = values.find(v);
  if (f == values.cend())
    throw std::runtime_error("Missing value in protobuf enum to string");
  return f->second;
}

const std::string& TripPath_VehicleType_Name(int v) {
  static const std::unordered_map<int, std::string> values{
      {0, "kCar"}, {1, "kMotorcycle"}, {2, "kAutoBus"}, {3, "kTractorTrailer"}, {4, "kMotorScooter"},
  };
  auto f = values.find(v);
  if (f == values.cend())
    throw std::runtime_error("Missing value in protobuf enum to string");
  return f->second;
}

const std::string& TripPath_PedestrianType_Name(int v) {
  static const std::unordered_map<int, std::string> values{
      {0, "kFoot"},
      {1, "kWheelchair"},
      {2, "kSegway"},
  };
  auto f = values.find(v);
  if (f == values.cend())
    throw std::runtime_error("Missing value in protobuf enum to string");
  return f->second;
}

const std::string& TripPath_BicycleType_Name(int v) {
  static const std::unordered_map<int, std::string> values{
      {0, "kRoad"},
      {1, "kCross"},
      {2, "kHybrid"},
      {3, "kMountain"},
  };
  auto f = values.find(v);
  if (f == values.cend())
    throw std::runtime_error("Missing value in protobuf enum to string");
  return f->second;
}

const std::string& TripPath_TransitType_Name(int v) {
  static const std::unordered_map<int, std::string> values{
      {0, "kTram"},  {1, "kMetro"},    {2, "kRail"},    {3, "kBus"},
      {4, "kFerry"}, {5, "kCableCar"}, {6, "kGondola"}, {7, "kFunicular"},
  };
  auto f = values.find(v);
  if (f == values.cend())
    throw std::runtime_error("Missing value in protobuf enum to string");
  return f->second;
}

const std::string& TripPath_CycleLane_Name(int v) {
  static const std::unordered_map<int, std::string> values{
      {0, "kNoCycleLane"},
      {1, "kShared"},
      {2, "kDedicated"},
      {3, "kSeparated"},
  };
  auto f = values.find(v);
  if (f == values.cend())
    throw std::runtime_error("Missing value in protobuf enum to string");
  return f->second;
}

const std::string& TripPath_Sidewalk_Name(int v) {
  static const std::unordered_map<int, std::string> values{
      {0, "kNoSidewalk"},
      {1, "kLeft"},
      {2, "kRight"},
      {3, "kBothSides"},
  };
  auto f = values.find(v);
  if (f == values.cend())
    throw std::runtime_error("Missing value in protobuf enum to string");
  return f->second;
}

} // namespace

namespace valhalla {
namespace odin {

///////////////////////////////////////////////////////////////////////////////
// EnhancedTripPath

EnhancedTripPath_Node* EnhancedTripPath::GetEnhancedNode(const int node_index) {
  return static_cast<EnhancedTripPath_Node*>(mutable_node(node_index));
}

EnhancedTripPath_Edge* EnhancedTripPath::GetPrevEdge(const int node_index, int delta) {
  int index = node_index - delta;
  if (IsValidNodeIndex(index)) {
    return static_cast<EnhancedTripPath_Edge*>(mutable_node(index)->mutable_edge());
  } else {
    return nullptr;
  }
}

EnhancedTripPath_Edge* EnhancedTripPath::GetCurrEdge(const int node_index) {
  return GetNextEdge(node_index, 0);
}

EnhancedTripPath_Edge* EnhancedTripPath::GetNextEdge(const int node_index, int delta) {
  int index = node_index + delta;
  if (IsValidNodeIndex(index) && !IsLastNodeIndex(index)) {
    return static_cast<EnhancedTripPath_Edge*>(mutable_node(index)->mutable_edge());
  } else {
    return nullptr;
  }
}

bool EnhancedTripPath::IsValidNodeIndex(int node_index) const {
  if ((node_index >= 0) && (node_index < node_size())) {
    return true;
  }
  return false;
}

bool EnhancedTripPath::IsFirstNodeIndex(int node_index) const {
  if (node_index == 0) {
    return true;
  }
  return false;
}

bool EnhancedTripPath::IsLastNodeIndex(int node_index) const {
  if (IsValidNodeIndex(node_index) && (node_index == (node_size() - 1))) {
    return true;
  }
  return false;
}

int EnhancedTripPath::GetLastNodeIndex() const {
  return (node_size() - 1);
}

EnhancedTripPath_Admin* EnhancedTripPath::GetAdmin(size_t index) {
  return static_cast<EnhancedTripPath_Admin*>(mutable_admin(index));
}

std::string EnhancedTripPath::GetCountryCode(int node_index) {
  return GetAdmin(node(node_index).admin_index())->country_code();
}

std::string EnhancedTripPath::GetStateCode(int node_index) {
  return GetAdmin(node(node_index).admin_index())->state_code();
}

const ::valhalla::odin::Location& EnhancedTripPath::GetOrigin() const {
  // Validate location count
  if (location_size() < 2) {
    throw valhalla_exception_t{212};
  }

  return location(0);
}

const ::valhalla::odin::Location& EnhancedTripPath::GetDestination() const {
  // Validate location count
  if (location_size() < 2) {
    throw valhalla_exception_t{212};
  }

  return location(location_size() - 1);
}

float EnhancedTripPath::GetLength(const DirectionsOptions::Units& units) {
  float length = 0.0f;
  for (const auto& n : node()) {
    if (n.has_edge()) {
      length += n.edge().length();
    }
  }
  if (units == DirectionsOptions::miles) {
    return (length * kMilePerKm);
  }
  return length;
}

///////////////////////////////////////////////////////////////////////////////
// EnhancedTripPath_Edge

bool EnhancedTripPath_Edge::IsUnnamed() const {
  return (name_size() == 0);
}

bool EnhancedTripPath_Edge::IsRoadUse() const {
  return (use() == TripPath_Use_kRoadUse);
}

bool EnhancedTripPath_Edge::IsRampUse() const {
  return (use() == TripPath_Use_kRampUse);
}

bool EnhancedTripPath_Edge::IsTurnChannelUse() const {
  return (use() == TripPath_Use_kTurnChannelUse);
}

bool EnhancedTripPath_Edge::IsTrackUse() const {
  return (use() == TripPath_Use_kTrackUse);
}

bool EnhancedTripPath_Edge::IsDrivewayUse() const {
  return (use() == TripPath_Use_kDrivewayUse);
}

bool EnhancedTripPath_Edge::IsAlleyUse() const {
  return (use() == TripPath_Use_kAlleyUse);
}

bool EnhancedTripPath_Edge::IsParkingAisleUse() const {
  return (use() == TripPath_Use_kParkingAisleUse);
}

bool EnhancedTripPath_Edge::IsEmergencyAccessUse() const {
  return (use() == TripPath_Use_kEmergencyAccessUse);
}

bool EnhancedTripPath_Edge::IsDriveThruUse() const {
  return (use() == TripPath_Use_kDriveThruUse);
}

bool EnhancedTripPath_Edge::IsCuldesacUse() const {
  return (use() == TripPath_Use_kCuldesacUse);
}

bool EnhancedTripPath_Edge::IsCyclewayUse() const {
  return (use() == TripPath_Use_kCyclewayUse);
}

bool EnhancedTripPath_Edge::IsMountainBikeUse() const {
  return (use() == TripPath_Use_kMountainBikeUse);
}

bool EnhancedTripPath_Edge::IsSidewalkUse() const {
  return (use() == TripPath_Use_kSidewalkUse);
}

bool EnhancedTripPath_Edge::IsFootwayUse() const {
  return (use() == TripPath_Use_kFootwayUse);
}

bool EnhancedTripPath_Edge::IsStepsUse() const {
  return (use() == TripPath_Use_kStepsUse);
}

bool EnhancedTripPath_Edge::IsPathUse() const {
  return (use() == TripPath_Use_kPathUse);
}

bool EnhancedTripPath_Edge::IsPedestrianUse() const {
  return (use() == TripPath_Use_kPedestrianUse);
}

bool EnhancedTripPath_Edge::IsBridlewayUse() const {
  return (use() == TripPath_Use_kBridlewayUse);
}

bool EnhancedTripPath_Edge::IsOtherUse() const {
  return (use() == TripPath_Use_kOtherUse);
}

bool EnhancedTripPath_Edge::IsFerryUse() const {
  return (use() == TripPath_Use_kFerryUse);
}

bool EnhancedTripPath_Edge::IsRailFerryUse() const {
  return (use() == TripPath_Use_kRailFerryUse);
}

bool EnhancedTripPath_Edge::IsRailUse() const {
  return (use() == TripPath_Use_kRailUse);
}

bool EnhancedTripPath_Edge::IsBusUse() const {
  return (use() == TripPath_Use_kBusUse);
}

bool EnhancedTripPath_Edge::IsEgressConnectionUse() const {
  return (use() == TripPath_Use_kEgressConnectionUse);
}

bool EnhancedTripPath_Edge::IsPlatformConnectionUse() const {
  return (use() == TripPath_Use_kPlatformConnectionUse);
}

bool EnhancedTripPath_Edge::IsTransitConnectionUse() const {
  return (use() == TripPath_Use_kTransitConnectionUse);
}

bool EnhancedTripPath_Edge::IsTransitConnection() const {
  return (IsTransitConnectionUse() || IsEgressConnectionUse() || IsPlatformConnectionUse());
}

bool EnhancedTripPath_Edge::IsUnnamedWalkway() const {
  return (IsUnnamed() && IsFootwayUse());
}

bool EnhancedTripPath_Edge::IsUnnamedCycleway() const {
  return (IsUnnamed() && IsCyclewayUse());
}

bool EnhancedTripPath_Edge::IsUnnamedMountainBikeTrail() const {
  return (IsUnnamed() && IsMountainBikeUse());
}

bool EnhancedTripPath_Edge::IsHighway() const {
  return ((road_class() == TripPath_RoadClass_kMotorway) && (!IsRampUse()));
}

bool EnhancedTripPath_Edge::IsOneway() const {
  return ((traversability() == TripPath_Traversability_kForward) ||
          (traversability() == TripPath_Traversability_kBackward));
}

bool EnhancedTripPath_Edge::IsForward(uint32_t prev2curr_turn_degree) const {
  return ((prev2curr_turn_degree > 314) || (prev2curr_turn_degree < 46));
}

bool EnhancedTripPath_Edge::IsWiderForward(uint32_t prev2curr_turn_degree) const {
  return ((prev2curr_turn_degree > 304) || (prev2curr_turn_degree < 56));
}

bool EnhancedTripPath_Edge::IsStraightest(uint32_t prev2curr_turn_degree,
                                          uint32_t straightest_xedge_turn_degree) const {
  if (IsWiderForward(prev2curr_turn_degree)) {
    int path_xedge_turn_degree_delta = std::abs(static_cast<int>(prev2curr_turn_degree) -
                                                static_cast<int>(straightest_xedge_turn_degree));
    if (path_xedge_turn_degree_delta > 180) {
      path_xedge_turn_degree_delta = (360 - path_xedge_turn_degree_delta);
    }
    uint32_t path_straight_delta =
        (prev2curr_turn_degree > 180) ? (360 - prev2curr_turn_degree) : prev2curr_turn_degree;
    uint32_t xedge_straight_delta = (straightest_xedge_turn_degree > 180)
                                        ? (360 - straightest_xedge_turn_degree)
                                        : straightest_xedge_turn_degree;
    return ((path_xedge_turn_degree_delta > 10) ? (path_straight_delta <= xedge_straight_delta)
                                                : true);
  } else {
    return false;
  }
}

std::vector<std::pair<std::string, bool>> EnhancedTripPath_Edge::GetNameList() const {
  std::vector<std::pair<std::string, bool>> name_list;
  for (const auto& name : this->name()) {
    name_list.push_back({name.value(), name.is_route_number()});
  }
  return name_list;
}

float EnhancedTripPath_Edge::GetLength(const DirectionsOptions::Units& units) {
  if (units == DirectionsOptions::miles) {
    return (length() * kMilePerKm);
  }
  return length();
}

#ifdef LOGGING_LEVEL_TRACE
std::string EnhancedTripPath_Edge::ToString() const {
  std::string str;
  str.reserve(256);

  str += "name=";
  if (name_size() == 0) {
    str += "unnamed";
  } else {
    str += StreetNamesToString(this->name());
  }

  str += " | length=";
  str += std::to_string(length());

  str += " | speed=";
  str += std::to_string(speed());

  str += " | road_class=";
  str += std::to_string(road_class());

  str += " | begin_heading=";
  str += std::to_string(begin_heading());

  str += " | end_heading=";
  str += std::to_string(end_heading());

  str += " | begin_shape_index=";
  str += std::to_string(begin_shape_index());

  str += " | end_shape_index=";
  str += std::to_string(end_shape_index());

  str += " | traversability=";
  str += std::to_string(traversability());

  str += " | use=";
  str += std::to_string(use());

  str += " | toll=";
  str += std::to_string(toll());

  str += " | unpaved=";
  str += std::to_string(unpaved());

  str += " | tunnel=";
  str += std::to_string(tunnel());

  str += " | bridge=";
  str += std::to_string(bridge());

  str += " | roundabout=";
  str += std::to_string(roundabout());

  str += " | internal_intersection=";
  str += std::to_string(internal_intersection());

  // Process exits, if needed
  if (this->has_sign()) {
    str += " | exit_numbers=";
    str += SignElementsToString(this->sign().exit_numbers());

    str += " | exit_onto_streets=";
    str += SignElementsToString(this->sign().exit_onto_streets());

    str += " | exit_toward_locations=";
    str += SignElementsToString(this->sign().exit_toward_locations());

    str += " | exit_names=";
    str += SignElementsToString(this->sign().exit_names());
  }

  str += " | travel_mode=";
  str += std::to_string(travel_mode());

  // NOTE: Current PopulateEdge implementation

  str += " | vehicle_type=";
  str += std::to_string(vehicle_type());

  str += " | pedestrian_type=";
  str += std::to_string(pedestrian_type());

  str += " | bicycle_type=";
  str += std::to_string(bicycle_type());

  str += " | transit_type=";
  str += std::to_string(transit_type());

  str += " | drive_on_right=";
  str += std::to_string(drive_on_right());

  str += " | surface=";
  str += std::to_string(surface());

  // Process transit route info, if needed
  if (has_transit_route_info()) {
    str += " | transit_route_info.onestop_id=";
    str += transit_route_info().onestop_id();

    str += " | transit_route_info.block_id=";
    str += std::to_string(transit_route_info().block_id());

    str += " | transit_route_info.trip_id=";
    str += std::to_string(transit_route_info().trip_id());

    str += " | transit_route_info.short_name=";
    str += transit_route_info().short_name();

    str += " | transit_route_info.long_name=";
    str += transit_route_info().long_name();

    str += " | transit_route_info.headsign=";
    str += transit_route_info().headsign();

    str += " | transit_route_info.color=";
    str += std::to_string(transit_route_info().color());

    str += " | transit_route_info.text_color=";
    str += std::to_string(transit_route_info().text_color());

    str += " | transit_route_info.description=";
    str += transit_route_info().description();

    str += " | transit_route_info.operator_onestop_id=";
    str += transit_route_info().operator_onestop_id();

    str += " | transit_route_info.operator_name=";
    str += transit_route_info().operator_name();

    str += " | transit_route_info.operator_url=";
    str += transit_route_info().operator_url();
  }

  str += " | id=";
  str += std::to_string(id());

  str += " | way_id=";
  str += std::to_string(way_id());

  str += " | weighted_grade=";
  str += std::to_string(weighted_grade());

  str += " | max_upward_grade=";
  str += std::to_string(max_upward_grade());

  str += " | max_downward_grade=";
  str += std::to_string(max_downward_grade());

  str += " | lane_count=";
  str += std::to_string(lane_count());

  str += " | cycle_lane=";
  str += std::to_string(cycle_lane());

  str += " | bicycle_network=";
  str += std::to_string(bicycle_network());

  str += " | sidewalk=";
  str += std::to_string(sidewalk());

  str += " | density=";
  str += std::to_string(density());

  str += " | speed_limit=";
  str += std::to_string(speed_limit());

  str += " | truck_speed=";
  str += std::to_string(truck_speed());

  str += " | truck_route=";
  str += std::to_string(truck_route());

  return str;
}

std::string EnhancedTripPath_Edge::ToParameterString() const {
  const std::string delim = ", ";
  std::string str;
  str.reserve(128);

  str += StreetNamesToParameterString(this->name());

  str += delim;
  str += std::to_string(length());

  str += delim;
  str += std::to_string(speed());

  str += delim;
  str += "TripPath_RoadClass_";
  str += TripPath_RoadClass_Name(road_class());

  str += delim;
  str += std::to_string(begin_heading());

  str += delim;
  str += std::to_string(end_heading());

  str += delim;
  str += std::to_string(begin_shape_index());

  str += delim;
  str += std::to_string(end_shape_index());

  str += delim;
  str += "TripPath_Traversability_";
  str += TripPath_Traversability_Name(traversability());

  str += delim;
  str += "TripPath_Use_";
  str += TripPath_Use_Name(use());

  str += delim;
  str += std::to_string(toll());

  str += delim;
  str += std::to_string(unpaved());

  str += delim;
  str += std::to_string(tunnel());

  str += delim;
  str += std::to_string(bridge());

  str += delim;
  str += std::to_string(roundabout());

  str += delim;
  str += std::to_string(internal_intersection());

  str += delim;
  str += SignElementsToParameterString(this->sign().exit_numbers());

  str += delim;
  str += SignElementsToParameterString(this->sign().exit_onto_streets());

  str += delim;
  str += SignElementsToParameterString(this->sign().exit_toward_locations());

  str += delim;
  str += SignElementsToParameterString(this->sign().exit_names());

  str += delim;
  if (this->has_travel_mode()) {
    str += "TripPath_TravelMode_";
    str += TripPath_TravelMode_Name(travel_mode());
  }

  // NOTE: Current PopulateEdge implementation

  str += delim;
  if (this->has_vehicle_type()) {
    str += "TripPath_VehicleType_";
    str += TripPath_VehicleType_Name(vehicle_type());
  }

  str += delim;
  if (this->has_pedestrian_type()) {
    str += "TripPath_PedestrianType_";
    str += TripPath_PedestrianType_Name(pedestrian_type());
  }

  str += delim;
  if (this->has_bicycle_type()) {
    str += "TripPath_BicycleType_";
    str += TripPath_BicycleType_Name(bicycle_type());
  }

  str += delim;
  if (this->has_transit_type()) {
    str += "TripPath_TransitType_";
    str += TripPath_TransitType_Name(transit_type());
  }

  str += delim;
  str += std::to_string(drive_on_right());

  str += delim;
  str += std::to_string(surface());

  str += delim;
  if (transit_route_info().has_onestop_id()) {
    str += "\"";
    str += transit_route_info().onestop_id();
    str += "\"";
  }

  str += delim;
  str += std::to_string(transit_route_info().block_id());

  str += delim;
  str += std::to_string(transit_route_info().trip_id());

  str += delim;
  if (transit_route_info().has_short_name()) {
    str += "\"";
    str += transit_route_info().short_name();
    str += "\"";
  }

  str += delim;
  if (transit_route_info().has_long_name()) {
    str += "\"";
    str += transit_route_info().long_name();
    str += "\"";
  }

  str += delim;
  if (transit_route_info().has_headsign()) {
    str += "\"";
    str += transit_route_info().headsign();
    str += "\"";
  }

  str += delim;
  str += std::to_string(transit_route_info().color());

  str += delim;
  str += std::to_string(transit_route_info().text_color());

  str += delim;
  if (transit_route_info().has_operator_onestop_id()) {
    str += "\"";
    str += transit_route_info().operator_onestop_id();
    str += "\"";
  }

  str += delim;
  str += std::to_string(id());

  str += delim;
  str += std::to_string(way_id());

  str += delim;
  str += std::to_string(weighted_grade());

  str += delim;
  str += std::to_string(max_upward_grade());

  str += delim;
  str += std::to_string(max_downward_grade());

  str += delim;
  str += std::to_string(lane_count());

  str += delim;
  str += "TripPath_CycleLane_";
  str += TripPath_CycleLane_Name(cycle_lane());

  str += delim;
  str += std::to_string(bicycle_network());

  str += delim;
  str += "TripPath_Sidewalk_";
  str += TripPath_Sidewalk_Name(sidewalk());

  str += delim;
  str += std::to_string(density());

  str += delim;
  str += std::to_string(speed_limit());

  str += delim;
  str += std::to_string(truck_speed());

  str += delim;
  str += std::to_string(truck_route());

  return str;
}

std::string EnhancedTripPath_Edge::StreetNamesToString(
    const ::google::protobuf::RepeatedPtrField<::valhalla::odin::StreetName>& street_names) const {
  std::string str;

  for (const auto& street_name : street_names) {
    if (!str.empty()) {
      str += "/";
    }
    str += street_name.value();
  }
  return str;
}

std::string EnhancedTripPath_Edge::StreetNamesToParameterString(
    const ::google::protobuf::RepeatedPtrField<::valhalla::odin::StreetName>& street_names) const {
  std::string str;
  std::string param_list;

  // FORMAT: { {"I 83 South",1}, {"Jones Expressway",0} }
  str += "{ ";
  for (const auto& street_name : street_names) {
    if (!param_list.empty()) {
      param_list += ", ";
    }
    param_list += "{ \"";
    param_list += street_name.value();
    param_list += "\", ";
    param_list += std::to_string(street_name.is_route_number());
    param_list += " }";
  }
  str += param_list;
  str += " }";

  return str;
}

std::string EnhancedTripPath_Edge::SignElementsToString(
    const ::google::protobuf::RepeatedPtrField<::valhalla::odin::TripPath_SignElement>& sign_elements)
    const {
  std::string str;

  for (const auto& sign_element : sign_elements) {
    if (!str.empty()) {
      str += "/";
    }
    str += sign_element.text();
  }
  return str;
}

std::string EnhancedTripPath_Edge::SignElementsToParameterString(
    const ::google::protobuf::RepeatedPtrField<::valhalla::odin::TripPath_SignElement>& sign_elements)
    const {
  std::string str;
  std::string param_list;

  // FORMAT: { {"I 83 South",1}, {"Jones Expressway",0} }
  str += "{ ";
  for (const auto& sign_element : sign_elements) {
    if (!param_list.empty()) {
      param_list += ", ";
    }
    param_list += "{ \"";
    param_list += sign_element.text();
    param_list += "\", ";
    param_list += std::to_string(sign_element.is_route_number());
    param_list += " }";
  }
  str += param_list;
  str += " }";

  return str;
}
#endif

///////////////////////////////////////////////////////////////////////////////
// EnhancedTripPath_IntersectingEdge

bool EnhancedTripPath_IntersectingEdge::IsTraversable(const TripPath_TravelMode travel_mode) const {
  TripPath_Traversability t;

  // Set traversability based on travel mode
  if (travel_mode == TripPath_TravelMode_kDrive) {
    t = driveability();
  } else if (travel_mode == TripPath_TravelMode_kBicycle) {
    t = cyclability();
  } else {
    t = walkability();
  }

  if (t != TripPath_Traversability_kNone) {
    return true;
  }
  return false;
}

bool EnhancedTripPath_IntersectingEdge::IsTraversableOutbound(
    const TripPath_TravelMode travel_mode) const {
  TripPath_Traversability t;

  // Set traversability based on travel mode
  if (travel_mode == TripPath_TravelMode_kDrive) {
    t = driveability();
  } else if (travel_mode == TripPath_TravelMode_kBicycle) {
    t = cyclability();
  } else {
    t = walkability();
  }

  if ((t == TripPath_Traversability_kForward) || (t == TripPath_Traversability_kBoth)) {
    return true;
  }
  return false;
}

std::string EnhancedTripPath_IntersectingEdge::ToString() const {
  std::string str;
  str.reserve(128);

  str += "begin_heading=";
  str += std::to_string(begin_heading());

  str += " | prev_name_consistency=";
  str += std::to_string(prev_name_consistency());

  str += " | curr_name_consistency=";
  str += std::to_string(curr_name_consistency());

  str += " | driveability=";
  str += std::to_string(driveability());

  str += " | cyclability=";
  str += std::to_string(cyclability());

  str += " | walkability=";
  str += std::to_string(walkability());

  return str;
}

///////////////////////////////////////////////////////////////////////////////
// EnhancedTripPath_Node

bool EnhancedTripPath_Node::HasIntersectingEdges() const {
  return (intersecting_edge_size() > 0);
}

bool EnhancedTripPath_Node::HasIntersectingEdgeNameConsistency() const {
  for (const auto& xedge : intersecting_edge()) {
    if (xedge.curr_name_consistency() || xedge.prev_name_consistency()) {
      return true;
    }
  }
  return false;
}

EnhancedTripPath_IntersectingEdge* EnhancedTripPath_Node::GetIntersectingEdge(size_t index) {
  return static_cast<EnhancedTripPath_IntersectingEdge*>(mutable_intersecting_edge(index));
}

void EnhancedTripPath_Node::CalculateRightLeftIntersectingEdgeCounts(
    uint32_t from_heading,
    const TripPath_TravelMode travel_mode,
    IntersectingEdgeCounts& xedge_counts) {
  xedge_counts.clear();

  // No turn - just return
  if (intersecting_edge_size() == 0) {
    return;
  }

  uint32_t path_turn_degree = GetTurnDegree(from_heading, edge().begin_heading());
  for (int i = 0; i < intersecting_edge_size(); ++i) {
    uint32_t intersecting_turn_degree =
        GetTurnDegree(from_heading, intersecting_edge(i).begin_heading());
    bool xedge_traversable_outbound = GetIntersectingEdge(i)->IsTraversableOutbound(travel_mode);

    if (path_turn_degree > 180) {
      if ((intersecting_turn_degree > path_turn_degree) || (intersecting_turn_degree < 180)) {
        ++xedge_counts.right;
        if (IsSimilarTurnDegree(path_turn_degree, intersecting_turn_degree, true)) {
          ++xedge_counts.right_similar;
          if (xedge_traversable_outbound) {
            ++xedge_counts.right_similar_traversable_outbound;
          }
        }
        if (xedge_traversable_outbound) {
          ++xedge_counts.right_traversable_outbound;
        }
      } else if ((intersecting_turn_degree < path_turn_degree) && (intersecting_turn_degree > 180)) {
        ++xedge_counts.left;
        if (IsSimilarTurnDegree(path_turn_degree, intersecting_turn_degree, false)) {
          ++xedge_counts.left_similar;
          if (xedge_traversable_outbound) {
            ++xedge_counts.left_similar_traversable_outbound;
          }
        }
        if (xedge_traversable_outbound) {
          ++xedge_counts.left_traversable_outbound;
        }
      }
    } else {
      if ((intersecting_turn_degree > path_turn_degree) && (intersecting_turn_degree < 180)) {
        ++xedge_counts.right;
        if (IsSimilarTurnDegree(path_turn_degree, intersecting_turn_degree, true)) {
          ++xedge_counts.right_similar;
          if (xedge_traversable_outbound) {
            ++xedge_counts.right_similar_traversable_outbound;
          }
        }
        if (xedge_traversable_outbound) {
          ++xedge_counts.right_traversable_outbound;
        }
      } else if ((intersecting_turn_degree < path_turn_degree) || (intersecting_turn_degree > 180)) {
        ++xedge_counts.left;
        if (IsSimilarTurnDegree(path_turn_degree, intersecting_turn_degree, false)) {
          ++xedge_counts.left_similar;
          if (xedge_traversable_outbound) {
            ++xedge_counts.left_similar_traversable_outbound;
          }
        }
        if (xedge_traversable_outbound) {
          ++xedge_counts.left_traversable_outbound;
        }
      }
    }
  }
}

bool EnhancedTripPath_Node::HasFowardIntersectingEdge(uint32_t from_heading) {

  for (int i = 0; i < intersecting_edge_size(); ++i) {
    uint32_t intersecting_turn_degree =
        GetTurnDegree(from_heading, intersecting_edge(i).begin_heading());
    if ((intersecting_turn_degree > 314) || (intersecting_turn_degree < 46)) {
      return true;
    }
  }
  return false;
}

bool EnhancedTripPath_Node::HasForwardTraversableIntersectingEdge(
    uint32_t from_heading,
    const TripPath_TravelMode travel_mode) {

  for (int i = 0; i < intersecting_edge_size(); ++i) {
    uint32_t intersecting_turn_degree =
        GetTurnDegree(from_heading, intersecting_edge(i).begin_heading());
    bool xedge_traversable_outbound = GetIntersectingEdge(i)->IsTraversableOutbound(travel_mode);
    if (((intersecting_turn_degree > 314) || (intersecting_turn_degree < 46)) &&
        xedge_traversable_outbound) {
      return true;
    }
  }
  return false;
}

bool EnhancedTripPath_Node::HasTraversableOutboundIntersectingEdge(
    const TripPath_TravelMode travel_mode) {

  for (int i = 0; i < intersecting_edge_size(); ++i) {
    if (GetIntersectingEdge(i)->IsTraversableOutbound(travel_mode)) {
      return true;
    }
  }
  return false;
}

// TODO: refactor to clean up code
uint32_t EnhancedTripPath_Node::GetStraightestTraversableIntersectingEdgeTurnDegree(
    uint32_t from_heading,
    const TripPath_TravelMode travel_mode) {

  uint32_t staightest_turn_degree = 180; // Initialize to reverse turn degree
  uint32_t staightest_delta = 180;       // Initialize to reverse delta

  for (int i = 0; i < intersecting_edge_size(); ++i) {
    uint32_t intersecting_turn_degree =
        GetTurnDegree(from_heading, intersecting_edge(i).begin_heading());
    bool xedge_traversable_outbound = GetIntersectingEdge(i)->IsTraversableOutbound(travel_mode);
    uint32_t straight_delta = (intersecting_turn_degree > 180) ? (360 - intersecting_turn_degree)
                                                               : intersecting_turn_degree;
    if (xedge_traversable_outbound && (straight_delta < staightest_delta)) {
      staightest_delta = straight_delta;
      staightest_turn_degree = intersecting_turn_degree;
    }
  }
  return staightest_turn_degree;
}

uint32_t EnhancedTripPath_Node::GetStraightestIntersectingEdgeTurnDegree(uint32_t from_heading) {

  uint32_t staightest_turn_degree = 180; // Initialize to reverse turn degree
  uint32_t staightest_delta = 180;       // Initialize to reverse delta

  for (int i = 0; i < intersecting_edge_size(); ++i) {
    uint32_t intersecting_turn_degree =
        GetTurnDegree(from_heading, intersecting_edge(i).begin_heading());
    uint32_t straight_delta = (intersecting_turn_degree > 180) ? (360 - intersecting_turn_degree)
                                                               : intersecting_turn_degree;
    if (straight_delta < staightest_delta) {
      staightest_delta = straight_delta;
      staightest_turn_degree = intersecting_turn_degree;
    }
  }
  return staightest_turn_degree;
}

bool EnhancedTripPath_Node::IsStreetIntersection() const {
  return (type() == TripPath_Node_Type_kStreetIntersection);
}

bool EnhancedTripPath_Node::IsGate() const {
  return (type() == TripPath_Node_Type_kGate);
}

bool EnhancedTripPath_Node::IsBollard() const {
  return (type() == TripPath_Node_Type_kBollard);
}

bool EnhancedTripPath_Node::IsTollBooth() const {
  return (type() == TripPath_Node_Type_kTollBooth);
}

bool EnhancedTripPath_Node::IsTransitEgress() const {
  return (type() == TripPath_Node_Type_kTransitEgress);
}

bool EnhancedTripPath_Node::IsTransitStation() const {
  return (type() == TripPath_Node_Type_kTransitStation);
}

bool EnhancedTripPath_Node::IsTransitPlatform() const {
  return (type() == TripPath_Node_Type_kTransitPlatform);
}

bool EnhancedTripPath_Node::IsBikeShare() const {
  return (type() == TripPath_Node_Type_kBikeShare);
}

bool EnhancedTripPath_Node::IsParking() const {
  return (type() == TripPath_Node_Type_kParking);
}

bool EnhancedTripPath_Node::IsMotorwayJunction() const {
  return (type() == TripPath_Node_Type_kMotorwayJunction);
}

bool EnhancedTripPath_Node::IsBorderControl() const {
  return (type() == TripPath_Node_Type_kBorderControl);
}

std::string EnhancedTripPath_Node::ToString() const {
  std::string str;
  str.reserve(256);

  str += "elapsed_time=";
  str += std::to_string(elapsed_time());

  str += " | admin_index=";
  str += std::to_string(admin_index());

  str += " | type=";
  str += std::to_string(type());

  str += " | fork=";
  str += std::to_string(fork());

  if (has_transit_platform_info()) {
    str += " | transit_platform_info.type=";
    str += std::to_string(transit_platform_info().type());

    str += " | transit_platform_info.onestop_id=";
    str += transit_platform_info().onestop_id();

    str += " | transit_platform_info.name=";
    str += transit_platform_info().name();

    str += " | transit_platform_info.arrival_date_time=";
    str += transit_platform_info().arrival_date_time();

    str += " | transit_platform_info.departure_date_time=";
    str += transit_platform_info().departure_date_time();

    str += " | transit_platform_info.assumed_schedule()=";
    str += std::to_string(transit_platform_info().assumed_schedule());

    str += " | transit_platform_info.station_onestop_id=";
    str += transit_platform_info().station_onestop_id();

    str += " | transit_platform_info.station_name=";
    str += transit_platform_info().station_name();
  }

  str += " | time_zone=";
  str += time_zone();

  return str;
}

///////////////////////////////////////////////////////////////////////////////
// EnhancedTripPath_Admin

std::string EnhancedTripPath_Admin::ToString() const {
  std::string str;
  str.reserve(256);

  str += "country_code=";
  str += country_code();

  str += " | country_text=";
  str += country_text();

  str += " | state_code=";
  str += state_code();

  str += " | state_text=";
  str += state_text();

  return str;
}

} // namespace odin
} // namespace valhalla
