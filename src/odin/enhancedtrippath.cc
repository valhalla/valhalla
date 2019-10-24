#include <cmath>
#include <cstdlib>
#include <iostream>

#include "baldr/turn.h"
#include "baldr/turnlanes.h"
#include "midgard/constants.h"
#include "midgard/util.h"

#include "worker.h"

#include "odin/enhancedtrippath.h"
#include "odin/util.h"

#include <valhalla/proto/trip.pb.h>

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace {
constexpr float kShortRemainingDistanceThreshold = 0.402f; // Kilometers (~quarter mile)
constexpr int kSignificantRoadClassThreshold = 2;          // Max lower road class delta

const std::string& TripLeg_RoadClass_Name(int v) {
  static const std::unordered_map<int, std::string> values{
      {0, "kMotorway"}, {1, "kTrunk"},        {2, "kPrimary"},     {3, "kSecondary"},
      {4, "kTertiary"}, {5, "kUnclassified"}, {6, "kResidential"}, {7, "kServiceOther"},
  };
  auto f = values.find(v);
  if (f == values.cend())
    throw std::runtime_error("Missing value in protobuf enum to string");
  return f->second;
}

const std::string& TripLeg_Traversability_Name(int v) {
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

const std::string& TripLeg_Use_Name(int v) {
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

const std::string& TripLeg_TravelMode_Name(int v) {
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

const std::string& TripLeg_VehicleType_Name(int v) {
  static const std::unordered_map<int, std::string> values{
      {0, "kCar"}, {1, "kMotorcycle"}, {2, "kAutoBus"}, {3, "kTractorTrailer"}, {4, "kMotorScooter"},
  };
  auto f = values.find(v);
  if (f == values.cend())
    throw std::runtime_error("Missing value in protobuf enum to string");
  return f->second;
}

const std::string& TripLeg_PedestrianType_Name(int v) {
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

const std::string& TripLeg_BicycleType_Name(int v) {
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

const std::string& TripLeg_TransitType_Name(int v) {
  static const std::unordered_map<int, std::string> values{
      {0, "kTram"},  {1, "kMetro"},    {2, "kRail"},    {3, "kBus"},
      {4, "kFerry"}, {5, "kCableCar"}, {6, "kGondola"}, {7, "kFunicular"},
  };
  auto f = values.find(v);
  if (f == values.cend())
    throw std::runtime_error("Missing value in protobuf enum to string");
  return f->second;
}

const std::string& TripLeg_CycleLane_Name(int v) {
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

const std::string& TripLeg_Sidewalk_Name(int v) {
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

bool is_forward(uint32_t turn_degree) {
  return ((turn_degree > 314) || (turn_degree < 46));
}

bool is_wider_forward(uint32_t turn_degree) {
  return ((turn_degree > 304) || (turn_degree < 56));
}

} // namespace

namespace valhalla {
namespace odin {

///////////////////////////////////////////////////////////////////////////////
// EnhancedTripLeg

EnhancedTripLeg::EnhancedTripLeg(TripLeg& trip_path) : trip_path_(trip_path) {
}

std::unique_ptr<EnhancedTripLeg_Node> EnhancedTripLeg::GetEnhancedNode(const int node_index) {
  return std::make_unique<EnhancedTripLeg_Node>(mutable_node(node_index));
}

std::unique_ptr<EnhancedTripLeg_Edge> EnhancedTripLeg::GetPrevEdge(const int node_index, int delta) {
  int index = node_index - delta;
  if (IsValidNodeIndex(index)) {
    return std::make_unique<EnhancedTripLeg_Edge>(mutable_node(index)->mutable_edge());
  } else {
    return nullptr;
  }
}

std::unique_ptr<EnhancedTripLeg_Edge> EnhancedTripLeg::GetCurrEdge(const int node_index) const {
  return GetNextEdge(node_index, 0);
}

std::unique_ptr<EnhancedTripLeg_Edge> EnhancedTripLeg::GetNextEdge(const int node_index,
                                                                   int delta) const {
  int index = node_index + delta;
  if (IsValidNodeIndex(index) && !IsLastNodeIndex(index)) {
    return std::make_unique<EnhancedTripLeg_Edge>(mutable_node(index)->mutable_edge());
  } else {
    return nullptr;
  }
}

bool EnhancedTripLeg::IsValidNodeIndex(int node_index) const {
  if ((node_index >= 0) && (node_index < node_size())) {
    return true;
  }
  return false;
}

bool EnhancedTripLeg::IsFirstNodeIndex(int node_index) const {
  if (node_index == 0) {
    return true;
  }
  return false;
}

bool EnhancedTripLeg::IsLastNodeIndex(int node_index) const {
  if (IsValidNodeIndex(node_index) && (node_index == (node_size() - 1))) {
    return true;
  }
  return false;
}

int EnhancedTripLeg::GetLastNodeIndex() const {
  return (node_size() - 1);
}

std::unique_ptr<EnhancedTripLeg_Admin> EnhancedTripLeg::GetAdmin(size_t index) {
  return std::make_unique<EnhancedTripLeg_Admin>(mutable_admin(index));
}

std::string EnhancedTripLeg::GetCountryCode(int node_index) {
  return GetAdmin(node(node_index).admin_index())->country_code();
}

std::string EnhancedTripLeg::GetStateCode(int node_index) {
  return GetAdmin(node(node_index).admin_index())->state_code();
}

const ::valhalla::Location& EnhancedTripLeg::GetOrigin() const {
  // Validate location count
  if (location_size() < 2) {
    throw valhalla_exception_t{212};
  }

  return location(0);
}

const ::valhalla::Location& EnhancedTripLeg::GetDestination() const {
  // Validate location count
  if (location_size() < 2) {
    throw valhalla_exception_t{212};
  }

  return location(location_size() - 1);
}

float EnhancedTripLeg::GetLength(const Options::Units& units) {
  float length = 0.0f;
  for (const auto& n : node()) {
    if (n.has_edge()) {
      length += n.edge().length();
    }
  }
  if (units == Options::miles) {
    return (length * kMilePerKm);
  }
  return length;
}

///////////////////////////////////////////////////////////////////////////////
// EnhancedTripLeg_Edge

EnhancedTripLeg_Edge::EnhancedTripLeg_Edge(TripLeg_Edge* mutable_edge) : mutable_edge_(mutable_edge) {
}

bool EnhancedTripLeg_Edge::IsUnnamed() const {
  return (name_size() == 0);
}

bool EnhancedTripLeg_Edge::IsRoadUse() const {
  return (use() == TripLeg_Use_kRoadUse);
}

bool EnhancedTripLeg_Edge::IsRampUse() const {
  return (use() == TripLeg_Use_kRampUse);
}

bool EnhancedTripLeg_Edge::IsTurnChannelUse() const {
  return (use() == TripLeg_Use_kTurnChannelUse);
}

bool EnhancedTripLeg_Edge::IsTrackUse() const {
  return (use() == TripLeg_Use_kTrackUse);
}

bool EnhancedTripLeg_Edge::IsDrivewayUse() const {
  return (use() == TripLeg_Use_kDrivewayUse);
}

bool EnhancedTripLeg_Edge::IsAlleyUse() const {
  return (use() == TripLeg_Use_kAlleyUse);
}

bool EnhancedTripLeg_Edge::IsParkingAisleUse() const {
  return (use() == TripLeg_Use_kParkingAisleUse);
}

bool EnhancedTripLeg_Edge::IsEmergencyAccessUse() const {
  return (use() == TripLeg_Use_kEmergencyAccessUse);
}

bool EnhancedTripLeg_Edge::IsDriveThruUse() const {
  return (use() == TripLeg_Use_kDriveThruUse);
}

bool EnhancedTripLeg_Edge::IsCuldesacUse() const {
  return (use() == TripLeg_Use_kCuldesacUse);
}

bool EnhancedTripLeg_Edge::IsCyclewayUse() const {
  return (use() == TripLeg_Use_kCyclewayUse);
}

bool EnhancedTripLeg_Edge::IsMountainBikeUse() const {
  return (use() == TripLeg_Use_kMountainBikeUse);
}

bool EnhancedTripLeg_Edge::IsSidewalkUse() const {
  return (use() == TripLeg_Use_kSidewalkUse);
}

bool EnhancedTripLeg_Edge::IsFootwayUse() const {
  return (use() == TripLeg_Use_kFootwayUse);
}

bool EnhancedTripLeg_Edge::IsStepsUse() const {
  return (use() == TripLeg_Use_kStepsUse);
}

bool EnhancedTripLeg_Edge::IsPathUse() const {
  return (use() == TripLeg_Use_kPathUse);
}

bool EnhancedTripLeg_Edge::IsPedestrianUse() const {
  return (use() == TripLeg_Use_kPedestrianUse);
}

bool EnhancedTripLeg_Edge::IsBridlewayUse() const {
  return (use() == TripLeg_Use_kBridlewayUse);
}

bool EnhancedTripLeg_Edge::IsOtherUse() const {
  return (use() == TripLeg_Use_kOtherUse);
}

bool EnhancedTripLeg_Edge::IsFerryUse() const {
  return (use() == TripLeg_Use_kFerryUse);
}

bool EnhancedTripLeg_Edge::IsRailFerryUse() const {
  return (use() == TripLeg_Use_kRailFerryUse);
}

bool EnhancedTripLeg_Edge::IsRailUse() const {
  return (use() == TripLeg_Use_kRailUse);
}

bool EnhancedTripLeg_Edge::IsBusUse() const {
  return (use() == TripLeg_Use_kBusUse);
}

bool EnhancedTripLeg_Edge::IsEgressConnectionUse() const {
  return (use() == TripLeg_Use_kEgressConnectionUse);
}

bool EnhancedTripLeg_Edge::IsPlatformConnectionUse() const {
  return (use() == TripLeg_Use_kPlatformConnectionUse);
}

bool EnhancedTripLeg_Edge::IsTransitConnectionUse() const {
  return (use() == TripLeg_Use_kTransitConnectionUse);
}

bool EnhancedTripLeg_Edge::IsTransitConnection() const {
  return (IsTransitConnectionUse() || IsEgressConnectionUse() || IsPlatformConnectionUse());
}

bool EnhancedTripLeg_Edge::IsUnnamedWalkway() const {
  return (IsUnnamed() && IsFootwayUse());
}

bool EnhancedTripLeg_Edge::IsUnnamedCycleway() const {
  return (IsUnnamed() && IsCyclewayUse());
}

bool EnhancedTripLeg_Edge::IsUnnamedMountainBikeTrail() const {
  return (IsUnnamed() && IsMountainBikeUse());
}

bool EnhancedTripLeg_Edge::IsHighway() const {
  return ((road_class() == TripLeg_RoadClass_kMotorway) && (!IsRampUse()));
}

bool EnhancedTripLeg_Edge::IsOneway() const {
  return ((traversability() == TripLeg_Traversability_kForward) ||
          (traversability() == TripLeg_Traversability_kBackward));
}

bool EnhancedTripLeg_Edge::IsForward(uint32_t prev2curr_turn_degree) const {
  return is_forward(prev2curr_turn_degree);
}

bool EnhancedTripLeg_Edge::IsWiderForward(uint32_t prev2curr_turn_degree) const {
  return is_wider_forward(prev2curr_turn_degree);
}

bool EnhancedTripLeg_Edge::IsStraightest(uint32_t prev2curr_turn_degree,
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

std::vector<std::pair<std::string, bool>> EnhancedTripLeg_Edge::GetNameList() const {
  std::vector<std::pair<std::string, bool>> name_list;
  for (const auto& name : this->name()) {
    name_list.push_back({name.value(), name.is_route_number()});
  }
  return name_list;
}

float EnhancedTripLeg_Edge::GetLength(const Options::Units& units) {
  if (units == Options::miles) {
    return (length() * kMilePerKm);
  }
  return length();
}

bool EnhancedTripLeg_Edge::HasActiveTurnLane() const {
  for (const auto& turn_lane : turn_lanes()) {
    if (turn_lane.is_active()) {
      return true;
    }
  }
  return false;
}

bool EnhancedTripLeg_Edge::HasNonDirectionalTurnLane() const {
  for (const auto& turn_lane : turn_lanes()) {
    // Return true if a directions mask is empty or none for a turn lane
    if ((turn_lane.directions_mask() == kTurnLaneEmpty) ||
        (turn_lane.directions_mask() & kTurnLaneNone)) {
      return true;
    }
  }
  return false;
}

bool EnhancedTripLeg_Edge::HasTurnLane(uint16_t turn_lane_direction) const {
  for (const auto& turn_lane : turn_lanes()) {
    // Return true if the specified turn lane exists
    if (turn_lane.directions_mask() & turn_lane_direction) {
      return true;
    }
  }
  return false;
}

uint16_t EnhancedTripLeg_Edge::ActivateTurnLanesFromLeft(uint16_t turn_lane_direction,
                                                         uint16_t activated_max) {
  uint16_t activated_count = 0;
  // Make sure turn lane has a direction
  if (!HasNonDirectionalTurnLane()) {
    for (auto& turn_lane : *(mutable_turn_lanes())) {
      // Stop processing the lanes if the activated maximum has been reached
      if (activated_count >= activated_max) {
        break;
      }

      // If the turn lane is in the specified direction then activate the lane
      // and increment the activated count
      if (turn_lane.directions_mask() & turn_lane_direction) {
        turn_lane.set_is_active(true);
        ++activated_count;
      }
    }
  }
  return activated_count;
}

uint16_t EnhancedTripLeg_Edge::ActivateTurnLanesFromRight(uint16_t turn_lane_direction,
                                                          uint16_t activated_max) {
  uint16_t activated_count = 0;
  // Make sure turn lane has a direction
  if (!HasNonDirectionalTurnLane()) {
    for (auto turn_lane_iter = mutable_turn_lanes()->rbegin();
         turn_lane_iter != mutable_turn_lanes()->rend(); ++turn_lane_iter) {
      //    for (auto& turn_lane : *(mutable_turn_lanes())) {
      // Stop processing the lanes if the activated maximum has been reached
      if (activated_count >= activated_max) {
        break;
      }

      // If the turn lane is in the specified direction then activate the lane
      // and increment the activated count
      if (turn_lane_iter->directions_mask() & turn_lane_direction) {
        turn_lane_iter->set_is_active(true);
        ++activated_count;
      }
    }
  }
  return activated_count;
}

uint16_t
EnhancedTripLeg_Edge::ActivateTurnLanes(uint16_t turn_lane_direction,
                                        float remaining_step_distance,
                                        const DirectionsLeg_Maneuver_Type& curr_maneuver_type,
                                        const DirectionsLeg_Maneuver_Type& next_maneuver_type) {
  if ((curr_maneuver_type == DirectionsLeg_Maneuver_Type_kUturnLeft) &&
      (turn_lane_direction != kTurnLaneReverse)) {
    // Activate the left most turn lane
    return ActivateTurnLanesFromLeft(turn_lane_direction, 1);
  } else if ((curr_maneuver_type == DirectionsLeg_Maneuver_Type_kUturnRight) &&
             (turn_lane_direction != kTurnLaneReverse)) {
    // Activate the right most turn lane
    return ActivateTurnLanesFromRight(turn_lane_direction, 1);
  } else if ((remaining_step_distance < kShortRemainingDistanceThreshold) &&
             !((next_maneuver_type == DirectionsLeg_Maneuver_Type_kBecomes) ||
               (next_maneuver_type == DirectionsLeg_Maneuver_Type_kContinue) ||
               (next_maneuver_type == DirectionsLeg_Maneuver_Type_kRampStraight) ||
               (next_maneuver_type == DirectionsLeg_Maneuver_Type_kStayStraight))) {
    // If remaining step distance is less than short threshold
    // and next maneuver is not a straight
    // Activate only specific matching turn lanes
    switch (next_maneuver_type) {
      case DirectionsLeg_Maneuver_Type_kUturnLeft:
      case DirectionsLeg_Maneuver_Type_kSharpLeft:
      case DirectionsLeg_Maneuver_Type_kLeft:
      case DirectionsLeg_Maneuver_Type_kSlightLeft:
      case DirectionsLeg_Maneuver_Type_kExitLeft:
      case DirectionsLeg_Maneuver_Type_kRampLeft:
      case DirectionsLeg_Maneuver_Type_kDestinationLeft:
      case DirectionsLeg_Maneuver_Type_kMergeLeft:
        return ActivateTurnLanesFromLeft(turn_lane_direction, 1);
      case DirectionsLeg_Maneuver_Type_kSlightRight:
      case DirectionsLeg_Maneuver_Type_kExitRight:
      case DirectionsLeg_Maneuver_Type_kRampRight:
      case DirectionsLeg_Maneuver_Type_kRight:
      case DirectionsLeg_Maneuver_Type_kSharpRight:
      case DirectionsLeg_Maneuver_Type_kUturnRight:
      case DirectionsLeg_Maneuver_Type_kDestinationRight:
      case DirectionsLeg_Maneuver_Type_kMergeRight:
        return ActivateTurnLanesFromRight(turn_lane_direction, 1);
      case DirectionsLeg_Maneuver_Type_kMerge:
        if (drive_on_right()) {
          return ActivateTurnLanesFromLeft(turn_lane_direction, 1);
        } else {
          return ActivateTurnLanesFromRight(turn_lane_direction, 1);
        }
      case DirectionsLeg_Maneuver_Type_kRoundaboutEnter:
      case DirectionsLeg_Maneuver_Type_kRoundaboutExit:
      case DirectionsLeg_Maneuver_Type_kFerryEnter:
      case DirectionsLeg_Maneuver_Type_kFerryExit:
        return ActivateTurnLanesFromLeft(turn_lane_direction);
      case DirectionsLeg_Maneuver_Type_kDestination:
        if (drive_on_right()) {
          return ActivateTurnLanesFromRight(turn_lane_direction, 1);
        } else {
          return ActivateTurnLanesFromLeft(turn_lane_direction, 1);
        }
      default:
        return ActivateTurnLanesFromLeft(turn_lane_direction);
    }
  } else {
    // Activate all matching turn lanes
    return ActivateTurnLanesFromLeft(turn_lane_direction);
  }
}

std::string EnhancedTripLeg_Edge::ToString() const {
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

  if (turn_lanes_size() > 0) {
    str += " | turn_lanes=";
    str += TurnLanesToString();
  }

  return str;
}

std::string EnhancedTripLeg_Edge::TurnLanesToString() const {
  std::string str;

  for (const auto& turn_lane : turn_lanes()) {
    if (str.empty()) {
      str = "[ ";
    } else {
      str += " | ";
    }

    uint16_t mask = turn_lane.directions_mask();

    // Process the turn lanes - from left to right
    // empty
    if (mask == kTurnLaneEmpty) {
      str += "empty";
    }
    // none
    else if (mask == kTurnLaneNone) {
      str += kTurnLaneNames.at(kTurnLaneNone);
    } else {
      bool prior_item = false;
      // reverse (left u-turn)
      if ((mask & kTurnLaneReverse) && drive_on_right()) {
        if (prior_item)
          str += ";";
        str += kTurnLaneNames.at(kTurnLaneReverse);
        prior_item = true;
      }
      // sharp_left
      if (mask & kTurnLaneSharpLeft) {
        if (prior_item)
          str += ";";
        str += kTurnLaneNames.at(kTurnLaneSharpLeft);
        prior_item = true;
      }
      // left
      if (mask & kTurnLaneLeft) {
        if (prior_item)
          str += ";";
        str += kTurnLaneNames.at(kTurnLaneLeft);
        prior_item = true;
      }
      // slight_left
      if (mask & kTurnLaneSlightLeft) {
        if (prior_item)
          str += ";";
        str += kTurnLaneNames.at(kTurnLaneSlightLeft);
        prior_item = true;
      }
      // merge_to_left
      if (mask & kTurnLaneMergeToLeft) {
        if (prior_item)
          str += ";";
        str += kTurnLaneNames.at(kTurnLaneMergeToLeft);
        prior_item = true;
      }
      // through
      if (mask & kTurnLaneThrough) {
        if (prior_item)
          str += ";";
        str += kTurnLaneNames.at(kTurnLaneThrough);
        prior_item = true;
      }
      // merge_to_right
      if (mask & kTurnLaneMergeToRight) {
        if (prior_item)
          str += ";";
        str += kTurnLaneNames.at(kTurnLaneMergeToRight);
        prior_item = true;
      }
      // slight_right
      if (mask & kTurnLaneSlightRight) {
        if (prior_item)
          str += ";";
        str += kTurnLaneNames.at(kTurnLaneSlightRight);
        prior_item = true;
      }
      // right
      if (mask & kTurnLaneRight) {
        if (prior_item)
          str += ";";
        str += kTurnLaneNames.at(kTurnLaneRight);
        prior_item = true;
      }
      // sharp_right
      if (mask & kTurnLaneSharpRight) {
        if (prior_item)
          str += ";";
        str += kTurnLaneNames.at(kTurnLaneSharpRight);
        prior_item = true;
      }
      // reverse (right u-turn)
      if ((mask & kTurnLaneReverse) && !drive_on_right()) {
        if (prior_item)
          str += ";";
        str += kTurnLaneNames.at(kTurnLaneReverse);
        prior_item = true;
      }
    }

    // Output if marked as active
    if (turn_lane.is_active()) {
      str += " ACTIVE";
    }
  }
  str += " ]";
  return str;
}

std::string EnhancedTripLeg_Edge::StreetNamesToString(
    const ::google::protobuf::RepeatedPtrField<::valhalla::StreetName>& street_names) const {
  std::string str;

  for (const auto& street_name : street_names) {
    if (!str.empty()) {
      str += "/";
    }
    str += street_name.value();
  }
  return str;
}

std::string EnhancedTripLeg_Edge::SignElementsToString(
    const ::google::protobuf::RepeatedPtrField<::valhalla::TripLeg_SignElement>& sign_elements)
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

#ifdef LOGGING_LEVEL_TRACE
std::string EnhancedTripLeg_Edge::ToParameterString() const {
  const std::string delim = ", ";
  std::string str;
  str.reserve(128);

  str += StreetNamesToParameterString(this->name());

  str += delim;
  str += std::to_string(length());

  str += delim;
  str += std::to_string(speed());

  str += delim;
  str += "TripLeg_RoadClass_";
  str += TripLeg_RoadClass_Name(road_class());

  str += delim;
  str += std::to_string(begin_heading());

  str += delim;
  str += std::to_string(end_heading());

  str += delim;
  str += std::to_string(begin_shape_index());

  str += delim;
  str += std::to_string(end_shape_index());

  str += delim;
  str += "TripLeg_Traversability_";
  str += TripLeg_Traversability_Name(traversability());

  str += delim;
  str += "TripLeg_Use_";
  str += TripLeg_Use_Name(use());

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
    str += "TripLeg_TravelMode_";
    str += TripLeg_TravelMode_Name(travel_mode());
  }

  // NOTE: Current PopulateEdge implementation

  str += delim;
  if (this->has_vehicle_type()) {
    str += "TripLeg_VehicleType_";
    str += TripLeg_VehicleType_Name(vehicle_type());
  }

  str += delim;
  if (this->has_pedestrian_type()) {
    str += "TripLeg_PedestrianType_";
    str += TripLeg_PedestrianType_Name(pedestrian_type());
  }

  str += delim;
  if (this->has_bicycle_type()) {
    str += "TripLeg_BicycleType_";
    str += TripLeg_BicycleType_Name(bicycle_type());
  }

  str += delim;
  if (this->has_transit_type()) {
    str += "TripLeg_TransitType_";
    str += TripLeg_TransitType_Name(transit_type());
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
  str += "TripLeg_CycleLane_";
  str += TripLeg_CycleLane_Name(cycle_lane());

  str += delim;
  str += std::to_string(bicycle_network());

  str += delim;
  str += "TripLeg_Sidewalk_";
  str += TripLeg_Sidewalk_Name(sidewalk());

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

std::string EnhancedTripLeg_Edge::StreetNamesToParameterString(
    const ::google::protobuf::RepeatedPtrField<::valhalla::StreetName>& street_names) const {
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

std::string EnhancedTripLeg_Edge::SignElementsToParameterString(
    const ::google::protobuf::RepeatedPtrField<::valhalla::TripLeg_SignElement>& sign_elements)
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
// EnhancedTripLeg_IntersectingEdge

EnhancedTripLeg_IntersectingEdge::EnhancedTripLeg_IntersectingEdge(
    TripLeg_IntersectingEdge* mutable_intersecting_edge)
    : mutable_intersecting_edge_(mutable_intersecting_edge) {
}

bool EnhancedTripLeg_IntersectingEdge::IsTraversable(const TripLeg_TravelMode travel_mode) const {
  TripLeg_Traversability t;

  // Set traversability based on travel mode
  if (travel_mode == TripLeg_TravelMode_kDrive) {
    t = driveability();
  } else if (travel_mode == TripLeg_TravelMode_kBicycle) {
    t = cyclability();
  } else {
    t = walkability();
  }

  if (t != TripLeg_Traversability_kNone) {
    return true;
  }
  return false;
}

bool EnhancedTripLeg_IntersectingEdge::IsTraversableOutbound(
    const TripLeg_TravelMode travel_mode) const {
  TripLeg_Traversability t;

  // Set traversability based on travel mode
  if (travel_mode == TripLeg_TravelMode_kDrive) {
    t = driveability();
  } else if (travel_mode == TripLeg_TravelMode_kBicycle) {
    t = cyclability();
  } else {
    t = walkability();
  }

  if ((t == TripLeg_Traversability_kForward) || (t == TripLeg_Traversability_kBoth)) {
    return true;
  }
  return false;
}

bool EnhancedTripLeg_IntersectingEdge::IsHighway() const {
  return ((road_class() == TripLeg_RoadClass_kMotorway) && !(use() == TripLeg_Use_kRampUse));
}

std::string EnhancedTripLeg_IntersectingEdge::ToString() const {
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

  str += " | use=";
  str += std::to_string(use());

  str += " | road_class=";
  str += std::to_string(road_class());

  return str;
}

///////////////////////////////////////////////////////////////////////////////
// EnhancedTripLeg_Node

EnhancedTripLeg_Node::EnhancedTripLeg_Node(TripLeg_Node* mutable_node) : mutable_node_(mutable_node) {
}

bool EnhancedTripLeg_Node::HasIntersectingEdges() const {
  return (intersecting_edge_size() > 0);
}

bool EnhancedTripLeg_Node::HasIntersectingEdgeNameConsistency() const {
  for (const auto& xedge : intersecting_edge()) {
    if (xedge.curr_name_consistency() || xedge.prev_name_consistency()) {
      return true;
    }
  }
  return false;
}

bool EnhancedTripLeg_Node::HasIntersectingEdgeCurrNameConsistency() const {
  for (const auto& xedge : intersecting_edge()) {
    if (xedge.curr_name_consistency()) {
      return true;
    }
  }
  return false;
}

std::unique_ptr<EnhancedTripLeg_IntersectingEdge>
EnhancedTripLeg_Node::GetIntersectingEdge(size_t index) {
  return std::make_unique<EnhancedTripLeg_IntersectingEdge>(mutable_intersecting_edge(index));
}

void EnhancedTripLeg_Node::CalculateRightLeftIntersectingEdgeCounts(
    uint32_t from_heading,
    const TripLeg_TravelMode travel_mode,
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

bool EnhancedTripLeg_Node::HasFowardIntersectingEdge(uint32_t from_heading) {

  for (int i = 0; i < intersecting_edge_size(); ++i) {
    if (is_forward(GetTurnDegree(from_heading, intersecting_edge(i).begin_heading()))) {
      return true;
    }
  }
  return false;
}

bool EnhancedTripLeg_Node::HasForwardTraversableIntersectingEdge(
    uint32_t from_heading,
    const TripLeg_TravelMode travel_mode) {

  for (int i = 0; i < intersecting_edge_size(); ++i) {
    if (is_forward(GetTurnDegree(from_heading, intersecting_edge(i).begin_heading())) &&
        GetIntersectingEdge(i)->IsTraversableOutbound(travel_mode)) {
      return true;
    }
  }
  return false;
}

bool EnhancedTripLeg_Node::HasForwardTraversableSignificantRoadClassXEdge(
    uint32_t from_heading,
    const TripLeg_TravelMode travel_mode,
    TripLeg_RoadClass path_road_class) {

  for (int i = 0; i < intersecting_edge_size(); ++i) {
    auto xedge = GetIntersectingEdge(i);
    // if the intersecting edge is forward
    // and is traversable based on mode
    // and is a significant road class as compared to the path road class
    if (is_forward(GetTurnDegree(from_heading, intersecting_edge(i).begin_heading())) &&
        xedge->IsTraversableOutbound(travel_mode) &&
        ((xedge->road_class() - path_road_class) <= kSignificantRoadClassThreshold)) {
      return true;
    }
  }
  return false;
}

bool EnhancedTripLeg_Node::HasWiderForwardTraversableIntersectingEdge(
    uint32_t from_heading,
    const TripLeg_TravelMode travel_mode) {

  for (int i = 0; i < intersecting_edge_size(); ++i) {
    if (is_wider_forward(GetTurnDegree(from_heading, intersecting_edge(i).begin_heading())) &&
        GetIntersectingEdge(i)->IsTraversableOutbound(travel_mode)) {
      return true;
    }
  }
  return false;
}

bool EnhancedTripLeg_Node::HasWiderForwardTraversableHighwayXEdge(
    uint32_t from_heading,
    const TripLeg_TravelMode travel_mode) {

  for (int i = 0; i < intersecting_edge_size(); ++i) {
    auto xedge = GetIntersectingEdge(i);
    if (is_wider_forward(GetTurnDegree(from_heading, xedge->begin_heading())) &&
        xedge->IsTraversableOutbound(travel_mode) && xedge->IsHighway()) {
      return true;
    }
  }
  return false;
}

bool EnhancedTripLeg_Node::HasTraversableOutboundIntersectingEdge(
    const TripLeg_TravelMode travel_mode) {

  for (int i = 0; i < intersecting_edge_size(); ++i) {
    if (GetIntersectingEdge(i)->IsTraversableOutbound(travel_mode)) {
      return true;
    }
  }
  return false;
}

bool EnhancedTripLeg_Node::HasSpecifiedTurnXEdge(const Turn::Type turn_type,
                                                 uint32_t from_heading,
                                                 const TripLeg_TravelMode travel_mode) {

  for (int i = 0; i < intersecting_edge_size(); ++i) {
    // Only process the traversable outbound edges
    if (GetIntersectingEdge(i)->IsTraversableOutbound(travel_mode) &&
        (Turn::GetType(GetTurnDegree(from_heading, intersecting_edge(i).begin_heading())) ==
         turn_type)) {
      // return true if an intersecting edge of the specified turn type exists
      return true;
    }
  }
  return false;
}

bool EnhancedTripLeg_Node::HasSpecifiedRoadClassXEdge(const TripLeg_RoadClass road_class) {

  // If no intersecting edges then return false
  if (!HasIntersectingEdges()) {
    return false;
  }

  for (int i = 0; i < intersecting_edge_size(); ++i) {
    if (GetIntersectingEdge(i)->road_class() == road_class) {
      // Return true there is a match
      return true;
    }
  }
  // No match found return false
  return false;
}

// TODO: refactor to clean up code
uint32_t EnhancedTripLeg_Node::GetStraightestTraversableIntersectingEdgeTurnDegree(
    uint32_t from_heading,
    const TripLeg_TravelMode travel_mode,
    TripLeg_Use* use) {

  uint32_t staightest_turn_degree = 180; // Initialize to reverse turn degree
  uint32_t staightest_delta = 180;       // Initialize to reverse delta

  for (int i = 0; i < intersecting_edge_size(); ++i) {
    auto xedge = GetIntersectingEdge(i);
    uint32_t intersecting_turn_degree = GetTurnDegree(from_heading, xedge->begin_heading());
    bool xedge_traversable_outbound = xedge->IsTraversableOutbound(travel_mode);
    uint32_t straight_delta = (intersecting_turn_degree > 180) ? (360 - intersecting_turn_degree)
                                                               : intersecting_turn_degree;
    if (xedge_traversable_outbound && (straight_delta < staightest_delta)) {
      staightest_delta = straight_delta;
      staightest_turn_degree = intersecting_turn_degree;
      // If a use pointer was passed in and the intersecting edge has a use then set
      if ((use != nullptr) && xedge->has_use()) {
        *use = xedge->use();
      }
    }
  }
  return staightest_turn_degree;
}

bool EnhancedTripLeg_Node::IsStraightestTraversableIntersectingEdgeReversed(
    uint32_t from_heading,
    const TripLeg_TravelMode travel_mode) {
  uint32_t straightest_traversable_xedge_turn_degree =
      GetStraightestTraversableIntersectingEdgeTurnDegree(from_heading, travel_mode);
  // Determine if the straightest intersecting edge is in the reversed direction
  if ((straightest_traversable_xedge_turn_degree > 124) &&
      (straightest_traversable_xedge_turn_degree < 236)) {
    return true;
  }
  return false;
}

uint32_t EnhancedTripLeg_Node::GetStraightestIntersectingEdgeTurnDegree(uint32_t from_heading) {

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

uint32_t EnhancedTripLeg_Node::GetRightMostTurnDegree(uint32_t turn_degree,
                                                      uint32_t from_heading,
                                                      const TripLeg_TravelMode travel_mode) {

  auto get_right_delta = [](uint32_t turn_degree) -> uint32_t {
    if (turn_degree < 90) {
      return (90 - turn_degree);
    } else if (turn_degree > 270) {
      return (360 - turn_degree + 90);
    } else {
      return (turn_degree - 90);
    }
  };

  uint32_t right_most_turn_degree = turn_degree;
  uint32_t right_most_delta = get_right_delta(turn_degree);

  for (int i = 0; i < intersecting_edge_size(); ++i) {
    // Only process traversable outbound edges
    if (GetIntersectingEdge(i)->IsTraversableOutbound(travel_mode)) {
      // Get the intersecting edge turn degree and right turn delta
      uint32_t xturn_degree = GetTurnDegree(from_heading, intersecting_edge(i).begin_heading());
      uint32_t right_delta = get_right_delta(xturn_degree);
      // Determine if the intersecting edge turn degree is closer to true right (90)
      if (right_delta < right_most_delta) {
        right_most_delta = right_delta;
        right_most_turn_degree = xturn_degree;
      }
    }
  }
  return right_most_turn_degree;
}

uint32_t EnhancedTripLeg_Node::GetLeftMostTurnDegree(uint32_t turn_degree,
                                                     uint32_t from_heading,
                                                     const TripLeg_TravelMode travel_mode) {

  auto get_left_delta = [](uint32_t turn_degree) -> uint32_t {
    if (turn_degree < 90) {
      return (90 + turn_degree);
    } else if (turn_degree < 270) {
      return (270 - turn_degree);
    } else {
      return (turn_degree - 270);
    }
  };

  uint32_t left_most_turn_degree = turn_degree;
  uint32_t left_most_delta = get_left_delta(turn_degree);

  for (int i = 0; i < intersecting_edge_size(); ++i) {
    // Only process traversable outbound edges
    if (GetIntersectingEdge(i)->IsTraversableOutbound(travel_mode)) {
      // Get the intersecting edge turn degree and left turn delta
      uint32_t xturn_degree = GetTurnDegree(from_heading, intersecting_edge(i).begin_heading());
      uint32_t left_delta = get_left_delta(xturn_degree);
      // Determine if the intersecting edge turn degree is closer to true left (270)
      if (left_delta < left_most_delta) {
        left_most_delta = left_delta;
        left_most_turn_degree = xturn_degree;
      }
    }
  }
  return left_most_turn_degree;
}

bool EnhancedTripLeg_Node::IsStreetIntersection() const {
  return (type() == TripLeg_Node_Type_kStreetIntersection);
}

bool EnhancedTripLeg_Node::IsGate() const {
  return (type() == TripLeg_Node_Type_kGate);
}

bool EnhancedTripLeg_Node::IsBollard() const {
  return (type() == TripLeg_Node_Type_kBollard);
}

bool EnhancedTripLeg_Node::IsTollBooth() const {
  return (type() == TripLeg_Node_Type_kTollBooth);
}

bool EnhancedTripLeg_Node::IsTransitEgress() const {
  return (type() == TripLeg_Node_Type_kTransitEgress);
}

bool EnhancedTripLeg_Node::IsTransitStation() const {
  return (type() == TripLeg_Node_Type_kTransitStation);
}

bool EnhancedTripLeg_Node::IsTransitPlatform() const {
  return (type() == TripLeg_Node_Type_kTransitPlatform);
}

bool EnhancedTripLeg_Node::IsBikeShare() const {
  return (type() == TripLeg_Node_Type_kBikeShare);
}

bool EnhancedTripLeg_Node::IsParking() const {
  return (type() == TripLeg_Node_Type_kParking);
}

bool EnhancedTripLeg_Node::IsMotorwayJunction() const {
  return (type() == TripLeg_Node_Type_kMotorwayJunction);
}

bool EnhancedTripLeg_Node::IsBorderControl() const {
  return (type() == TripLeg_Node_Type_kBorderControl);
}

std::string EnhancedTripLeg_Node::ToString() const {
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
// EnhancedTripLeg_Admin

EnhancedTripLeg_Admin::EnhancedTripLeg_Admin(TripLeg_Admin* mutable_admin)
    : mutable_admin_(mutable_admin) {
}

std::string EnhancedTripLeg_Admin::ToString() const {
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
