#include <iostream>
#include <cstdlib>

#include <valhalla/midgard/util.h>
#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/constants.h>

#include "proto/trippath.pb.h"
#include "odin/util.h"
#include "odin/enhancedtrippath.h"

using namespace valhalla::midgard;

namespace valhalla {
namespace odin {

///////////////////////////////////////////////////////////////////////////////
// EnhancedTripPath

EnhancedTripPath_Node* EnhancedTripPath::GetEnhancedNode(const int node_index) {
  return static_cast<EnhancedTripPath_Node*>(mutable_node(node_index));
}

EnhancedTripPath_Edge* EnhancedTripPath::GetPrevEdge(const int node_index,
                                                     int delta) {
  int index = node_index - delta;
  if (IsValidNodeIndex(index))
    return static_cast<EnhancedTripPath_Edge*>(mutable_node(index)->mutable_edge());
  else
    return nullptr;
}

EnhancedTripPath_Edge* EnhancedTripPath::GetCurrEdge(const int node_index) {
  return GetNextEdge(node_index, 0);
}

EnhancedTripPath_Edge* EnhancedTripPath::GetNextEdge(const int node_index,
                                                     int delta) {
  int index = node_index + delta;
  if (IsValidNodeIndex(index) && !IsLastNodeIndex(index))
    return static_cast<EnhancedTripPath_Edge*>(mutable_node(index)->mutable_edge());
  else
    return nullptr;
}

bool EnhancedTripPath::IsValidNodeIndex(int node_index) const {
  if ((node_index >= 0) && (node_index < node_size()))
    return true;
  return false;
}

bool EnhancedTripPath::IsFirstNodeIndex(int node_index) const {
  if (node_index == 0)
    return true;
  return false;
}

bool EnhancedTripPath::IsLastNodeIndex(int node_index) const {
  if (IsValidNodeIndex(node_index) && (node_index == (node_size() - 1)))
    return true;
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

const ::valhalla::odin::TripPath_Location& EnhancedTripPath::GetOrigin() const {
  // Validate location count
  if (location_size() < 2) {
    throw std::runtime_error("Trip must have at least 2 locations");
  }

  return location(0);
}

const ::valhalla::odin::TripPath_Location& EnhancedTripPath::GetDestination() const {
  // Validate location count
  if (location_size() < 2) {
    throw std::runtime_error("Trip must have at least 2 locations");
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
  if (units == DirectionsOptions::Units::DirectionsOptions_Units_kMiles) {
    return (length * kMilePerKm);
  }
  return length;
}

///////////////////////////////////////////////////////////////////////////////
// EnhancedTripPath_Edge

bool EnhancedTripPath_Edge::IsUnnamed() const {
  return (name_size() == 0);
}

bool EnhancedTripPath_Edge::IsUnnamedWalkway() const {
  return (IsUnnamed() && footway());
}

bool EnhancedTripPath_Edge::IsUnnamedCycleway() const {
  return (IsUnnamed() && cycleway());
}

bool EnhancedTripPath_Edge::IsUnnamedMountainBikeTrail() const {
  return (IsUnnamed() && mountain_bike());
}

bool EnhancedTripPath_Edge::IsHighway() const {
  return ((road_class() == TripPath_RoadClass_kMotorway) && (!ramp()));
}

bool EnhancedTripPath_Edge::IsOneway() const {
  return ((traversability() == TripPath_Traversability_kForward)
      || (traversability() == TripPath_Traversability_kBackward));
}

bool EnhancedTripPath_Edge::IsForward(uint32_t prev2curr_turn_degree) const {
  return ((prev2curr_turn_degree > 314) || (prev2curr_turn_degree < 46));
}

bool EnhancedTripPath_Edge::IsWiderForward(uint32_t prev2curr_turn_degree) const {
  return ((prev2curr_turn_degree > 304) || (prev2curr_turn_degree < 56));
}

bool EnhancedTripPath_Edge::IsStraightest(
    uint32_t prev2curr_turn_degree,
    uint32_t straightest_xedge_turn_degree) const {
  if (IsWiderForward(prev2curr_turn_degree)) {
    int path_xedge_turn_degree_delta = std::abs(
        static_cast<int>(prev2curr_turn_degree)
            - static_cast<int>(straightest_xedge_turn_degree));
    if (path_xedge_turn_degree_delta > 180) {
      path_xedge_turn_degree_delta = (360 - path_xedge_turn_degree_delta);
    }
    uint32_t path_straight_delta =
        (prev2curr_turn_degree > 180) ?
            (360 - prev2curr_turn_degree) : prev2curr_turn_degree;
    uint32_t xedge_straight_delta =
        (straightest_xedge_turn_degree > 180) ?
            (360 - straightest_xedge_turn_degree) :
            straightest_xedge_turn_degree;
    return (
        (path_xedge_turn_degree_delta > 10) ?
            (path_straight_delta <= xedge_straight_delta) : true);
  } else {
    return false;
  }
}

std::vector<std::string> EnhancedTripPath_Edge::GetNameList() const {
  std::vector<std::string> name_list;
  for (const auto& name : this->name()) {
    name_list.push_back(name);
  }
  return name_list;
}

float EnhancedTripPath_Edge::GetLength(const DirectionsOptions::Units& units) {
  if (units == DirectionsOptions::Units::DirectionsOptions_Units_kMiles) {
    return (length() * kMilePerKm);
  }
  return length();
}

std::string EnhancedTripPath_Edge::ToString() const {
  std::string str;
  str.reserve(256);

  str += "name=";
  if (name_size() == 0) {
    str += "unnamed";
  } else {
    str += ListToString(this->name());
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

  str += " | ramp=";
  str += std::to_string(ramp());

  str += " | turn_channel=";
  str += std::to_string(turn_channel());

  str += " | ferry=";
  str += std::to_string(ferry());

  str += " | rail_ferry=";
  str += std::to_string(rail_ferry());

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

  str += " | end_node_index=";
  str += std::to_string(end_node_index());

  // Process exits, if needed
  if (this->has_sign()) {
    str += " | exit.number=";
    str += ListToString(this->sign().exit_number());

    str += " | exit.branch=";
    str += ListToString(this->sign().exit_branch());

    str += " | exit.toward=";
    str += ListToString(this->sign().exit_toward());

    str += " | exit.name=";
    str += ListToString(this->sign().exit_name());
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

  str += " | road=";
  str += std::to_string(road());

  str += " | track=";
  str += std::to_string(track());

  str += " | driveway=";
  str += std::to_string(driveway());

  str += " | alley=";
  str += std::to_string(alley());

  str += " | parking_aisle=";
  str += std::to_string(parking_aisle());

  str += " | emergency_access=";
  str += std::to_string(emergency_access());

  str += " | drive_thru=";
  str += std::to_string(drive_thru());

  str += " | culdesac=";
  str += std::to_string(culdesac());

  str += " | footway=";
  str += std::to_string(footway());

  str += " | stairs=";
  str += std::to_string(stairs());

  str += " | cycleway=";
  str += std::to_string(cycleway());

  str += " | mountain_bike=";
  str += std::to_string(mountain_bike());

  str += " | rail=";
  str += std::to_string(rail());

  str += " | bus=";
  str += std::to_string(bus());

  str += " | transit_connection=";
  str += std::to_string(transit_connection());

  str += " | other=";
  str += std::to_string(other());

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

  return str;
}

std::string EnhancedTripPath_Edge::ToParameterString() const {
  const std::string delim = ", ";
  std::string str;
  str.reserve(128);

  str += ListToParameterString(this->name());

  str += delim;
  str += std::to_string(length());

  str += delim;
  str += std::to_string(speed());

  str += delim;
  str += "TripPath_RoadClass_";
  str +=
      TripPath_RoadClass_descriptor()->FindValueByNumber(road_class())->name();

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
  str += TripPath_Traversability_descriptor()->FindValueByNumber(traversability())
      ->name();

  str += delim;
  str += std::to_string(ramp());

  str += delim;
  str += std::to_string(turn_channel());

  str += delim;
  str += std::to_string(ferry());

  str += delim;
  str += std::to_string(rail_ferry());

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
  str += std::to_string(end_node_index());

  str += delim;
  str += ListToParameterString(this->sign().exit_number());

  str += delim;
  str += ListToParameterString(this->sign().exit_branch());

  str += delim;
  str += ListToParameterString(this->sign().exit_toward());

  str += delim;
  str += ListToParameterString(this->sign().exit_name());

  str += delim;
  if (this->has_travel_mode()) {
    str += "TripPath_TravelMode_";
    str += TripPath_TravelMode_descriptor()->FindValueByNumber(travel_mode())
        ->name();
  }

  // NOTE: Current PopulateEdge implementation

  str += delim;
  if (this->has_vehicle_type()) {
    str += "TripPath_VehicleType_";
    str += TripPath_VehicleType_descriptor()->FindValueByNumber(vehicle_type())
        ->name();
  }

  str += delim;
  if (this->has_pedestrian_type()) {
    str += "TripPath_PedestrianType_";
    str += TripPath_PedestrianType_descriptor()->FindValueByNumber(
        pedestrian_type())->name();
  }

  str += delim;
  if (this->has_bicycle_type()) {
    str += "TripPath_BicycleType_";
    str += TripPath_BicycleType_descriptor()->FindValueByNumber(bicycle_type())
        ->name();
  }

  str += delim;
  if (this->has_transit_type()) {
    str += "TripPath_TransitType_";
    str += TripPath_TransitType_descriptor()->FindValueByNumber(transit_type())
          ->name();
  }

  str += delim;
  str += std::to_string(drive_on_right());

  str += delim;
  str += std::to_string(road());

  str += delim;
  str += std::to_string(track());

  str += delim;
  str += std::to_string(driveway());

  str += delim;
  str += std::to_string(alley());

  str += delim;
  str += std::to_string(parking_aisle());

  str += delim;
  str += std::to_string(emergency_access());

  str += delim;
  str += std::to_string(drive_thru());

  str += delim;
  str += std::to_string(culdesac());

  str += delim;
  str += std::to_string(footway());

  str += delim;
  str += std::to_string(stairs());

  str += delim;
  str += std::to_string(cycleway());

  str += delim;
  str += std::to_string(mountain_bike());

  str += delim;
  str += std::to_string(rail());

  str += delim;
  str += std::to_string(bus());

  str += delim;
  str += std::to_string(transit_connection());

  str += delim;
  str += std::to_string(other());

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

  return str;
}

std::string EnhancedTripPath_Edge::ListToString(
    const ::google::protobuf::RepeatedPtrField<::std::string>& string_list) const {
  std::string str;

  bool is_first = true;
  for (const auto& item : string_list) {
    if (is_first)
      is_first = false;
    else
      str += "/";
    str += item;
  }
  return str;
}

std::string EnhancedTripPath_Edge::ListToParameterString(
    const ::google::protobuf::RepeatedPtrField<::std::string>& string_list) const {
  std::string str;

  str += "{ ";
  bool is_first = true;
  for (const auto& item : string_list) {
    if (is_first)
      is_first = false;
    else
      str += ", ";
    str += "\"";
    str += item;
    str += "\"";
  }
  str += " }";

  return str;
}

///////////////////////////////////////////////////////////////////////////////
// EnhancedTripPath_IntersectingEdge

bool EnhancedTripPath_IntersectingEdge::IsTraversable(
    const TripPath_TravelMode travel_mode) const {
  TripPath_Traversability t;

  // Set traversability based on travel mode
  if (travel_mode == TripPath_TravelMode_kDrive)
    t = driveability();
  else if (travel_mode == TripPath_TravelMode_kBicycle)
    t = cyclability();
  else
    t = walkability();

  if (t != TripPath_Traversability_kNone) {
    return true;
  }
  return false;

}


bool EnhancedTripPath_IntersectingEdge::IsTraversableOutbound(
    const TripPath_TravelMode travel_mode) const {
  TripPath_Traversability t;

  // Set traversability based on travel mode
  if (travel_mode == TripPath_TravelMode_kDrive)
    t = driveability();
  else if (travel_mode == TripPath_TravelMode_kBicycle)
    t = cyclability();
  else
    t = walkability();

  if ((t == TripPath_Traversability_kForward)
      || (t == TripPath_Traversability_kBoth)) {
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

EnhancedTripPath_IntersectingEdge* EnhancedTripPath_Node::GetIntersectingEdge(
    size_t index) {
  return static_cast<EnhancedTripPath_IntersectingEdge*>(mutable_intersecting_edge(
      index));
}

void EnhancedTripPath_Node::CalculateRightLeftIntersectingEdgeCounts(
    uint32_t from_heading, const TripPath_TravelMode travel_mode,
    IntersectingEdgeCounts& xedge_counts) {
  xedge_counts.clear();

  // No turn - just return
  if (intersecting_edge_size() == 0)
    return;

  uint32_t path_turn_degree = GetTurnDegree(from_heading,
                                            edge().begin_heading());
  for (int i = 0; i < intersecting_edge_size(); ++i) {
    uint32_t intersecting_turn_degree = GetTurnDegree(
        from_heading, intersecting_edge(i).begin_heading());
    bool xedge_traversable_outbound =
        GetIntersectingEdge(i)->IsTraversableOutbound(travel_mode);

    if (path_turn_degree > 180) {
      if ((intersecting_turn_degree > path_turn_degree)
          || (intersecting_turn_degree < 180)) {
        ++xedge_counts.right;
        if (IsSimilarTurnDegree(path_turn_degree, intersecting_turn_degree,
                                true)) {
          ++xedge_counts.right_similar;
          if (xedge_traversable_outbound) {
            ++xedge_counts.right_similar_traversable_outbound;
          }
        }
        if (xedge_traversable_outbound) {
          ++xedge_counts.right_traversable_outbound;
        }
      } else if ((intersecting_turn_degree < path_turn_degree)
          && (intersecting_turn_degree > 180)) {
        ++xedge_counts.left;
        if (IsSimilarTurnDegree(path_turn_degree, intersecting_turn_degree,
                                false)) {
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
      if ((intersecting_turn_degree > path_turn_degree)
          && (intersecting_turn_degree < 180)) {
        ++xedge_counts.right;
        if (IsSimilarTurnDegree(path_turn_degree, intersecting_turn_degree,
                                true)) {
          ++xedge_counts.right_similar;
          if (xedge_traversable_outbound) {
            ++xedge_counts.right_similar_traversable_outbound;
          }
        }
        if (xedge_traversable_outbound) {
          ++xedge_counts.right_traversable_outbound;
        }
      } else if ((intersecting_turn_degree < path_turn_degree)
          || (intersecting_turn_degree > 180)) {
        ++xedge_counts.left;
        if (IsSimilarTurnDegree(path_turn_degree, intersecting_turn_degree,
                                false)) {
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

bool EnhancedTripPath_Node::HasFowardIntersectingEdge(
    uint32_t from_heading) {

  for (int i = 0; i < intersecting_edge_size(); ++i) {
    uint32_t intersecting_turn_degree = GetTurnDegree(
        from_heading, intersecting_edge(i).begin_heading());
    if ((intersecting_turn_degree > 314) || (intersecting_turn_degree < 46)) {
      return true;
    }
  }
  return false;
}

bool EnhancedTripPath_Node::HasForwardTraversableIntersectingEdge(
    uint32_t from_heading, const TripPath_TravelMode travel_mode) {

  for (int i = 0; i < intersecting_edge_size(); ++i) {
    uint32_t intersecting_turn_degree = GetTurnDegree(
        from_heading, intersecting_edge(i).begin_heading());
    bool xedge_traversable_outbound =
        GetIntersectingEdge(i)->IsTraversableOutbound(travel_mode);
    if (((intersecting_turn_degree > 314) || (intersecting_turn_degree < 46))
        && xedge_traversable_outbound) {
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

//TODO: refactor to clean up code
uint32_t EnhancedTripPath_Node::GetStraightestTraversableIntersectingEdgeTurnDegree(
    uint32_t from_heading, const TripPath_TravelMode travel_mode) {

  uint32_t staightest_turn_degree = 180;  // Initialize to reverse turn degree
  uint32_t staightest_delta = 180;  // Initialize to reverse delta

  for (int i = 0; i < intersecting_edge_size(); ++i) {
    uint32_t intersecting_turn_degree = GetTurnDegree(
        from_heading, intersecting_edge(i).begin_heading());
    bool xedge_traversable_outbound =
        GetIntersectingEdge(i)->IsTraversableOutbound(travel_mode);
    uint32_t straight_delta =
        (intersecting_turn_degree > 180) ?
            (360 - intersecting_turn_degree) : intersecting_turn_degree;
    if (xedge_traversable_outbound && (straight_delta < staightest_delta)) {
      staightest_delta = straight_delta;
      staightest_turn_degree = intersecting_turn_degree;
    }
  }
  return staightest_turn_degree;
}

uint32_t EnhancedTripPath_Node::GetStraightestIntersectingEdgeTurnDegree(
    uint32_t from_heading) {

  uint32_t staightest_turn_degree = 180;  // Initialize to reverse turn degree
  uint32_t staightest_delta = 180;  // Initialize to reverse delta

  for (int i = 0; i < intersecting_edge_size(); ++i) {
    uint32_t intersecting_turn_degree = GetTurnDegree(
        from_heading, intersecting_edge(i).begin_heading());
    uint32_t straight_delta =
        (intersecting_turn_degree > 180) ?
            (360 - intersecting_turn_degree) : intersecting_turn_degree;
    if (straight_delta < staightest_delta) {
      staightest_delta = straight_delta;
      staightest_turn_degree = intersecting_turn_degree;
    }
  }
  return staightest_turn_degree;
}

std::string EnhancedTripPath_Node::ToString() const {
  std::string str;
  str.reserve(256);

  str += "elapsed_time=";
  str += std::to_string(elapsed_time());

  str += " | admin_index=";
  str += std::to_string(admin_index());

  str += " | street_intersection=";
  str += std::to_string(street_intersection());

  str += " | gate=";
  str += std::to_string(gate());

  str += " | bollard=";
  str += std::to_string(bollard());

  str += " | toll_booth=";
  str += std::to_string(toll_booth());

  str += " | bike_share=";
  str += std::to_string(bike_share());

  str += " | parking=";
  str += std::to_string(parking());

  str += " | motorway_junction=";
  str += std::to_string(motorway_junction());

  str += " | fork=";
  str += std::to_string(fork());

  if (has_transit_stop_info()) {
    str += " | transit_stop_info.type=";
    str += std::to_string(transit_stop_info().type());

    str += " | transit_stop_info.onestop_id=";
    str += transit_stop_info().onestop_id();

    str += " | transit_stop_info.name=";
    str += transit_stop_info().name();

    str += " | transit_stop_info.arrival_date_time=";
    str += transit_stop_info().arrival_date_time();

    str += " | transit_stop_info.departure_date_time=";
    str += transit_stop_info().departure_date_time();

    str += " | transit_stop_info.is_parent_stop=";
    str += std::to_string(transit_stop_info().is_parent_stop());

    str += " | transit_stop_info.assumed_schedule()=";
    str += std::to_string(transit_stop_info().assumed_schedule());
  }

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

}
}

