#include <iostream>
#include <list>

#include "baldr/streetnames.h"
#include "baldr/streetnames_us.h"
#include "midgard/constants.h"
#include "midgard/logging.h"
#include "midgard/util.h"

#include "odin/maneuver.h"
#include "odin/transitrouteinfo.h"

#include <valhalla/proto/directions.pb.h>
#include <valhalla/proto/options.pb.h>
#include <valhalla/proto/tripcommon.pb.h>

using namespace valhalla::odin;
using namespace valhalla::baldr;

namespace {
const std::unordered_map<int, std::string>
    relative_direction_string{{static_cast<int>(Maneuver::RelativeDirection::kNone),
                               "Maneuver::RelativeDirection::kNone"},
                              {static_cast<int>(Maneuver::RelativeDirection::kKeepStraight),
                               "Maneuver::RelativeDirection::kKeepStraight"},
                              {static_cast<int>(Maneuver::RelativeDirection::kKeepRight),
                               "Maneuver::RelativeDirection::kKeepRight"},
                              {static_cast<int>(Maneuver::RelativeDirection::kRight),
                               "Maneuver::RelativeDirection::kRight"},
                              {static_cast<int>(Maneuver::RelativeDirection::KReverse),
                               "Maneuver::RelativeDirection::KReverse"},
                              {static_cast<int>(Maneuver::RelativeDirection::kLeft),
                               "Maneuver::RelativeDirection::kLeft"},
                              {static_cast<int>(Maneuver::RelativeDirection::kKeepLeft),
                               "Maneuver::RelativeDirection::kKeepLeft"}};

const std::string& DirectionsLeg_Maneuver_Type_Name(int v) {
  static const std::unordered_map<int, std::string> values{
      {0, "kNone"},
      {1, "kStart"},
      {2, "kStartRight"},
      {3, "kStartLeft"},
      {4, "kDestination"},
      {5, "kDestinationRight"},
      {6, "kDestinationLeft"},
      {7, "kBecomes"},
      {8, "kContinue"},
      {9, "kSlightRight"},
      {10, "kRight"},
      {11, "kSharpRight"},
      {12, "kUturnRight"},
      {13, "kUturnLeft"},
      {14, "kSharpLeft"},
      {15, "kLeft"},
      {16, "kSlightLeft"},
      {17, "kRampStraight"},
      {18, "kRampRight"},
      {19, "kRampLeft"},
      {20, "kExitRight"},
      {21, "kExitLeft"},
      {22, "kStayStraight"},
      {23, "kStayRight"},
      {24, "kStayLeft"},
      {25, "kMerge"},
      {26, "kRoundaboutEnter"},
      {27, "kRoundaboutExit"},
      {28, "kFerryEnter"},
      {29, "kFerryExit"},
      {30, "kTransit"},
      {31, "kTransitTransfer"},
      {32, "kTransitRemainOn"},
      {33, "kTransitConnectionStart"},
      {34, "kTransitConnectionTransfer"},
      {35, "kTransitConnectionDestination"},
      {36, "kPostTransitConnectionDestination"},
      {37, "kMergeRight"},
      {38, "kMergeLeft"},
  };
  auto f = values.find(v);
  if (f == values.cend())
    throw std::runtime_error("Missing value in protobuf enum to string");
  return f->second;
}

const std::string& DirectionsLeg_Maneuver_CardinalDirection_Name(int v) {
  static const std::unordered_map<int, std::string> values{
      {0, "kNorth"}, {1, "kNorthEast"}, {2, "kEast"}, {3, "kSouthEast"},
      {4, "kSouth"}, {5, "kSouthWest"}, {6, "kWest"}, {7, "kNorthWest"},
  };
  auto f = values.find(v);
  if (f == values.cend())
    throw std::runtime_error("Missing value in protobuf enum to string");
  return f->second;
}

} // namespace

namespace valhalla {
namespace odin {

Maneuver::Maneuver()
    : type_(DirectionsLeg_Maneuver_Type_kNone), length_(0.0f), time_(0), basic_time_(0),
      turn_degree_(0), begin_relative_direction_(RelativeDirection::kNone),
      begin_cardinal_direction_(DirectionsLeg_Maneuver_CardinalDirection_kNorth), begin_heading_(0),
      end_heading_(0), begin_node_index_(0), end_node_index_(0), begin_shape_index_(0),
      end_shape_index_(0), ramp_(false), turn_channel_(false), ferry_(false), rail_ferry_(false),
      roundabout_(false), portions_toll_(false), portions_unpaved_(false), portions_highway_(false),
      internal_intersection_(false), internal_right_turn_count_(0), internal_left_turn_count_(0),
      roundabout_exit_count_(0), travel_mode_(TripLeg_TravelMode_kDrive),
      vehicle_type_(TripLeg_VehicleType_kCar), pedestrian_type_(TripLeg_PedestrianType_kFoot),
      bicycle_type_(TripLeg_BicycleType_kRoad), transit_type_(TripLeg_TransitType_kRail),
      transit_connection_(false), rail_(false), bus_(false), fork_(false),
      begin_intersecting_edge_name_consistency_(false), intersecting_forward_edge_(false),
      tee_(false), unnamed_walkway_(false), unnamed_cycleway_(false),
      unnamed_mountain_bike_trail_(false), verbal_multi_cue_(false), to_stay_on_(false),
      drive_on_right_(true), has_time_restrictions_(false) {
  street_names_ = std::make_unique<StreetNames>();
  begin_street_names_ = std::make_unique<StreetNames>();
  cross_street_names_ = std::make_unique<StreetNames>();
  roundabout_exit_street_names_ = std::make_unique<StreetNames>();
}

const DirectionsLeg_Maneuver_Type& Maneuver::type() const {
  return type_;
}

void Maneuver::set_type(const DirectionsLeg_Maneuver_Type& type) {
  type_ = type;
}

bool Maneuver::IsDestinationType() const {
  return ((type_ == DirectionsLeg_Maneuver_Type_kDestination) ||
          (type_ == DirectionsLeg_Maneuver_Type_kDestinationLeft) ||
          (type_ == DirectionsLeg_Maneuver_Type_kDestinationRight));
}

bool Maneuver::IsMergeType() const {
  return ((type_ == DirectionsLeg_Maneuver_Type_kMerge) ||
          (type_ == DirectionsLeg_Maneuver_Type_kMergeLeft) ||
          (type_ == DirectionsLeg_Maneuver_Type_kMergeRight));
}

const StreetNames& Maneuver::street_names() const {
  return *street_names_;
}

void Maneuver::set_street_names(const std::vector<std::pair<std::string, bool>>& names) {
  street_names_ = std::make_unique<StreetNamesUs>(names);
}

void Maneuver::set_street_names(std::unique_ptr<StreetNames>&& street_names) {
  street_names_ = std::move(street_names);
}

bool Maneuver::HasStreetNames() const {
  return (!street_names_->empty());
}

void Maneuver::ClearStreetNames() {
  street_names_->clear();
}

bool Maneuver::HasSameNames(const Maneuver* other_maneuver,
                            bool allow_begin_intersecting_edge_name_consistency) const {

  // Allow similar intersecting edge names
  // OR verify that there are no similar intersecting edge names
  if (allow_begin_intersecting_edge_name_consistency || !begin_intersecting_edge_name_consistency()) {
    // If this maneuver has street names
    // and other maneuver exists
    if (HasStreetNames() && other_maneuver) {
      // other and this maneuvers have same names
      std::unique_ptr<StreetNames> same_street_names =
          other_maneuver->street_names().FindCommonStreetNames(street_names());
      if (!same_street_names->empty() && (street_names().size() == same_street_names->size())) {
        return true;
      }
    }
  }
  return false;
}

bool Maneuver::HasSimilarNames(const Maneuver* other_maneuver,
                               bool allow_begin_intersecting_edge_name_consistency) const {

  // Allow similar intersecting edge names
  // OR verify that there are no similar intersecting edge names
  if (allow_begin_intersecting_edge_name_consistency || !begin_intersecting_edge_name_consistency()) {
    // If this maneuver has street names
    // and other maneuver exists
    if (HasStreetNames() && other_maneuver) {
      // other and this maneuvers have similar names
      std::unique_ptr<StreetNames> similar_street_names =
          other_maneuver->street_names().FindCommonBaseNames(street_names());
      if (!similar_street_names->empty() && (street_names().size() == similar_street_names->size())) {
        return true;
      }
    }
  }
  return false;
}

const StreetNames& Maneuver::begin_street_names() const {
  return *begin_street_names_;
}

void Maneuver::set_begin_street_names(const std::vector<std::pair<std::string, bool>>& names) {
  begin_street_names_ = std::make_unique<StreetNamesUs>(names);
}

void Maneuver::set_begin_street_names(std::unique_ptr<StreetNames>&& begin_street_names) {
  begin_street_names_ = std::move(begin_street_names);
}

bool Maneuver::HasBeginStreetNames() const {
  return (!begin_street_names_->empty());
}

void Maneuver::ClearBeginStreetNames() {
  begin_street_names_->clear();
}

const StreetNames& Maneuver::cross_street_names() const {
  return *cross_street_names_;
}

void Maneuver::set_cross_street_names(const std::vector<std::pair<std::string, bool>>& names) {
  cross_street_names_ = std::make_unique<StreetNamesUs>(names);
}

void Maneuver::set_cross_street_names(std::unique_ptr<StreetNames>&& cross_street_names) {
  cross_street_names_ = std::move(cross_street_names);
}

bool Maneuver::HasCrossStreetNames() const {
  return (!cross_street_names_->empty());
}

void Maneuver::ClearCrossStreetNames() {
  cross_street_names_->clear();
}

const std::string& Maneuver::instruction() const {
  return instruction_;
}

void Maneuver::set_instruction(const std::string& instruction) {
  instruction_ = instruction;
}

void Maneuver::set_instruction(std::string&& instruction) {
  instruction_ = std::move(instruction);
}

float Maneuver::length(const Options::Units& units) const {
  if (units == Options::miles) {
    return (length_ * midgard::kMilePerKm);
  }
  return length_;
}

void Maneuver::set_length(float length) {
  length_ = length;
}

uint32_t Maneuver::time() const {
  return time_;
}

void Maneuver::set_time(uint32_t time) {
  time_ = time;
}

uint32_t Maneuver::basic_time() const {
  return basic_time_;
}

void Maneuver::set_basic_time(uint32_t basic_time) {
  basic_time_ = basic_time;
}

uint32_t Maneuver::turn_degree() const {
  return turn_degree_;
}

void Maneuver::set_turn_degree(uint32_t turn_degree) {
  turn_degree_ = turn_degree;
}

Maneuver::RelativeDirection Maneuver::begin_relative_direction() const {
  return begin_relative_direction_;
}

void Maneuver::set_begin_relative_direction(RelativeDirection begin_relative_direction) {
  begin_relative_direction_ = begin_relative_direction;
}

DirectionsLeg_Maneuver_CardinalDirection Maneuver::begin_cardinal_direction() const {
  return begin_cardinal_direction_;
}

void Maneuver::set_begin_cardinal_direction(
    DirectionsLeg_Maneuver_CardinalDirection begin_cardinal_direction) {
  begin_cardinal_direction_ = begin_cardinal_direction;
}

uint32_t Maneuver::begin_heading() const {
  return begin_heading_;
}

void Maneuver::set_begin_heading(uint32_t beginHeading) {
  begin_heading_ = beginHeading;
}

uint32_t Maneuver::end_heading() const {
  return end_heading_;
}

void Maneuver::set_end_heading(uint32_t endHeading) {
  end_heading_ = endHeading;
}

uint32_t Maneuver::begin_node_index() const {
  return begin_node_index_;
}

void Maneuver::set_begin_node_index(uint32_t beginNodeIndex) {
  begin_node_index_ = beginNodeIndex;
}

uint32_t Maneuver::end_node_index() const {
  return end_node_index_;
}

void Maneuver::set_end_node_index(uint32_t endNodeIndex) {
  end_node_index_ = endNodeIndex;
}

uint32_t Maneuver::begin_shape_index() const {
  return begin_shape_index_;
}

void Maneuver::set_begin_shape_index(uint32_t beginShapeIndex) {
  begin_shape_index_ = beginShapeIndex;
}

uint32_t Maneuver::end_shape_index() const {
  return end_shape_index_;
}

void Maneuver::set_end_shape_index(uint32_t endShapeIndex) {
  end_shape_index_ = endShapeIndex;
}

bool Maneuver::ramp() const {
  return ramp_;
}

void Maneuver::set_ramp(bool ramp) {
  ramp_ = ramp;
}

bool Maneuver::turn_channel() const {
  return turn_channel_;
}

void Maneuver::set_turn_channel(bool turn_channel) {
  turn_channel_ = turn_channel;
}

bool Maneuver::ferry() const {
  return ferry_;
}

void Maneuver::set_ferry(bool ferry) {
  ferry_ = ferry;
}

bool Maneuver::rail_ferry() const {
  return rail_ferry_;
}

void Maneuver::set_rail_ferry(bool rail_ferry) {
  rail_ferry_ = rail_ferry;
}

bool Maneuver::roundabout() const {
  return roundabout_;
}

void Maneuver::set_roundabout(bool roundabout) {
  roundabout_ = roundabout;
}

bool Maneuver::portions_toll() const {
  return portions_toll_;
}

void Maneuver::set_portions_toll(bool portionsToll) {
  portions_toll_ = portionsToll;
}

bool Maneuver::has_time_restrictions() const {
  return has_time_restrictions_;
}
void Maneuver::set_has_time_restrictions(bool has_time_restrictions) {
  has_time_restrictions_ = has_time_restrictions;
}

bool Maneuver::portions_unpaved() const {
  return portions_unpaved_;
}

void Maneuver::set_portions_unpaved(bool portionsUnpaved) {
  portions_unpaved_ = portionsUnpaved;
}

bool Maneuver::portions_highway() const {
  return portions_highway_;
}

void Maneuver::set_portions_highway(bool portionsHighway) {
  portions_highway_ = portionsHighway;
}

bool Maneuver::internal_intersection() const {
  return internal_intersection_;
}

void Maneuver::set_internal_intersection(bool internal_intersection) {
  internal_intersection_ = internal_intersection;
}

bool Maneuver::HasUsableInternalIntersectionName() const {
  uint32_t link_count = (end_node_index_ - begin_node_index_);
  if (internal_intersection_ && !street_names_->empty() && ((link_count == 1) || (link_count == 3))) {
    return true;
  }
  return false;
}

const Signs& Maneuver::signs() const {
  return signs_;
}

Signs* Maneuver::mutable_signs() {
  return &signs_;
}

bool Maneuver::HasExitSign() const {
  return signs_.HasExit();
}

bool Maneuver::HasExitNumberSign() const {
  return signs_.HasExitNumber();
}

bool Maneuver::HasExitBranchSign() const {
  return signs_.HasExitBranch();
}

bool Maneuver::HasExitTowardSign() const {
  return signs_.HasExitToward();
}

bool Maneuver::HasExitNameSign() const {
  return signs_.HasExitName();
}

uint32_t Maneuver::internal_right_turn_count() const {
  return internal_right_turn_count_;
}

void Maneuver::set_internal_right_turn_count(uint32_t internal_right_turn_count) {
  internal_right_turn_count_ = internal_right_turn_count;
}

uint32_t Maneuver::internal_left_turn_count() const {
  return internal_left_turn_count_;
}

void Maneuver::set_internal_left_turn_count(uint32_t internal_left_turn_count) {
  internal_left_turn_count_ = internal_left_turn_count;
}

uint32_t Maneuver::roundabout_exit_count() const {
  return roundabout_exit_count_;
}

void Maneuver::set_roundabout_exit_count(uint32_t roundabout_exit_count) {
  roundabout_exit_count_ = roundabout_exit_count;
}

bool Maneuver::fork() const {
  return fork_;
}

void Maneuver::set_fork(bool fork) {
  fork_ = fork;
}

bool Maneuver::begin_intersecting_edge_name_consistency() const {
  return begin_intersecting_edge_name_consistency_;
}

void Maneuver::set_begin_intersecting_edge_name_consistency(
    bool begin_intersecting_edge_name_consistency) {
  begin_intersecting_edge_name_consistency_ = begin_intersecting_edge_name_consistency;
}

bool Maneuver::intersecting_forward_edge() const {
  return intersecting_forward_edge_;
}

void Maneuver::set_intersecting_forward_edge(bool intersecting_forward_edge) {
  intersecting_forward_edge_ = intersecting_forward_edge;
}

const std::string& Maneuver::verbal_transition_alert_instruction() const {
  return verbal_transition_alert_instruction_;
}

void Maneuver::set_verbal_transition_alert_instruction(
    const std::string& verbal_transition_alert_instruction) {
  verbal_transition_alert_instruction_ = verbal_transition_alert_instruction;
}

void Maneuver::set_verbal_transition_alert_instruction(
    std::string&& verbal_transition_alert_instruction) {
  verbal_transition_alert_instruction_ = std::move(verbal_transition_alert_instruction);
}

bool Maneuver::HasVerbalTransitionAlertInstruction() const {
  return (!verbal_transition_alert_instruction_.empty());
}

const std::string& Maneuver::verbal_pre_transition_instruction() const {
  return verbal_pre_transition_instruction_;
}

void Maneuver::set_verbal_pre_transition_instruction(
    const std::string& verbal_pre_transition_instruction) {
  verbal_pre_transition_instruction_ = verbal_pre_transition_instruction;
}

void Maneuver::set_verbal_pre_transition_instruction(
    std::string&& verbal_pre_transition_instruction) {
  verbal_pre_transition_instruction_ = std::move(verbal_pre_transition_instruction);
}

bool Maneuver::HasVerbalPreTransitionInstruction() const {
  return (!verbal_pre_transition_instruction_.empty());
}

const std::string& Maneuver::verbal_post_transition_instruction() const {
  return verbal_post_transition_instruction_;
}

void Maneuver::set_verbal_post_transition_instruction(
    const std::string& verbal_post_transition_instruction) {
  verbal_post_transition_instruction_ = verbal_post_transition_instruction;
}

void Maneuver::set_verbal_post_transition_instruction(
    std::string&& verbal_post_transition_instruction) {
  verbal_post_transition_instruction_ = std::move(verbal_post_transition_instruction);
}

bool Maneuver::HasVerbalPostTransitionInstruction() const {
  return (!verbal_post_transition_instruction_.empty());
}

bool Maneuver::tee() const {
  return tee_;
}

void Maneuver::set_tee(bool tee) {
  tee_ = tee;
}

bool Maneuver::unnamed_walkway() const {
  return unnamed_walkway_;
}

void Maneuver::set_unnamed_walkway(bool unnamed_walkway) {
  unnamed_walkway_ = unnamed_walkway;
}

bool Maneuver::unnamed_cycleway() const {
  return unnamed_cycleway_;
}

void Maneuver::set_unnamed_cycleway(bool unnamed_cycleway) {
  unnamed_cycleway_ = unnamed_cycleway;
}

bool Maneuver::unnamed_mountain_bike_trail() const {
  return unnamed_mountain_bike_trail_;
}

void Maneuver::set_unnamed_mountain_bike_trail(bool unnamed_mountain_bike_trail) {
  unnamed_mountain_bike_trail_ = unnamed_mountain_bike_trail;
}

bool Maneuver::verbal_multi_cue() const {
  return verbal_multi_cue_;
}

void Maneuver::set_verbal_multi_cue(bool verbal_multi_cue) {
  verbal_multi_cue_ = verbal_multi_cue;
}

bool Maneuver::to_stay_on() const {
  return to_stay_on_;
}

void Maneuver::set_to_stay_on(bool to_stay_on) {
  to_stay_on_ = to_stay_on;
}

const StreetNames& Maneuver::roundabout_exit_street_names() const {
  return *roundabout_exit_street_names_;
}

void Maneuver::set_roundabout_exit_street_names(
    const std::vector<std::pair<std::string, bool>>& names) {
  roundabout_exit_street_names_ = std::make_unique<StreetNamesUs>(names);
}

void Maneuver::set_roundabout_exit_street_names(
    std::unique_ptr<StreetNames>&& roundabout_exit_street_names) {
  roundabout_exit_street_names_ = std::move(roundabout_exit_street_names);
}

bool Maneuver::HasRoundaboutExitStreetNames() const {
  return (!roundabout_exit_street_names_->empty());
}

void Maneuver::ClearRoundaboutExitStreetNames() {
  roundabout_exit_street_names_->clear();
}

Maneuver::RelativeDirection Maneuver::merge_to_relative_direction() const {
  return merge_to_relative_direction_;
}

void Maneuver::set_merge_to_relative_direction(RelativeDirection merge_to_relative_direction) {
  merge_to_relative_direction_ = merge_to_relative_direction;
}

bool Maneuver::drive_on_right() const {
  return drive_on_right_;
}

void Maneuver::set_drive_on_right(bool drive_on_right) {
  drive_on_right_ = drive_on_right;
}

TripLeg_TravelMode Maneuver::travel_mode() const {
  return travel_mode_;
}

void Maneuver::set_travel_mode(TripLeg_TravelMode travel_mode) {
  travel_mode_ = travel_mode;
}

TripLeg_VehicleType Maneuver::vehicle_type() const {
  return vehicle_type_;
}

void Maneuver::set_vehicle_type(TripLeg_VehicleType vehicle_type) {
  vehicle_type_ = vehicle_type;
}

TripLeg_PedestrianType Maneuver::pedestrian_type() const {
  return pedestrian_type_;
}

void Maneuver::set_pedestrian_type(TripLeg_PedestrianType pedestrian_type) {
  pedestrian_type_ = pedestrian_type;
}

TripLeg_BicycleType Maneuver::bicycle_type() const {
  return bicycle_type_;
}

void Maneuver::set_bicycle_type(TripLeg_BicycleType bicycle_type) {
  bicycle_type_ = bicycle_type;
}

TripLeg_TransitType Maneuver::transit_type() const {
  return transit_type_;
}

void Maneuver::set_transit_type(TripLeg_TransitType transit_type) {
  transit_type_ = transit_type;
}

bool Maneuver::transit_connection() const {
  return transit_connection_;
}

void Maneuver::set_transit_connection(bool transit_connection) {
  transit_connection_ = transit_connection;
}

const TransitEgressInfo& Maneuver::transit_connection_egress_info() const {
  return transit_connection_egress_info_;
}

void Maneuver::set_transit_connection_egress_info(
    const TransitEgressInfo& transit_connection_egress_info) {
  transit_connection_egress_info_ = transit_connection_egress_info;
}

const TransitStationInfo& Maneuver::transit_connection_station_info() const {
  return transit_connection_station_info_;
}

void Maneuver::set_transit_connection_station_info(
    const TransitStationInfo& transit_connection_station_info) {
  transit_connection_station_info_ = transit_connection_station_info;
}

const TransitPlatformInfo& Maneuver::transit_connection_platform_info() const {
  return transit_connection_platform_info_;
}

void Maneuver::set_transit_connection_platform_info(
    const TransitPlatformInfo& transit_connection_platform_info) {
  transit_connection_platform_info_ = transit_connection_platform_info;
}

bool Maneuver::rail() const {
  return rail_;
}

void Maneuver::set_rail(bool rail) {
  rail_ = rail;
}

bool Maneuver::bus() const {
  return bus_;
}

void Maneuver::set_bus(bool bus) {
  bus_ = bus;
}

bool Maneuver::IsTransit() const {
  return ((type_ == DirectionsLeg_Maneuver_Type_kTransit) ||
          (type_ == DirectionsLeg_Maneuver_Type_kTransitTransfer) ||
          (type_ == DirectionsLeg_Maneuver_Type_kTransitRemainOn));
}

const TransitRouteInfo& Maneuver::transit_info() const {
  return transit_info_;
}

TransitRouteInfo* Maneuver::mutable_transit_info() {
  return &transit_info_;
}

std::string Maneuver::GetTransitArrivalTime() const {
  return transit_info_.transit_stops.back().arrival_date_time();
}

std::string Maneuver::GetTransitDepartureTime() const {
  return transit_info_.transit_stops.front().departure_date_time();
}

const std::list<TransitPlatformInfo>& Maneuver::GetTransitStops() const {
  return transit_info_.transit_stops;
}

size_t Maneuver::GetTransitStopCount() const {
  return (transit_info_.transit_stops.size() > 0) ? (transit_info_.transit_stops.size() - 1) : 0;
}

void Maneuver::InsertTransitStop(const TransitPlatformInfo& transit_platform) {
  transit_info_.transit_stops.push_front(transit_platform);
  // TODO
  //  LOG_TRACE("InsertTransitPlatform=" +
  //  transit_info_.transit_platforms.front().ToParameterString());
}

const std::string& Maneuver::depart_instruction() const {
  return depart_instruction_;
}

void Maneuver::set_depart_instruction(const std::string& depart_instruction) {
  depart_instruction_ = depart_instruction;
}

void Maneuver::set_depart_instruction(std::string&& depart_instruction) {
  depart_instruction_ = std::move(depart_instruction);
}

const std::string& Maneuver::verbal_depart_instruction() const {
  return verbal_depart_instruction_;
}

void Maneuver::set_verbal_depart_instruction(const std::string& verbal_depart_instruction) {
  verbal_depart_instruction_ = verbal_depart_instruction;
}

void Maneuver::set_verbal_depart_instruction(std::string&& verbal_depart_instruction) {
  verbal_depart_instruction_ = std::move(verbal_depart_instruction);
}

const std::string& Maneuver::arrive_instruction() const {
  return arrive_instruction_;
}

void Maneuver::set_arrive_instruction(const std::string& arrive_instruction) {
  arrive_instruction_ = arrive_instruction;
}

void Maneuver::set_arrive_instruction(std::string&& arrive_instruction) {
  arrive_instruction_ = std::move(arrive_instruction);
}

const std::string& Maneuver::verbal_arrive_instruction() const {
  return verbal_arrive_instruction_;
}

void Maneuver::set_verbal_arrive_instruction(const std::string& verbal_arrive_instruction) {
  verbal_arrive_instruction_ = verbal_arrive_instruction;
}

void Maneuver::set_verbal_arrive_instruction(std::string&& verbal_arrive_instruction) {
  verbal_arrive_instruction_ = std::move(verbal_arrive_instruction);
}

const VerbalTextFormatter* Maneuver::verbal_formatter() const {
  return verbal_formatter_.get();
}

void Maneuver::set_verbal_formatter(std::unique_ptr<VerbalTextFormatter>&& verbal_formatter) {
  verbal_formatter_ = std::move(verbal_formatter);
}

#ifdef LOGGING_LEVEL_TRACE
std::string Maneuver::ToString() const {
  std::string man_str;
  man_str.reserve(256);

  man_str += "type_=";
  man_str += std::to_string(type_);

  man_str += " | street_names_=";
  man_str += street_names_->ToString();

  man_str += " | begin_street_names=";
  man_str += begin_street_names_->ToString();

  man_str += " | cross_street_names=";
  man_str += cross_street_names_->ToString();

  man_str += " | instruction=";
  man_str += instruction_;

  man_str += " | distance_=";
  man_str += std::to_string(length_);

  man_str += " | time=";
  man_str += std::to_string(time_);

  man_str += " | turn_degree=";
  man_str += std::to_string(turn_degree_);

  man_str += " | begin_relative_direction=";
  man_str += std::to_string(static_cast<int>(begin_relative_direction_));

  man_str += " | begin_cardinal_direction=";
  man_str += std::to_string(begin_cardinal_direction_);

  man_str += " | begin_heading=";
  man_str += std::to_string(begin_heading_);

  man_str += " | end_heading=";
  man_str += std::to_string(end_heading_);

  man_str += " | begin_node_index=";
  man_str += std::to_string(begin_node_index_);

  man_str += " | end_node_index=";
  man_str += std::to_string(end_node_index_);

  man_str += " | begin_shape_index=";
  man_str += std::to_string(begin_shape_index_);

  man_str += " | end_shape_index=";
  man_str += std::to_string(end_shape_index_);

  man_str += " | ramp=";
  man_str += std::to_string(ramp_);

  man_str += " | turn_channel=";
  man_str += std::to_string(turn_channel_);

  man_str += " | ferry=";
  man_str += std::to_string(ferry_);

  man_str += " | rail_ferry=";
  man_str += std::to_string(rail_ferry_);

  man_str += " | roundabout=";
  man_str += std::to_string(roundabout_);

  man_str += " | portions_toll=";
  man_str += std::to_string(portions_toll_);

  man_str += " | portions_unpaved=";
  man_str += std::to_string(portions_unpaved_);

  man_str += " | portions_highway=";
  man_str += std::to_string(portions_highway_);

  man_str += " | internal_intersection=";
  man_str += std::to_string(internal_intersection_);

  man_str += " | ";
  man_str += signs_.ToString();

  man_str += " | internal_right_turn_count=";
  man_str += std::to_string(internal_right_turn_count_);

  man_str += " | internal_left_turn_count=";
  man_str += std::to_string(internal_left_turn_count_);

  man_str += " | roundabout_exit_count=";
  man_str += std::to_string(roundabout_exit_count_);

  man_str += " | fork=";
  man_str += std::to_string(fork_);

  man_str += " | begin_intersecting_edge_name_consistency=";
  man_str += std::to_string(begin_intersecting_edge_name_consistency_);

  man_str += " | intersecting_forward_edge=";
  man_str += std::to_string(intersecting_forward_edge_);

  man_str += " | verbal_transition_alert_instruction=";
  man_str += verbal_transition_alert_instruction_;

  man_str += " | verbal_pre_transition_instruction=";
  man_str += verbal_pre_transition_instruction_;

  man_str += " | verbal_post_transition_instruction=";
  man_str += verbal_post_transition_instruction_;

  man_str += " | tee=";
  man_str += std::to_string(tee_);

  man_str += " | unnamed_walkway=";
  man_str += std::to_string(unnamed_walkway_);

  man_str += " | unnamed_cycleway=";
  man_str += std::to_string(unnamed_cycleway_);

  man_str += " | unnamed_mountain_bike_trail=";
  man_str += std::to_string(unnamed_mountain_bike_trail_);

  man_str += " | basic_time=";
  man_str += std::to_string(basic_time_);

  man_str += " | verbal_multi_cue=";
  man_str += std::to_string(verbal_multi_cue_);

  man_str += " | travel_mode=";
  man_str += std::to_string(travel_mode_);

  man_str += " | rail=";
  man_str += std::to_string(rail_);

  man_str += " | bus=";
  man_str += std::to_string(bus_);

  // TODO travel types

  man_str += " | transit_connection=";
  man_str += std::to_string(transit_connection_);

  // TODO: transit_connection_stop
  // TODO: transit_route_info
  // TODO
  //  std::string depart_instruction_;
  //  std::string verbal_depart_instruction_;
  //  std::string arrive_instruction_;
  //  std::string verbal_arrive_instruction_;

  return man_str;
}

// Used by ManeuverBuilder unit tests - therefore, order is important
std::string Maneuver::ToParameterString() const {
  const std::string delim = ", ";
  std::string man_str;
  man_str.reserve(256);

  man_str += "DirectionsLeg_Maneuver_Type_";
  man_str += DirectionsLeg_Maneuver_Type_Name(type_);

  man_str += delim;
  man_str += street_names_->ToParameterString();

  man_str += delim;
  man_str += begin_street_names_->ToParameterString();

  man_str += delim;
  man_str += cross_street_names_->ToParameterString();

  man_str += delim;
  man_str += "\"";
  man_str += instruction_;
  man_str += "\"";

  man_str += delim;
  man_str += std::to_string(length_);

  man_str += delim;
  man_str += std::to_string(time_);

  man_str += delim;
  man_str += std::to_string(turn_degree_);

  man_str += delim;
  man_str += relative_direction_string.find(static_cast<int>(begin_relative_direction_))->second;

  man_str += delim;
  man_str += "DirectionsLeg_Maneuver_CardinalDirection_";
  man_str += DirectionsLeg_Maneuver_CardinalDirection_Name(begin_cardinal_direction_);

  man_str += delim;
  man_str += std::to_string(begin_heading_);

  man_str += delim;
  man_str += std::to_string(end_heading_);

  man_str += delim;
  man_str += std::to_string(begin_node_index_);

  man_str += delim;
  man_str += std::to_string(end_node_index_);

  man_str += delim;
  man_str += std::to_string(begin_shape_index_);

  man_str += delim;
  man_str += std::to_string(end_shape_index_);

  man_str += delim;
  man_str += std::to_string(ramp_);

  man_str += delim;
  man_str += std::to_string(turn_channel_);

  man_str += delim;
  man_str += std::to_string(ferry_);

  man_str += delim;
  man_str += std::to_string(rail_ferry_);

  man_str += delim;
  man_str += std::to_string(roundabout_);

  man_str += delim;
  man_str += std::to_string(portions_toll_);

  man_str += delim;
  man_str += std::to_string(portions_unpaved_);

  man_str += delim;
  man_str += std::to_string(portions_highway_);

  man_str += delim;
  man_str += std::to_string(internal_intersection_);

  man_str += delim;
  man_str += signs_.ToParameterString();

  man_str += delim;
  man_str += std::to_string(internal_right_turn_count_);

  man_str += delim;
  man_str += std::to_string(internal_left_turn_count_);

  man_str += delim;
  man_str += std::to_string(roundabout_exit_count_);

  man_str += delim;
  man_str += std::to_string(fork_);

  man_str += delim;
  man_str += std::to_string(begin_intersecting_edge_name_consistency_);

  man_str += delim;
  man_str += std::to_string(intersecting_forward_edge_);

  man_str += delim;
  man_str += "\"";
  man_str += verbal_transition_alert_instruction_;
  man_str += "\"";

  man_str += delim;
  man_str += "\"";
  man_str += verbal_pre_transition_instruction_;
  man_str += "\"";

  man_str += delim;
  man_str += "\"";
  man_str += verbal_post_transition_instruction_;
  man_str += "\"";

  man_str += delim;
  man_str += std::to_string(tee_);

  man_str += delim;
  man_str += std::to_string(unnamed_walkway_);

  man_str += delim;
  man_str += std::to_string(unnamed_cycleway_);

  man_str += delim;
  man_str += std::to_string(unnamed_mountain_bike_trail_);

  man_str += delim;
  man_str += std::to_string(basic_time_);

  man_str += delim;
  man_str += std::to_string(verbal_multi_cue_);

  //  man_str += delim;
  //  man_str += "TripLeg_TravelMode_";
  //  man_str += TripLeg_TravelMode_descriptor()
  //      ->FindValueByNumber(travel_mode_)->name();
  //  bool rail_;
  //  bool bus_;

  // TODO Travel Types

  // Transit TODO
  // Transit connection flag and the associated stop
  //  transit_connection_;
  //  TransitPlatformInfo transit_connection_stop_;

  // The transit info including list of stops
  //  TransitInfo transit_info_;
  //  std::string depart_instruction_;
  //  std::string verbal_depart_instruction_;
  //  std::string arrive_instruction_;
  //  std::string verbal_arrive_instruction_;

  return man_str;
}
#endif

} // namespace odin
} // namespace valhalla
