#include <iostream>
#include <list>
#include <utility>

#include "baldr/streetnames.h"
#include "baldr/streetnames_us.h"
#include "midgard/constants.h"
#include "midgard/logging.h"
#include "midgard/util.h"

#include "odin/maneuver.h"
#include "odin/transitrouteinfo.h"

#include "proto/common.pb.h"
#include "proto/directions.pb.h"
#include "proto/options.pb.h"

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
  static const std::unordered_map<int, std::string> values{{0, "kNone"},
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
                                                           {39, "kElevatorEnter"},
                                                           {40, "kStepsEnter"},
                                                           {41, "kEscalatorEnter"},
                                                           {42, "kBuildingEnter"},
                                                           {43, "kBuildingExit"}};
  auto f = values.find(v);
  if (f == values.cend())
    throw std::runtime_error(
        "Missing DirectionsLeg_Maneuver_Type_Name value in protobuf enum to string");
  return f->second;
}

const std::string& DirectionsLeg_Maneuver_CardinalDirection_Name(int v) {
  static const std::unordered_map<int, std::string> values{{0, "kNorth"}, {1, "kNorthEast"},
                                                           {2, "kEast"},  {3, "kSouthEast"},
                                                           {4, "kSouth"}, {5, "kSouthWest"},
                                                           {6, "kWest"},  {7, "kNorthWest"}};
  auto f = values.find(v);
  if (f == values.cend())
    throw std::runtime_error(
        "Missing DirectionsLeg_Maneuver_CardinalDirection_Name value in protobuf enum to string");
  return f->second;
}

const std::string& TrailType_Name(int v) {
  static const std::unordered_map<int, std::string> values{{0, "kNone"},
                                                           {1, "kNamedCycleway"},
                                                           {2, "kUnnamedCycleway"},
                                                           {3, "kNamedWalkway"},
                                                           {4, "kUnnamedWalkway"},
                                                           {5, "kNamedMtbTrail"},
                                                           {6, "kUnnamedMtbTrail"}};
  auto f = values.find(v);
  if (f == values.cend())
    throw std::runtime_error("Missing TrailType_Name value in protobuf enum to string");
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
      travel_mode_(TravelMode::kDrive), vehicle_type_(VehicleType::kCar),
      pedestrian_type_(PedestrianType::kFoot), bicycle_type_(BicycleType::kRoad),
      transit_type_(TransitType::kRail), transit_connection_(false), rail_(false), bus_(false),
      fork_(false), begin_intersecting_edge_name_consistency_(false),
      intersecting_forward_edge_(false), tee_(false), trail_type_(TrailType::kNone),
      imminent_verbal_multi_cue_(false), distant_verbal_multi_cue_(false), to_stay_on_(false),
      drive_on_right_(true), has_time_restrictions_(false),
      has_right_traversable_outbound_intersecting_edge_(false),
      has_left_traversable_outbound_intersecting_edge_(false),
      bss_maneuver_type_(DirectionsLeg_Maneuver_BssManeuverType_kNoneAction),
      include_verbal_pre_transition_length_(false), contains_obvious_maneuver_(false),
      roundabout_exit_count_(0), has_combined_enter_exit_roundabout_(false), roundabout_length_(0.0f),
      roundabout_exit_length_(0.0f), roundabout_exit_begin_heading_(0),
      roundabout_exit_turn_degree_(0), roundabout_exit_shape_index_(0),
      has_collapsed_small_end_ramp_fork_(false), has_collapsed_merge_maneuver_(false),
      pedestrian_crossing_(false), has_long_street_name_(false), elevator_(false),
      indoor_steps_(false), escalator_(false), building_enter_(false), building_exit_(false),
      end_level_ref_("") {
  street_names_ = std::make_unique<StreetNames>();
  begin_street_names_ = std::make_unique<StreetNames>();
  cross_street_names_ = std::make_unique<StreetNames>();
  roundabout_exit_street_names_ = std::make_unique<StreetNames>();
  roundabout_exit_begin_street_names_ = std::make_unique<StreetNames>();
}

const DirectionsLeg_Maneuver_Type& Maneuver::type() const {
  return type_;
}

void Maneuver::set_type(const DirectionsLeg_Maneuver_Type& type) {
  type_ = type;
}

bool Maneuver::IsStartType() const {
  return ((type_ == DirectionsLeg_Maneuver_Type_kStart) ||
          (type_ == DirectionsLeg_Maneuver_Type_kStartLeft) ||
          (type_ == DirectionsLeg_Maneuver_Type_kStartRight));
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

bool Maneuver::IsLeftType() const {
  return ((type_ == DirectionsLeg_Maneuver_Type_kSlightLeft ||
           type_ == DirectionsLeg_Maneuver_Type_kLeft ||
           type_ == DirectionsLeg_Maneuver_Type_kSharpLeft ||
           type_ == DirectionsLeg_Maneuver_Type_kUturnLeft ||
           type_ == DirectionsLeg_Maneuver_Type_kRampLeft ||
           type_ == DirectionsLeg_Maneuver_Type_kExitLeft ||
           type_ == DirectionsLeg_Maneuver_Type_kStayLeft ||
           type_ == DirectionsLeg_Maneuver_Type_kDestinationLeft ||
           type_ == DirectionsLeg_Maneuver_Type_kMergeLeft));
}

bool Maneuver::IsRightType() const {
  return ((type_ == DirectionsLeg_Maneuver_Type_kSlightRight ||
           type_ == DirectionsLeg_Maneuver_Type_kRight ||
           type_ == DirectionsLeg_Maneuver_Type_kSharpRight ||
           type_ == DirectionsLeg_Maneuver_Type_kUturnRight ||
           type_ == DirectionsLeg_Maneuver_Type_kRampRight ||
           type_ == DirectionsLeg_Maneuver_Type_kExitRight ||
           type_ == DirectionsLeg_Maneuver_Type_kStayRight ||
           type_ == DirectionsLeg_Maneuver_Type_kDestinationRight ||
           type_ == DirectionsLeg_Maneuver_Type_kMergeRight));
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

void Maneuver::set_length(float km_length) {
  length_ = km_length;
}

double Maneuver::time() const {
  return time_;
}

void Maneuver::set_time(double time) {
  time_ = time;
}

double Maneuver::basic_time() const {
  return basic_time_;
}

void Maneuver::set_basic_time(double basic_time) {
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

bool Maneuver::HasSigns() const {
  return (HasExitSign() || HasGuideSign() || HasJunctionNameSign());
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

bool Maneuver::HasGuideSign() const {
  return signs_.HasGuide();
}

bool Maneuver::HasGuideBranchSign() const {
  return signs_.HasGuideBranch();
}

bool Maneuver::HasGuideTowardSign() const {
  return signs_.HasGuideToward();
}

bool Maneuver::HasJunctionNameSign() const {
  return signs_.HasJunctionName();
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

const std::string& Maneuver::verbal_succinct_transition_instruction() const {
  return verbal_succinct_transition_instruction_;
}

void Maneuver::set_verbal_succinct_transition_instruction(
    const std::string& verbal_succinct_transition_instruction) {
  verbal_succinct_transition_instruction_ = verbal_succinct_transition_instruction;
}

void Maneuver::set_verbal_succinct_transition_instruction(
    std::string&& verbal_succinct_transition_instruction) {
  verbal_succinct_transition_instruction_ = std::move(verbal_succinct_transition_instruction);
}

bool Maneuver::HasVerbalSuccinctTransitionInstruction() const {
  return (!verbal_succinct_transition_instruction_.empty());
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

TrailType Maneuver::trail_type() const {
  return trail_type_;
}

void Maneuver::set_trail_type(const TrailType trail) {
  trail_type_ = trail;
}

bool Maneuver::is_walkway() const {
  return trail_type_ == TrailType::kNamedWalkway || trail_type_ == TrailType::kUnnamedWalkway;
}

bool Maneuver::unnamed_walkway() const {
  return trail_type_ == TrailType::kUnnamedWalkway;
}

bool Maneuver::pedestrian_crossing() const {
  return pedestrian_crossing_;
}

void Maneuver::set_pedestrian_crossing(bool pedestrian_crossing) {
  pedestrian_crossing_ = pedestrian_crossing;
}

bool Maneuver::is_cycleway() const {
  return trail_type_ == TrailType::kNamedCycleway || trail_type_ == TrailType::kUnnamedCycleway;
}

bool Maneuver::unnamed_cycleway() const {
  return trail_type_ == TrailType::kUnnamedCycleway;
}

bool Maneuver::is_mountain_bike_trail() const {
  return trail_type_ == TrailType::kNamedMtbTrail || trail_type_ == TrailType::kUnnamedMtbTrail;
}

bool Maneuver::unnamed_mountain_bike_trail() const {
  return trail_type_ == TrailType::kUnnamedMtbTrail;
}

bool Maneuver::imminent_verbal_multi_cue() const {
  return imminent_verbal_multi_cue_;
}

void Maneuver::set_imminent_verbal_multi_cue(bool imminent_verbal_multi_cue) {
  imminent_verbal_multi_cue_ = imminent_verbal_multi_cue;
}

bool Maneuver::distant_verbal_multi_cue() const {
  return distant_verbal_multi_cue_;
}

void Maneuver::set_distant_verbal_multi_cue(bool distant_verbal_multi_cue) {
  distant_verbal_multi_cue_ = distant_verbal_multi_cue;
}

bool Maneuver::HasVerbalMultiCue() const {
  return (imminent_verbal_multi_cue_ || distant_verbal_multi_cue_);
}

bool Maneuver::to_stay_on() const {
  return to_stay_on_;
}

void Maneuver::set_to_stay_on(bool to_stay_on) {
  to_stay_on_ = to_stay_on;
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

bool Maneuver::has_time_restrictions() const {
  return has_time_restrictions_;
}

void Maneuver::set_has_time_restrictions(bool has_time_restrictions) {
  has_time_restrictions_ = has_time_restrictions;
}

bool Maneuver::has_right_traversable_outbound_intersecting_edge() const {
  return has_right_traversable_outbound_intersecting_edge_;
}

void Maneuver::set_has_right_traversable_outbound_intersecting_edge(
    bool has_right_traversable_outbound_intersecting_edge) {
  has_right_traversable_outbound_intersecting_edge_ =
      has_right_traversable_outbound_intersecting_edge;
}

bool Maneuver::has_left_traversable_outbound_intersecting_edge() const {
  return has_left_traversable_outbound_intersecting_edge_;
}

void Maneuver::set_has_left_traversable_outbound_intersecting_edge(
    bool has_left_traversable_outbound_intersecting_edge) {
  has_left_traversable_outbound_intersecting_edge_ = has_left_traversable_outbound_intersecting_edge;
}

bool Maneuver::include_verbal_pre_transition_length() const {
  return include_verbal_pre_transition_length_;
}

void Maneuver::set_include_verbal_pre_transition_length(bool include_verbal_pre_transition_length) {
  include_verbal_pre_transition_length_ = include_verbal_pre_transition_length;
}

bool Maneuver::contains_obvious_maneuver() const {
  return contains_obvious_maneuver_;
}

void Maneuver::set_contains_obvious_maneuver(bool contains_obvious_maneuver) {
  contains_obvious_maneuver_ = contains_obvious_maneuver;
}

uint32_t Maneuver::roundabout_exit_count() const {
  return roundabout_exit_count_;
}

void Maneuver::set_roundabout_exit_count(uint32_t roundabout_exit_count) {
  roundabout_exit_count_ = roundabout_exit_count;
}

bool Maneuver::has_combined_enter_exit_roundabout() const {
  return has_combined_enter_exit_roundabout_;
}

void Maneuver::set_has_combined_enter_exit_roundabout(bool has_combined_enter_exit_roundabout) {
  has_combined_enter_exit_roundabout_ = has_combined_enter_exit_roundabout;
}

float Maneuver::roundabout_length(const Options::Units& units) const {
  if (units == Options::miles) {
    return (roundabout_length_ * midgard::kMilePerKm);
  }
  return roundabout_length_;
}

void Maneuver::set_roundabout_length(float roundabout_km_length) {
  roundabout_length_ = roundabout_km_length;
}

float Maneuver::roundabout_exit_length(const Options::Units& units) const {
  if (units == Options::miles) {
    return (roundabout_exit_length_ * midgard::kMilePerKm);
  }
  return roundabout_exit_length_;
}

void Maneuver::set_roundabout_exit_length(float roundabout_exit_km_length) {
  roundabout_exit_length_ = roundabout_exit_km_length;
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

const StreetNames& Maneuver::roundabout_exit_begin_street_names() const {
  return *roundabout_exit_begin_street_names_;
}

void Maneuver::set_roundabout_exit_begin_street_names(
    const std::vector<std::pair<std::string, bool>>& names) {
  roundabout_exit_begin_street_names_ = std::make_unique<StreetNamesUs>(names);
}

void Maneuver::set_roundabout_exit_begin_street_names(
    std::unique_ptr<StreetNames>&& roundabout_exit_begin_street_names) {
  roundabout_exit_begin_street_names_ = std::move(roundabout_exit_begin_street_names);
}

bool Maneuver::HasRoundaboutExitBeginStreetNames() const {
  return (!roundabout_exit_begin_street_names_->empty());
}

void Maneuver::ClearRoundaboutExitBeginStreetNames() {
  roundabout_exit_begin_street_names_->clear();
}

const Signs& Maneuver::roundabout_exit_signs() const {
  return roundabout_exit_signs_;
}

Signs* Maneuver::mutable_roundabout_exit_signs() {
  return &roundabout_exit_signs_;
}

uint32_t Maneuver::roundabout_exit_begin_heading() const {
  return roundabout_exit_begin_heading_;
}

void Maneuver::set_roundabout_exit_begin_heading(uint32_t beginHeading) {
  roundabout_exit_begin_heading_ = beginHeading;
}

uint32_t Maneuver::roundabout_exit_turn_degree() const {
  return roundabout_exit_turn_degree_;
}

void Maneuver::set_roundabout_exit_turn_degree(uint32_t turnDegree) {
  roundabout_exit_turn_degree_ = turnDegree;
}

uint32_t Maneuver::roundabout_exit_shape_index() const {
  return roundabout_exit_shape_index_;
}

void Maneuver::set_roundabout_exit_shape_index(uint32_t roundabout_exit_shape_index) {
  roundabout_exit_shape_index_ = roundabout_exit_shape_index;
}

bool Maneuver::has_collapsed_small_end_ramp_fork() const {
  return has_collapsed_small_end_ramp_fork_;
}

void Maneuver::set_has_collapsed_small_end_ramp_fork(bool has_collapsed_small_end_ramp_fork) {
  has_collapsed_small_end_ramp_fork_ = has_collapsed_small_end_ramp_fork;
}

bool Maneuver::has_collapsed_merge_maneuver() const {
  return has_collapsed_merge_maneuver_;
}

void Maneuver::set_has_collapsed_merge_maneuver(bool has_collapsed_merge_maneuver) {
  has_collapsed_merge_maneuver_ = has_collapsed_merge_maneuver;
}

TravelMode Maneuver::travel_mode() const {
  return travel_mode_;
}

void Maneuver::set_travel_mode(TravelMode travel_mode) {
  travel_mode_ = travel_mode;
}

VehicleType Maneuver::vehicle_type() const {
  return vehicle_type_;
}

void Maneuver::set_vehicle_type(VehicleType vehicle_type) {
  vehicle_type_ = vehicle_type;
}

PedestrianType Maneuver::pedestrian_type() const {
  return pedestrian_type_;
}

void Maneuver::set_pedestrian_type(PedestrianType pedestrian_type) {
  pedestrian_type_ = pedestrian_type;
}

BicycleType Maneuver::bicycle_type() const {
  return bicycle_type_;
}

void Maneuver::set_bicycle_type(BicycleType bicycle_type) {
  bicycle_type_ = bicycle_type;
}

TransitType Maneuver::transit_type() const {
  return transit_type_;
}

void Maneuver::set_transit_type(TransitType transit_type) {
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

const std::vector<DirectionsLeg_GuidanceView>& Maneuver::guidance_views() const {
  return guidance_views_;
}

std::vector<DirectionsLeg_GuidanceView>* Maneuver::mutable_guidance_views() {
  return &guidance_views_;
}

DirectionsLeg_Maneuver_BssManeuverType Maneuver::bss_maneuver_type() const {
  return bss_maneuver_type_;
}

void Maneuver::set_bss_maneuver_type(DirectionsLeg_Maneuver_BssManeuverType type) {
  bss_maneuver_type_ = type;
}

bool Maneuver::has_long_street_name() const {

  return has_long_street_name_;
}

void Maneuver::set_long_street_name(bool has_long_street_name) {
  has_long_street_name_ = has_long_street_name;
}

const BikeShareStationInfo& Maneuver::bss_info() const {
  return bss_info_;
}

void Maneuver::set_bss_info(const BikeShareStationInfo& bss_info) {
  bss_info_ = bss_info;
}

bool Maneuver::indoor_steps() const {
  return indoor_steps_;
}

void Maneuver::set_indoor_steps(bool indoor_steps) {
  indoor_steps_ = indoor_steps;
}

bool Maneuver::elevator() const {
  return elevator_;
}

void Maneuver::set_elevator(bool elevator) {
  elevator_ = elevator;
}

bool Maneuver::escalator() const {
  return escalator_;
}

void Maneuver::set_escalator(bool escalator) {
  escalator_ = escalator;
}

bool Maneuver::building_enter() const {
  return building_enter_;
}

void Maneuver::set_building_enter(bool building_enter) {
  building_enter_ = building_enter;
}

bool Maneuver::building_exit() const {
  return building_exit_;
}

void Maneuver::set_building_exit(bool building_exit) {
  building_exit_ = building_exit;
}

std::string Maneuver::end_level_ref() const {
  return end_level_ref_;
}

void Maneuver::set_end_level_ref(std::string end_level_ref) {
  end_level_ref_ = std::move(end_level_ref);
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

  man_str += " | fork=";
  man_str += std::to_string(fork_);

  man_str += " | begin_intersecting_edge_name_consistency=";
  man_str += std::to_string(begin_intersecting_edge_name_consistency_);

  man_str += " | intersecting_forward_edge=";
  man_str += std::to_string(intersecting_forward_edge_);

  man_str += " | verbal_succinct_transition_instruction=";
  man_str += verbal_succinct_transition_instruction_;

  man_str += " | verbal_transition_alert_instruction=";
  man_str += verbal_transition_alert_instruction_;

  man_str += " | verbal_pre_transition_instruction=";
  man_str += verbal_pre_transition_instruction_;

  man_str += " | verbal_post_transition_instruction=";
  man_str += verbal_post_transition_instruction_;

  man_str += " | tee=";
  man_str += std::to_string(tee_);

  man_str += " | trail_type=";
  man_str += TrailType_Name(static_cast<int>(trail_type_));

  man_str += " | unnamed_walkway=";
  man_str += std::to_string(unnamed_walkway());

  man_str += " | unnamed_cycleway=";
  man_str += std::to_string(unnamed_cycleway());

  man_str += " | unnamed_mountain_bike_trail=";
  man_str += std::to_string(unnamed_mountain_bike_trail());

  man_str += " | basic_time=";
  man_str += std::to_string(basic_time_);

  man_str += " | imminent_verbal_multi_cue=";
  man_str += std::to_string(imminent_verbal_multi_cue_);

  man_str += " | distant_verbal_multi_cue=";
  man_str += std::to_string(distant_verbal_multi_cue_);

  man_str += " | contains_obvious_maneuver=";
  man_str += std::to_string(contains_obvious_maneuver_);

  man_str += " | roundabout_exit_count=";
  man_str += std::to_string(roundabout_exit_count_);

  man_str += " | has_combined_enter_exit_roundabout=";
  man_str += std::to_string(has_combined_enter_exit_roundabout_);

  man_str += " | roundabout_length=";
  man_str += std::to_string(roundabout_length_);

  man_str += " | roundabout_exit_length=";
  man_str += std::to_string(roundabout_exit_length_);

  man_str += " | roundabout_exit_begin_heading=";
  man_str += std::to_string(roundabout_exit_begin_heading_);

  man_str += " | roundabout_exit_turn_degree=";
  man_str += std::to_string(roundabout_exit_turn_degree_);

  man_str += " | roundabout_exit_shape_index=";
  man_str += std::to_string(roundabout_exit_shape_index_);

  man_str += " | has_collapsed_small_end_ramp_fork=";
  man_str += std::to_string(has_collapsed_small_end_ramp_fork_);

  man_str += " | has_collapsed_merge_maneuver=";
  man_str += std::to_string(has_collapsed_merge_maneuver_);

  man_str += " | travel_mode=";
  man_str += std::to_string(travel_mode_);

  man_str += " | rail=";
  man_str += std::to_string(rail_);

  man_str += " | bus=";
  man_str += std::to_string(bus_);

  man_str += " | has_long_street_name=";
  man_str += std::to_string(has_long_street_name_);

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

  man_str += " | guidance_views=";
  for (const auto& guidance_view : guidance_views_) {
    man_str += "[";
    man_str += " data_id=" + guidance_view.data_id();
    man_str += " type=" + std::to_string(guidance_view.type());
    man_str += " base_id=" + guidance_view.base_id();
    man_str += " overlay_ids=";
    for (const auto& overlay_id : guidance_view.overlay_ids()) {
      man_str += overlay_id + " ";
    }
    man_str += "]";
  }

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
  man_str += std::to_string(fork_);

  man_str += delim;
  man_str += std::to_string(begin_intersecting_edge_name_consistency_);

  man_str += delim;
  man_str += std::to_string(intersecting_forward_edge_);

  man_str += delim;
  man_str += std::to_string(roundabout_exit_count_);

  man_str += delim;
  man_str += std::to_string(has_combined_enter_exit_roundabout_);

  man_str += delim;
  man_str += std::to_string(roundabout_length_);

  man_str += delim;
  man_str += std::to_string(roundabout_exit_length_);

  man_str += delim;
  man_str += std::to_string(roundabout_exit_begin_heading_);

  man_str += delim;
  man_str += std::to_string(roundabout_exit_turn_degree_);

  man_str += delim;
  man_str += std::to_string(roundabout_exit_shape_index_);

  man_str += delim;
  man_str += "\"";
  man_str += verbal_succinct_transition_instruction_;
  man_str += "\"";

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
  man_str += TrailType_Name(static_cast<int>(trail_type_));

  man_str += delim;
  man_str += std::to_string(unnamed_walkway());

  man_str += delim;
  man_str += std::to_string(unnamed_cycleway());

  man_str += delim;
  man_str += std::to_string(unnamed_mountain_bike_trail());

  man_str += delim;
  man_str += std::to_string(basic_time_);

  man_str += delim;
  man_str += std::to_string(imminent_verbal_multi_cue_);

  //  man_str += delim;
  //  man_str += "TravelMode_";
  //  man_str += TravelMode_descriptor()
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
