#include <iostream>

#include "odin/maneuver.h"

#include <valhalla/midgard/util.h>
#include <valhalla/baldr/streetnames.h>
#include <valhalla/baldr/streetnames_us.h>

using namespace valhalla::odin;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace valhalla {
namespace odin {

const std::unordered_map<int, std::string> Maneuver::relative_direction_string_ =
    { { static_cast<int>(Maneuver::RelativeDirection::kNone),
        "Maneuver::RelativeDirection::kNone" }, {
        static_cast<int>(Maneuver::RelativeDirection::kKeepStraight),
        "Maneuver::RelativeDirection::kKeepStraight" }, {
        static_cast<int>(Maneuver::RelativeDirection::kKeepRight),
        "Maneuver::RelativeDirection::kKeepRight" }, {
        static_cast<int>(Maneuver::RelativeDirection::kRight),
        "Maneuver::RelativeDirection::kRight" }, {
        static_cast<int>(Maneuver::RelativeDirection::KReverse),
        "Maneuver::RelativeDirection::KReverse" }, {
        static_cast<int>(Maneuver::RelativeDirection::kLeft),
        "Maneuver::RelativeDirection::kLeft" }, {
        static_cast<int>(Maneuver::RelativeDirection::kKeepLeft),
        "Maneuver::RelativeDirection::kKeepLeft" } };

Maneuver::Maneuver()
    : type_(TripDirections_Maneuver_Type_kNone),
      distance_(0.0f),
      time_(0),
      turn_degree_(0),
      begin_relative_direction_(RelativeDirection::kNone),
      begin_cardinal_direction_(
          TripDirections_Maneuver_CardinalDirection_kNorth),
      begin_heading_(0),
      end_heading_(0),
      begin_node_index_(0),
      end_node_index_(0),
      begin_shape_index_(0),
      end_shape_index_(0),
      ramp_(false),
      turn_channel_(false),
      ferry_(false),
      rail_ferry_(false),
      roundabout_(false),
      portions_toll_(false),
      portions_unpaved_(false),
      portions_highway_(false),
      internal_intersection_(false),
      internal_right_turn_count_(0),
      internal_left_turn_count_(0),
      roundabout_exit_count_(0) {
  street_names_ = make_unique<StreetNamesUs>();
  begin_street_names_ = make_unique<StreetNamesUs>();
  cross_street_names_ = make_unique<StreetNamesUs>();
}

const TripDirections_Maneuver_Type& Maneuver::type() const {
  return type_;
}

void Maneuver::set_type(const TripDirections_Maneuver_Type& type) {
  type_ = type;
}

const StreetNames& Maneuver::street_names() const {
  return *street_names_;
}

void Maneuver::set_street_names(const std::vector<std::string>& names) {
  street_names_ = make_unique<StreetNamesUs>(names);
}

void Maneuver::set_street_names(std::unique_ptr<StreetNames>&& street_names) {
  street_names_ = std::move(street_names);
}

bool Maneuver::HasStreetNames() const {
  return (!street_names_->empty());
}

const StreetNames& Maneuver::begin_street_names() const {
  return *begin_street_names_;
}

void Maneuver::set_begin_street_names(const std::vector<std::string>& names) {
  begin_street_names_ = make_unique<StreetNamesUs>(names);
}

void Maneuver::set_begin_street_names(
    std::unique_ptr<StreetNames>&& begin_street_names) {
  begin_street_names_ = std::move(begin_street_names);
}

bool Maneuver::HasBeginStreetNames() const {
  return (!begin_street_names_->empty());
}

const StreetNames& Maneuver::cross_street_names() const {
  return *cross_street_names_;
}

void Maneuver::set_cross_street_names(const std::vector<std::string>& names) {
  cross_street_names_ = make_unique<StreetNamesUs>(names);
}

void Maneuver::set_cross_street_names(
    std::unique_ptr<StreetNames>&& cross_street_names) {
  cross_street_names_ = std::move(cross_street_names);
}

bool Maneuver::HasCrossStreetNames() const {
  return (!cross_street_names_->empty());
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

float Maneuver::distance() const {
  return distance_;
}

void Maneuver::set_distance(float distance) {
  distance_ = distance;
}

uint32_t Maneuver::time() const {
  return time_;
}

void Maneuver::set_time(uint32_t time) {
  time_ = time;
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

void Maneuver::set_begin_relative_direction(
    RelativeDirection begin_relative_direction) {
  begin_relative_direction_ = begin_relative_direction;
}

TripDirections_Maneuver_CardinalDirection Maneuver::begin_cardinal_direction() const {
  return begin_cardinal_direction_;
}

void Maneuver::set_begin_cardinal_direction(
    TripDirections_Maneuver_CardinalDirection begin_cardinal_direction) {
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
  if (internal_intersection_ && !street_names_->empty()
      && ((link_count == 1) || (link_count == 3))) {
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

void Maneuver::set_internal_right_turn_count(
    uint32_t internal_right_turn_count) {
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
  man_str += std::to_string(distance_);

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

  return man_str;
}

std::string Maneuver::ToParameterString() const {
  const std::string delim = ", ";
  std::string man_str;
  man_str.reserve(256);

  man_str += "TripDirections_Maneuver_Type_";
  man_str += TripDirections_Maneuver_Type_descriptor()->FindValueByNumber(type_)
      ->name();

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
  man_str += std::to_string(distance_);

  man_str += delim;
  man_str += std::to_string(time_);

  man_str += delim;
  man_str += std::to_string(turn_degree_);

  man_str += delim;
  man_str += Maneuver::relative_direction_string_.find(
      static_cast<int>(begin_relative_direction_))->second;

  man_str += delim;
  man_str += "TripDirections_Maneuver_CardinalDirection_";
  man_str += TripDirections_Maneuver_CardinalDirection_descriptor()
      ->FindValueByNumber(begin_cardinal_direction_)->name();

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

  return man_str;
}

}
}

