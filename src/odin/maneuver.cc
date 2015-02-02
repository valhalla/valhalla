#include <iostream>

#include "odin/maneuver.h"

namespace valhalla {
namespace odin {

Maneuver::Maneuver()
    : type_(TripDirections_Maneuver_Type_kNone),
      distance_(0.0f),
      time_(0),
      turn_degree_(0),
      begin_cardinal_direction_(
          TripDirections_Maneuver_CardinalDirection_kNorth),
      begin_heading_(0),
      end_heading_(0),
      begin_node_index_(0),
      end_node_index_(0),
      begin_shape_index_(0),
      end_shape_index_(0),
      ramp_(false),
      ferry_(false),
      rail_ferry_(false),
      roundabout_(false),
      portions_toll_(false),
      portions_unpaved_(false),
      portions_highway_(false) {
}

const TripDirections_Maneuver_Type& Maneuver::type() const {
  return type_;
}

void Maneuver::set_type(const TripDirections_Maneuver_Type& type) {
  type_ = type;
}

const StreetNames& Maneuver::street_names() const {
  return street_names_;
}

StreetNames* Maneuver::mutable_street_names() {
  return &street_names_;
}

void Maneuver::set_street_names(const StreetNames& street_names) {
  street_names_ = street_names;
}

void Maneuver::set_street_names(StreetNames&& street_names) {
  street_names_ = std::move(street_names);
}

bool Maneuver::HasStreetNames() const {
  return (street_names_.size() > 0);
}

const StreetNames& Maneuver::begin_street_names() const {
  return begin_street_names_;
}

void Maneuver::set_begin_street_names(const StreetNames& begin_street_names) {
  begin_street_names_ = begin_street_names;
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

std::string Maneuver::ToString() const {
  std::string man_str;
  man_str.reserve(128);

  man_str += "type_=";
  man_str += std::to_string(type_);
  man_str += " | street_names_=";
  man_str += street_names_.ToString();
  man_str += " | distance_=";
  man_str += std::to_string(distance_);
  // TODO - others
  //man_str += " | TBD=";
  //man_str += TBD;

  return man_str;
}

}
}

