#ifndef VALHALLA_ODIN_MANEUVER_H_
#define VALHALLA_ODIN_MANEUVER_H_

#include <string>
#include <unordered_map>

#include <valhalla/proto/tripdirections.pb.h>
#include <valhalla/baldr/streetnames.h>
#include <valhalla/odin/signs.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace odin {

class Maneuver {
 public:
  enum class RelativeDirection {
    kNone,
    kKeepStraight,
    kKeepRight,
    kRight,
    KReverse,
    kLeft,
    kKeepLeft
  };

  Maneuver();

  const TripDirections_Maneuver_Type& type() const;
  void set_type(const TripDirections_Maneuver_Type& type);

  const StreetNames& street_names() const;
  StreetNames* mutable_street_names();
  void set_street_names(const StreetNames& street_names);
  void set_street_names(StreetNames&& street_names);
  bool HasStreetNames() const;

  const StreetNames& begin_street_names() const;
  StreetNames* mutable_begin_street_names();
  void set_begin_street_names(const StreetNames& begin_street_names);
  void set_begin_street_names(StreetNames&& begin_street_names);
  bool HasBeginStreetNames() const;

  const StreetNames& cross_street_names() const;
  StreetNames* mutable_cross_street_names();
  void set_cross_street_names(const StreetNames& cross_street_names);
  void set_cross_street_names(StreetNames&& cross_street_names);
  bool HasCrossStreetNames() const;

  const std::string& instruction() const;
  void set_instruction(const std::string& instruction);
  void set_instruction(std::string&& instruction);

  float distance() const;
  void set_distance(float distance);

  uint32_t time() const;
  void set_time(uint32_t time);

  uint32_t turn_degree() const;
  void set_turn_degree(uint32_t turn_degree);

  RelativeDirection begin_relative_direction() const;
  void set_begin_relative_direction(RelativeDirection begin_relative_direction);

  TripDirections_Maneuver_CardinalDirection begin_cardinal_direction() const;
  void set_begin_cardinal_direction(
      TripDirections_Maneuver_CardinalDirection begin_cardinal_direction);
  uint32_t begin_heading() const;
  void set_begin_heading(uint32_t beginHeading);

  uint32_t end_heading() const;
  void set_end_heading(uint32_t endHeading);

  uint32_t begin_node_index() const;
  void set_begin_node_index(uint32_t beginNodeIndex);

  uint32_t end_node_index() const;
  void set_end_node_index(uint32_t endNodeIndex);

  uint32_t begin_shape_index() const;
  void set_begin_shape_index(uint32_t beginShapeIndex);

  uint32_t end_shape_index() const;
  void set_end_shape_index(uint32_t endShapeIndex);

  bool ramp() const;
  void set_ramp(bool ramp);

  bool turn_channel() const;
  void set_turn_channel(bool ramp);

  bool ferry() const;
  void set_ferry(bool ferry);

  bool rail_ferry() const;
  void set_rail_ferry(bool rail_ferry);

  bool roundabout() const;
  void set_roundabout(bool roundabout);

  bool portions_toll() const;
  void set_portions_toll(bool portionsToll);

  bool portions_unpaved() const;
  void set_portions_unpaved(bool portionsUnpaved);

  bool portions_highway() const;
  void set_portions_highway(bool portionsHighway);

  bool internal_intersection() const;
  void set_internal_intersection(bool internal_intersection);
  bool HasUsableInternalIntersectionName() const;

  const Signs& signs() const;
  Signs* mutable_signs();

  bool HasExitSign() const;
  bool HasExitNumberSign() const;
  bool HasExitBranchSign() const;
  bool HasExitTowardSign() const;
  bool HasExitNameSign() const;

  uint32_t internal_right_turn_count() const;
  void set_internal_right_turn_count(uint32_t internal_right_turn_count);

  uint32_t internal_left_turn_count() const;
  void set_internal_left_turn_count(uint32_t internal_left_turn_count);

  std::string ToString() const;

  std::string ToParameterString() const;

 protected:
  TripDirections_Maneuver_Type type_;
  StreetNames street_names_;
  StreetNames begin_street_names_;
  StreetNames cross_street_names_;
  std::string instruction_;
  float distance_;
  uint32_t time_;
  uint32_t turn_degree_;
  RelativeDirection begin_relative_direction_;
  TripDirections_Maneuver_CardinalDirection begin_cardinal_direction_;
  uint32_t begin_heading_;
  uint32_t end_heading_;
  uint32_t begin_node_index_;
  uint32_t end_node_index_;
  uint32_t begin_shape_index_;
  uint32_t end_shape_index_;
  bool ramp_;
  bool turn_channel_;
  bool ferry_;
  bool rail_ferry_;
  bool roundabout_;
  bool portions_toll_;
  bool portions_unpaved_;
  bool portions_highway_;
  bool internal_intersection_;
  Signs signs_;
  uint32_t internal_right_turn_count_;
  uint32_t internal_left_turn_count_;

  // TODO notes

  static const std::unordered_map<int, std::string> relative_direction_string_;

};

}
}

#endif  // VALHALLA_ODIN_MANEUVER_H_
