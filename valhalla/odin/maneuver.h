#ifndef VALHALLA_ODIN_MANEUVER_H_
#define VALHALLA_ODIN_MANEUVER_H_

#include <string>

#include <valhalla/proto/tripdirections.pb.h>
#include <valhalla/odin/streetnames.h>

namespace valhalla {
namespace odin {

class Maneuver {
 public:
  Maneuver();

  const TripDirections_Maneuver_Type& type() const;
  void set_type(const TripDirections_Maneuver_Type& type);

  const StreetNames& street_names() const;
  StreetNames* mutable_street_names();
  void set_street_names(const StreetNames& street_names);

  bool HasStreetNames() const;

  const StreetNames& begin_street_names() const;
  void set_begin_street_names(const StreetNames& begin_street_names);

  const std::string& instruction() const;
  void set_instruction(const std::string& instruction);
  void set_instruction(std::string&& instruction);

  float distance() const;
  void set_distance(float distance);

  uint32_t time() const;
  void set_time(uint32_t time);

  uint32_t turn_degree() const;
  void set_turn_degree(uint32_t turn_degree);

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

  bool portions_toll() const;
  void set_portions_toll(bool portionsToll);

  bool portions_unpaved() const;
  void set_portions_unpaved(bool portionsUnpaved);

  std::string ToString() const;

 protected:
  TripDirections_Maneuver_Type type_;
  StreetNames street_names_;
  StreetNames begin_street_names_;
  std::string instruction_;
  float distance_;
  uint32_t time_;
  uint32_t turn_degree_;
  TripDirections_Maneuver_CardinalDirection begin_cardinal_direction_;
  uint32_t begin_heading_;
  uint32_t end_heading_;
  uint32_t begin_node_index_;
  uint32_t end_node_index_;
  uint32_t begin_shape_index_;
  uint32_t end_shape_index_;
  bool ramp_;
  bool portions_toll_;
  bool portions_unpaved_;

  // TODO notes

//  GDG
//  optional Type type = 1;                                  // Maneuver type
//  optional string text_instruction = 2;                    // instruction text
//  repeated string street_name = 3;                         // street names
//  optional float length = 4;                               // km (TODO: based on type)
//  optional uint32 time = 5;                                // seconds
//  optional CardinalDirection begin_cardinal_direction = 6; // CardinalDirection
//  optional uint32 begin_heading = 7;                       // 0-360
//  optional uint32 begin_shape_index = 8;                   // inclusive index
//  optional uint32 end_shape_index = 9;                     // inclusive index
//  optional bool portions_toll = 10;                        // has portions toll
//  optional bool portions_unpaved = 11;                     // has portions unpaved
};

}
}

#endif  // VALHALLA_ODIN_MANEUVER_H_
