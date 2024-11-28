#ifndef VALHALLA_ODIN_MANEUVER_H_
#define VALHALLA_ODIN_MANEUVER_H_

#include <cstdint>
#include <list>
#include <memory>
#include <string>

#include <valhalla/baldr/streetnames.h>
#include <valhalla/baldr/verbal_text_formatter.h>

#include <valhalla/odin/signs.h>
#include <valhalla/odin/transitrouteinfo.h>
#include <valhalla/proto/common.pb.h>
#include <valhalla/proto/directions.pb.h>
#include <valhalla/proto/options.pb.h>
#include <valhalla/proto/trip.pb.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace odin {

// Trail type - for cycleways, walkways, mountain bike trails
enum class TrailType {
  kNone,
  kNamedCycleway,
  kUnnamedCycleway,
  kNamedWalkway,
  kUnnamedWalkway,
  kNamedMtbTrail,
  kUnnamedMtbTrail
};

/**
 * This class is a utility class that is used during the creation of the
 * maneuver list that will be used to populate the trip directions.
 */
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

  const DirectionsLeg_Maneuver_Type& type() const;
  void set_type(const DirectionsLeg_Maneuver_Type& type);
  bool IsStartType() const;
  bool IsDestinationType() const;
  bool IsMergeType() const;
  bool IsRightType() const;
  bool IsLeftType() const;

  void set_node_type(TripLeg_Node_Type type);
  TripLeg_Node_Type node_type() const;
  bool has_node_type() const;
  bool traffic_signal() const;
  void set_traffic_signal(bool traffic_signal);
  bool is_steps() const;
  void set_steps(bool steps);
  bool is_bridge() const;
  void set_bridge(bool bridge);
  bool is_tunnel() const;
  void set_tunnel(bool tunnel);

  const StreetNames& street_names() const;
  void set_street_names(const std::vector<std::pair<std::string, bool>>& names);
  void set_street_names(std::unique_ptr<StreetNames>&& street_names);
  bool HasStreetNames() const;
  void ClearStreetNames();

  bool HasSameNames(const Maneuver* other_maneuver,
                    bool allow_begin_intersecting_edge_name_consistency = false) const;

  bool HasSimilarNames(const Maneuver* other_maneuver,
                       bool allow_begin_intersecting_edge_name_consistency = false) const;

  const StreetNames& begin_street_names() const;
  void set_begin_street_names(const std::vector<std::pair<std::string, bool>>& names);
  void set_begin_street_names(std::unique_ptr<StreetNames>&& begin_street_names);
  bool HasBeginStreetNames() const;
  void ClearBeginStreetNames();

  const StreetNames& cross_street_names() const;
  void set_cross_street_names(const std::vector<std::pair<std::string, bool>>& names);
  void set_cross_street_names(std::unique_ptr<StreetNames>&& cross_street_names);
  bool HasCrossStreetNames() const;
  void ClearCrossStreetNames();

  const std::string& instruction() const;
  void set_instruction(const std::string& instruction);
  void set_instruction(std::string&& instruction);

  float length(const Options::Units& units = Options::kilometers) const;
  void set_length(float km_length); // Kilometers

  // Seconds
  double time() const;
  void set_time(double time);

  // len/speed on each edge with no stop impact in seconds
  double basic_time() const;
  void set_basic_time(double basic_time);

  uint32_t turn_degree() const;
  void set_turn_degree(uint32_t turn_degree);

  RelativeDirection begin_relative_direction() const;
  void set_begin_relative_direction(RelativeDirection begin_relative_direction);

  DirectionsLeg_Maneuver_CardinalDirection begin_cardinal_direction() const;
  void
  set_begin_cardinal_direction(DirectionsLeg_Maneuver_CardinalDirection begin_cardinal_direction);

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

  bool HasSigns() const;

  bool HasExitSign() const;
  bool HasExitNumberSign() const;
  bool HasExitBranchSign() const;
  bool HasExitTowardSign() const;
  bool HasExitNameSign() const;

  bool HasGuideSign() const;
  bool HasGuideBranchSign() const;
  bool HasGuideTowardSign() const;

  bool HasJunctionNameSign() const;

  uint32_t internal_right_turn_count() const;
  void set_internal_right_turn_count(uint32_t internal_right_turn_count);

  uint32_t internal_left_turn_count() const;
  void set_internal_left_turn_count(uint32_t internal_left_turn_count);

  bool fork() const;
  void set_fork(bool fork);

  bool begin_intersecting_edge_name_consistency() const;
  void set_begin_intersecting_edge_name_consistency(bool begin_intersecting_edge_name_consistency);

  bool intersecting_forward_edge() const;
  void set_intersecting_forward_edge(bool intersecting_forward_edge);

  const std::string& verbal_succinct_transition_instruction() const;
  void set_verbal_succinct_transition_instruction(
      const std::string& verbal_succinct_transition_instruction);
  void
  set_verbal_succinct_transition_instruction(std::string&& verbal_succinct_transition_instruction);
  bool HasVerbalSuccinctTransitionInstruction() const;

  const std::string& verbal_transition_alert_instruction() const;
  void
  set_verbal_transition_alert_instruction(const std::string& verbal_transition_alert_instruction);
  void set_verbal_transition_alert_instruction(std::string&& verbal_transition_alert_instruction);
  bool HasVerbalTransitionAlertInstruction() const;

  const std::string& verbal_pre_transition_instruction() const;
  void set_verbal_pre_transition_instruction(const std::string& verbal_pre_transition_instruction);
  void set_verbal_pre_transition_instruction(std::string&& verbal_pre_transition_instruction);
  bool HasVerbalPreTransitionInstruction() const;

  const std::string& verbal_post_transition_instruction() const;
  void set_verbal_post_transition_instruction(const std::string& verbal_post_transition_instruction);
  void set_verbal_post_transition_instruction(std::string&& verbal_post_transition_instruction);
  bool HasVerbalPostTransitionInstruction() const;

  bool tee() const;
  void set_tee(bool tee);

  TrailType trail_type() const;
  void set_trail_type(const TrailType trail);
  bool is_walkway() const;
  bool unnamed_walkway() const;
  bool is_cycleway() const;
  bool unnamed_cycleway() const;
  bool is_mountain_bike_trail() const;
  bool unnamed_mountain_bike_trail() const;
  bool pedestrian_crossing() const;
  void set_pedestrian_crossing(bool pedestrian_crossing);

  bool imminent_verbal_multi_cue() const;
  void set_imminent_verbal_multi_cue(bool imminent_verbal_multi_cue);
  bool distant_verbal_multi_cue() const;
  void set_distant_verbal_multi_cue(bool distant_verbal_multi_cue);
  bool HasVerbalMultiCue() const;

  bool to_stay_on() const;
  void set_to_stay_on(bool to_stay_on);

  RelativeDirection merge_to_relative_direction() const;
  void set_merge_to_relative_direction(RelativeDirection merge_to_relative_direction);

  bool drive_on_right() const;
  void set_drive_on_right(bool drive_on_right);

  bool has_time_restrictions() const;
  void set_has_time_restrictions(bool has_time_restrictions);

  bool has_right_traversable_outbound_intersecting_edge() const;
  void set_has_right_traversable_outbound_intersecting_edge(
      bool has_right_traversable_outbound_intersecting_edge);

  bool has_left_traversable_outbound_intersecting_edge() const;
  void set_has_left_traversable_outbound_intersecting_edge(
      bool has_left_traversable_outbound_intersecting_edge);

  bool include_verbal_pre_transition_length() const;
  void set_include_verbal_pre_transition_length(bool include_verbal_pre_transition_length);

  bool contains_obvious_maneuver() const;
  void set_contains_obvious_maneuver(bool contains_obvious_maneuver);

  uint32_t roundabout_exit_count() const;
  void set_roundabout_exit_count(uint32_t roundabout_exit_count);

  bool has_combined_enter_exit_roundabout() const;
  void set_has_combined_enter_exit_roundabout(bool has_combined_enter_exit_roundabout);

  float roundabout_length(const Options::Units& units = Options::kilometers) const;
  void set_roundabout_length(float roundabout_km_length); // Kilometers

  float roundabout_exit_length(const Options::Units& units = Options::kilometers) const;
  void set_roundabout_exit_length(float roundabout_exit_km_length); // Kilometers

  const StreetNames& roundabout_exit_street_names() const;
  void set_roundabout_exit_street_names(const std::vector<std::pair<std::string, bool>>& names);
  void set_roundabout_exit_street_names(std::unique_ptr<StreetNames>&& roundabout_exit_street_names);
  bool HasRoundaboutExitStreetNames() const;
  void ClearRoundaboutExitStreetNames();

  const StreetNames& roundabout_exit_begin_street_names() const;
  void set_roundabout_exit_begin_street_names(const std::vector<std::pair<std::string, bool>>& names);
  void set_roundabout_exit_begin_street_names(
      std::unique_ptr<StreetNames>&& roundabout_exit_begin_street_names);
  bool HasRoundaboutExitBeginStreetNames() const;
  void ClearRoundaboutExitBeginStreetNames();

  const Signs& roundabout_exit_signs() const;
  Signs* mutable_roundabout_exit_signs();

  uint32_t roundabout_exit_begin_heading() const;
  void set_roundabout_exit_begin_heading(uint32_t beginHeading);

  uint32_t roundabout_exit_turn_degree() const;
  void set_roundabout_exit_turn_degree(uint32_t turnDegree);

  uint32_t roundabout_exit_shape_index() const;
  void set_roundabout_exit_shape_index(uint32_t shapeIndex);

  bool has_collapsed_small_end_ramp_fork() const;
  void set_has_collapsed_small_end_ramp_fork(bool has_collapsed_small_end_ramp_fork);

  bool has_collapsed_merge_maneuver() const;
  void set_has_collapsed_merge_maneuver(bool has_collapsed_merge_maneuver);

  TravelMode travel_mode() const;
  void set_travel_mode(TravelMode travel_mode);

  bool rail() const;
  void set_rail(bool rail);

  bool bus() const;
  void set_bus(bool bus);

  VehicleType vehicle_type() const;
  void set_vehicle_type(VehicleType vehicle_type);

  PedestrianType pedestrian_type() const;
  void set_pedestrian_type(PedestrianType pedestrian_type);

  BicycleType bicycle_type() const;
  void set_bicycle_type(BicycleType bicycle_type);

  TransitType transit_type() const;
  void set_transit_type(TransitType transit_type);

  bool transit_connection() const;
  void set_transit_connection(bool transit_connection);

  const TransitEgressInfo& transit_connection_egress_info() const;
  void set_transit_connection_egress_info(const TransitEgressInfo& transit_connection_egress_info);

  const TransitStationInfo& transit_connection_station_info() const;
  void set_transit_connection_station_info(const TransitStationInfo& transit_connection_station_info);

  const TransitPlatformInfo& transit_connection_platform_info() const;
  void
  set_transit_connection_platform_info(const TransitPlatformInfo& transit_connection_platform_info);

  bool IsTransit() const;

  const TransitRouteInfo& transit_info() const;
  TransitRouteInfo* mutable_transit_info();

  std::string GetTransitArrivalTime() const;

  std::string GetTransitDepartureTime() const;

  const std::list<TransitPlatformInfo>& GetTransitStops() const;

  size_t GetTransitStopCount() const;

  void InsertTransitStop(const TransitPlatformInfo& transit_stop);

  const std::string& depart_instruction() const;
  void set_depart_instruction(const std::string& depart_instruction);
  void set_depart_instruction(std::string&& depart_instruction);

  const std::string& verbal_depart_instruction() const;
  void set_verbal_depart_instruction(const std::string& verbal_depart_instruction);
  void set_verbal_depart_instruction(std::string&& verbal_depart_instruction);

  const std::string& arrive_instruction() const;
  void set_arrive_instruction(const std::string& arrive_instruction);
  void set_arrive_instruction(std::string&& arrive_instruction);

  const std::string& verbal_arrive_instruction() const;
  void set_verbal_arrive_instruction(const std::string& verbal_arrive_instruction);
  void set_verbal_arrive_instruction(std::string&& verbal_arrive_instruction);

  const VerbalTextFormatter* verbal_formatter() const;
  void set_verbal_formatter(std::unique_ptr<VerbalTextFormatter>&& verbal_formatter);

  const std::vector<DirectionsLeg_GuidanceView>& guidance_views() const;
  std::vector<DirectionsLeg_GuidanceView>* mutable_guidance_views();

  DirectionsLeg_Maneuver_BssManeuverType bss_maneuver_type() const;
  void set_bss_maneuver_type(DirectionsLeg_Maneuver_BssManeuverType);

  bool has_long_street_name() const;
  void set_long_street_name(bool has_long_street_name);

  const BikeShareStationInfo& bss_info() const;
  void set_bss_info(const BikeShareStationInfo& bss_info);

  bool elevator() const;
  void set_elevator(bool elevator);

  bool indoor_steps() const;
  void set_indoor_steps(bool indoor_steps);

  bool escalator() const;
  void set_escalator(bool escalator);

  bool building_enter() const;
  void set_building_enter(bool building_enter);

  bool building_exit() const;
  void set_building_exit(bool building_exit);

  std::string end_level_ref() const;
  void set_end_level_ref(const std::string& end_level_ref);

  const std::vector<RouteLandmark>& landmarks() const;
  void set_landmarks(const std::vector<RouteLandmark>& landmarks);

#ifdef LOGGING_LEVEL_TRACE
  std::string ToString() const;

  std::string ToParameterString() const;
#endif

protected:
  DirectionsLeg_Maneuver_Type type_;
  TripLeg_Node_Type node_type_;
  bool has_node_type_;
  bool traffic_signal_;
  bool is_steps_;
  bool is_bridge_;
  bool is_tunnel_;
  std::unique_ptr<StreetNames> street_names_;
  std::unique_ptr<StreetNames> begin_street_names_;
  std::unique_ptr<StreetNames> cross_street_names_;
  std::string instruction_;
  float length_;      // Kilometers
  double time_;       // Seconds
  double basic_time_; // len/speed on each edge with no stop impact in seconds
  uint32_t turn_degree_;
  RelativeDirection begin_relative_direction_;
  DirectionsLeg_Maneuver_CardinalDirection begin_cardinal_direction_;
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
  bool fork_;
  bool begin_intersecting_edge_name_consistency_;
  bool intersecting_forward_edge_;
  std::string verbal_succinct_transition_instruction_;
  std::string verbal_transition_alert_instruction_;
  std::string verbal_pre_transition_instruction_;
  std::string verbal_post_transition_instruction_;
  bool tee_;
  TrailType trail_type_;
  bool imminent_verbal_multi_cue_;
  bool distant_verbal_multi_cue_;
  bool to_stay_on_;
  bool pedestrian_crossing_;
  RelativeDirection merge_to_relative_direction_;
  bool drive_on_right_; // Defaults to true
  bool has_time_restrictions_;
  bool has_right_traversable_outbound_intersecting_edge_;
  bool has_left_traversable_outbound_intersecting_edge_;
  bool include_verbal_pre_transition_length_;
  bool contains_obvious_maneuver_;

  uint32_t roundabout_exit_count_;
  bool has_combined_enter_exit_roundabout_;
  float roundabout_length_;      // Kilometers
  float roundabout_exit_length_; // Kilometers
  std::unique_ptr<StreetNames> roundabout_exit_street_names_;
  std::unique_ptr<StreetNames> roundabout_exit_begin_street_names_;
  Signs roundabout_exit_signs_;
  uint32_t roundabout_exit_begin_heading_;
  uint32_t roundabout_exit_turn_degree_;
  uint32_t roundabout_exit_shape_index_;

  bool has_collapsed_small_end_ramp_fork_;
  bool has_collapsed_merge_maneuver_;
  bool has_long_street_name_;

  // Bss support
  BikeShareStationInfo bss_info_;

  // Indoor elements
  bool elevator_;
  bool indoor_steps_;
  bool escalator_;
  bool building_enter_;
  bool building_exit_;
  std::string end_level_ref_;

  // Landmarks correlated to the maneuver
  std::vector<RouteLandmark> landmarks_;

  ////////////////////////////////////////////////////////////////////////////
  // Transit support

  // Transit connection flag and the associated stop
  bool transit_connection_;

  TransitEgressInfo transit_connection_egress_info_;
  TransitStationInfo transit_connection_station_info_;
  TransitPlatformInfo transit_connection_platform_info_;

  // The transit route info including list of stops
  TransitRouteInfo transit_info_;

  std::string depart_instruction_;
  std::string verbal_depart_instruction_;
  std::string arrive_instruction_;
  std::string verbal_arrive_instruction_;
  ////////////////////////////////////////////////////////////////////////////

  // Travel mode
  TravelMode travel_mode_;
  bool rail_;
  bool bus_;

  // Travel types
  VehicleType vehicle_type_;
  PedestrianType pedestrian_type_;
  BicycleType bicycle_type_;
  TransitType transit_type_;

  DirectionsLeg_Maneuver_BssManeuverType bss_maneuver_type_;

  std::unique_ptr<VerbalTextFormatter> verbal_formatter_;

  std::vector<DirectionsLeg_GuidanceView> guidance_views_;
};

} // namespace odin
} // namespace valhalla

#endif // VALHALLA_ODIN_MANEUVER_H_
