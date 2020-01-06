#ifndef VALHALLA_ODIN_ENHANCEDTRIPPATH_H_
#define VALHALLA_ODIN_ENHANCEDTRIPPATH_H_

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <valhalla/baldr/turn.h>
#include <valhalla/proto/directions.pb.h>
#include <valhalla/proto/options.pb.h>
#include <valhalla/proto/trip.pb.h>

namespace valhalla {
namespace odin {

class EnhancedTripLeg;
class EnhancedTripLeg_Edge;
class EnhancedTripLeg_Node;
class EnhancedTripLeg_Admin;

class EnhancedTripLeg {
public:
  EnhancedTripLeg(TripLeg& trip_path);

  const std::string& shape() const {
    return trip_path_.shape();
  }

  int node_size() const {
    return trip_path_.node_size();
  }

  const ::valhalla::TripLeg_Node& node(int index) const {
    return trip_path_.node(index);
  }

  ::valhalla::TripLeg_Node* mutable_node(int index) const {
    return trip_path_.mutable_node(index);
  }

  const ::google::protobuf::RepeatedPtrField<::valhalla::TripLeg_Node>& node() const {
    return trip_path_.node();
  }

  int location_size() const {
    return trip_path_.location_size();
  }

  const ::valhalla::Location& location(int index) const {
    return trip_path_.location(index);
  }

  int admin_size() const {
    return trip_path_.admin_size();
  }

  ::valhalla::TripLeg_Admin* mutable_admin(int index) {
    return trip_path_.mutable_admin(index);
  }

  uint64_t osm_changeset() const {
    return trip_path_.osm_changeset();
  }

  uint64_t trip_id() const {
    return trip_path_.trip_id();
  }

  uint32_t leg_id() const {
    return trip_path_.leg_id();
  }

  uint32_t leg_count() const {
    return trip_path_.leg_count();
  }

  const ::google::protobuf::RepeatedPtrField<::valhalla::Location>& location() const {
    return trip_path_.location();
  }

  const ::valhalla::BoundingBox& bbox() const {
    return trip_path_.bbox();
  }

  std::unique_ptr<EnhancedTripLeg_Node> GetEnhancedNode(const int node_index);

  std::unique_ptr<EnhancedTripLeg_Edge> GetPrevEdge(const int node_index, int delta = 1);

  std::unique_ptr<EnhancedTripLeg_Edge> GetCurrEdge(const int node_index) const;

  std::unique_ptr<EnhancedTripLeg_Edge> GetNextEdge(const int node_index, int delta = 1) const;

  bool IsValidNodeIndex(int node_index) const;

  bool IsFirstNodeIndex(int node_index) const;

  bool IsLastNodeIndex(int node_index) const;

  int GetLastNodeIndex() const;

  std::unique_ptr<EnhancedTripLeg_Admin> GetAdmin(size_t index);

  std::string GetCountryCode(int node_index);

  std::string GetStateCode(int node_index);

  const ::valhalla::Location& GetOrigin() const;

  const ::valhalla::Location& GetDestination() const;

  float GetLength(const Options::Units& units);

protected:
  TripLeg& trip_path_;
};

class EnhancedTripLeg_Edge {
public:
  EnhancedTripLeg_Edge(TripLeg_Edge* mutable_edge);

  int name_size() const {
    return mutable_edge_->name_size();
  }

  const ::valhalla::StreetName& name(int index) const {
    return mutable_edge_->name(index);
  }

  const ::google::protobuf::RepeatedPtrField<::valhalla::StreetName>& name() const {
    return mutable_edge_->name();
  }

  float length() const {
    return mutable_edge_->length();
  }

  float speed() const {
    return mutable_edge_->speed();
  }

  ::valhalla::TripLeg_RoadClass road_class() const {
    return mutable_edge_->road_class();
  }

  uint32_t begin_heading() const {
    return mutable_edge_->begin_heading();
  }

  void set_begin_heading(uint32_t value) {
    return mutable_edge_->set_begin_heading(value);
  }

  uint32_t end_heading() const {
    return mutable_edge_->end_heading();
  }

  void set_end_heading(uint32_t value) {
    return mutable_edge_->set_end_heading(value);
  }

  uint32_t begin_shape_index() const {
    return mutable_edge_->begin_shape_index();
  }

  uint32_t end_shape_index() const {
    return mutable_edge_->end_shape_index();
  }

  ::valhalla::TripLeg_Traversability traversability() const {
    return mutable_edge_->traversability();
  }

  ::valhalla::TripLeg_Use use() const {
    return mutable_edge_->use();
  }

  bool has_vehicle_type() const {
    return mutable_edge_->has_vehicle_type();
  }

  ::valhalla::TripLeg_VehicleType vehicle_type() const {
    return mutable_edge_->vehicle_type();
  }

  bool has_pedestrian_type() const {
    return mutable_edge_->has_pedestrian_type();
  }

  ::valhalla::TripLeg_PedestrianType pedestrian_type() const {
    return mutable_edge_->pedestrian_type();
  }

  bool has_bicycle_type() const {
    return mutable_edge_->has_bicycle_type();
  }

  ::valhalla::TripLeg_BicycleType bicycle_type() const {
    return mutable_edge_->bicycle_type();
  }

  bool has_transit_type() const {
    return mutable_edge_->has_transit_type();
  }

  ::valhalla::TripLeg_TransitType transit_type() const {
    return mutable_edge_->transit_type();
    return mutable_edge_->transit_type();
  }

  bool toll() const {
    return mutable_edge_->toll();
  }

  bool has_time_restrictions() const {
    return mutable_edge_->has_time_restrictions();
  }

  bool unpaved() const {
    return mutable_edge_->unpaved();
  }

  bool tunnel() const {
    return mutable_edge_->tunnel();
  }

  bool bridge() const {
    return mutable_edge_->bridge();
  }

  bool roundabout() const {
    return mutable_edge_->roundabout();
  }

  bool internal_intersection() const {
    return mutable_edge_->internal_intersection();
  }

  bool drive_on_right() const {
    return mutable_edge_->drive_on_right();
  }

  ::valhalla::TripLeg_Surface surface() const {
    return mutable_edge_->surface();
  }

  bool has_sign() const {
    return mutable_edge_->has_sign();
  }

  const ::valhalla::TripLeg_Sign& sign() const {
    return mutable_edge_->sign();
  }

  bool has_travel_mode() const {
    return mutable_edge_->has_travel_mode();
  }

  ::valhalla::TripLeg_TravelMode travel_mode() const {
    return mutable_edge_->travel_mode();
  }

  bool has_transit_route_info() const {
    return mutable_edge_->has_transit_route_info();
  }

  const ::valhalla::TripLeg_TransitRouteInfo& transit_route_info() const {
    return mutable_edge_->transit_route_info();
  }

  uint64_t id() const {
    return mutable_edge_->id();
  }

  uint64_t way_id() const {
    return mutable_edge_->way_id();
  }

  float weighted_grade() const {
    return mutable_edge_->weighted_grade();
  }

  int32_t max_upward_grade() const {
    return mutable_edge_->max_upward_grade();
  }

  int32_t max_downward_grade() const {
    return mutable_edge_->max_downward_grade();
  }

  int32_t lane_count() const {
    return mutable_edge_->lane_count();
  }

  ::valhalla::TripLeg_CycleLane cycle_lane() const {
    return mutable_edge_->cycle_lane();
  }

  int32_t bicycle_network() const {
    return mutable_edge_->bicycle_network();
  }

  ::valhalla::TripLeg_Sidewalk sidewalk() const {
    return mutable_edge_->sidewalk();
  }

  int32_t density() const {
    return mutable_edge_->density();
  }

  int32_t speed_limit() const {
    return mutable_edge_->speed_limit();
  }

  float truck_speed() const {
    return mutable_edge_->truck_speed();
  }

  bool truck_route() const {
    return mutable_edge_->truck_route();
  }

  int turn_lanes_size() const {
    return mutable_edge_->turn_lanes_size();
  }

  const ::google::protobuf::RepeatedPtrField<::valhalla::TurnLane>& turn_lanes() const {
    return mutable_edge_->turn_lanes();
  }

  ::google::protobuf::RepeatedPtrField<::valhalla::TurnLane>* mutable_turn_lanes() {
    return mutable_edge_->mutable_turn_lanes();
  }

  bool IsUnnamed() const;

  // Use
  bool IsRoadUse() const;
  bool IsRampUse() const;
  bool IsTurnChannelUse() const;
  bool IsTrackUse() const;
  bool IsDrivewayUse() const;
  bool IsAlleyUse() const;
  bool IsParkingAisleUse() const;
  bool IsEmergencyAccessUse() const;
  bool IsDriveThruUse() const;
  bool IsCuldesacUse() const;
  bool IsCyclewayUse() const;
  bool IsMountainBikeUse() const;
  bool IsSidewalkUse() const;
  bool IsFootwayUse() const;
  bool IsStepsUse() const;
  bool IsPathUse() const;
  bool IsPedestrianUse() const;
  bool IsBridlewayUse() const;
  bool IsOtherUse() const;
  bool IsFerryUse() const;
  bool IsRailFerryUse() const;
  bool IsRailUse() const;
  bool IsBusUse() const;
  bool IsEgressConnectionUse() const;
  bool IsPlatformConnectionUse() const;
  bool IsTransitConnectionUse() const;

  bool IsTransitConnection() const;

  bool IsUnnamedWalkway() const;

  bool IsUnnamedCycleway() const;

  bool IsUnnamedMountainBikeTrail() const;

  bool IsHighway() const;

  bool IsOneway() const;

  bool IsForward(uint32_t prev2curr_turn_degree) const;

  bool IsWiderForward(uint32_t prev2curr_turn_degree) const;

  bool IsStraightest(uint32_t prev2curr_turn_degree, uint32_t straightest_xedge_turn_degree) const;

  std::vector<std::pair<std::string, bool>> GetNameList() const;

  float GetLength(const Options::Units& units);

  // Turn Lanes
  bool HasActiveTurnLane() const;
  bool HasNonDirectionalTurnLane() const;
  bool HasTurnLane(uint16_t turn_lane_direction) const;
  uint16_t ActivateTurnLanes(uint16_t turn_lane_direction,
                             float remaining_step_distance,
                             const DirectionsLeg_Maneuver_Type& curr_maneuver_type,
                             const DirectionsLeg_Maneuver_Type& next_maneuver_type);
  uint16_t ActivateTurnLanesFromLeft(uint16_t turn_lane_direction,
                                     uint16_t activated_max = std::numeric_limits<uint16_t>::max());
  uint16_t ActivateTurnLanesFromRight(uint16_t turn_lane_direction,
                                      uint16_t activated_max = std::numeric_limits<uint16_t>::max());

  std::string ToString() const;

  std::string TurnLanesToString() const;

#ifdef LOGGING_LEVEL_TRACE
  std::string ToParameterString() const;
#endif

protected:
  TripLeg_Edge* mutable_edge_;

  std::string StreetNamesToString(
      const ::google::protobuf::RepeatedPtrField<::valhalla::StreetName>& street_names) const;

  std::string SignElementsToString(
      const ::google::protobuf::RepeatedPtrField<::valhalla::TripLeg_SignElement>& sign_elements)
      const;

#ifdef LOGGING_LEVEL_TRACE
  std::string StreetNamesToParameterString(
      const ::google::protobuf::RepeatedPtrField<::valhalla::StreetName>& street_names) const;

  std::string SignElementsToParameterString(
      const ::google::protobuf::RepeatedPtrField<::valhalla::TripLeg_SignElement>& sign_elements)
      const;
#endif
};

class EnhancedTripLeg_IntersectingEdge {
public:
  EnhancedTripLeg_IntersectingEdge(TripLeg_IntersectingEdge* mutable_intersecting_edge);

  uint32_t begin_heading() const {
    return mutable_intersecting_edge_->begin_heading();
  }

  bool prev_name_consistency() const {
    return mutable_intersecting_edge_->prev_name_consistency();
  }

  bool curr_name_consistency() const {
    return mutable_intersecting_edge_->curr_name_consistency();
  }

  ::valhalla::TripLeg_Traversability driveability() const {
    return mutable_intersecting_edge_->driveability();
  }

  ::valhalla::TripLeg_Traversability cyclability() const {
    return mutable_intersecting_edge_->cyclability();
  }

  ::valhalla::TripLeg_Traversability walkability() const {
    return mutable_intersecting_edge_->walkability();
  }

  bool has_use() const {
    return mutable_intersecting_edge_->has_use();
  }

  ::valhalla::TripLeg_Use use() const {
    return mutable_intersecting_edge_->use();
  }

  ::valhalla::TripLeg_RoadClass road_class() const {
    return mutable_intersecting_edge_->road_class();
  }

  bool IsTraversable(const TripLeg_TravelMode travel_mode) const;

  bool IsTraversableOutbound(const TripLeg_TravelMode travel_mode) const;

  bool IsHighway() const;

  std::string ToString() const;

protected:
  TripLeg_IntersectingEdge* mutable_intersecting_edge_;
};

struct IntersectingEdgeCounts {

  IntersectingEdgeCounts() {
    clear();
  }

  IntersectingEdgeCounts(uint32_t r,
                         uint32_t rs,
                         uint32_t rdo,
                         uint32_t rsdo,
                         uint32_t l,
                         uint32_t ls,
                         uint32_t ldo,
                         uint32_t lsdo)
      : right(r), right_similar(rs), right_traversable_outbound(rdo),
        right_similar_traversable_outbound(rsdo), left(l), left_similar(ls),
        left_traversable_outbound(ldo), left_similar_traversable_outbound(lsdo) {
  }

  void clear() {
    right = 0;
    right_similar = 0;
    right_traversable_outbound = 0;
    right_similar_traversable_outbound = 0;
    left = 0;
    left_similar = 0;
    left_traversable_outbound = 0;
    left_similar_traversable_outbound = 0;
  }

  uint32_t right;
  uint32_t right_similar;
  uint32_t right_traversable_outbound;
  uint32_t right_similar_traversable_outbound;
  uint32_t left;
  uint32_t left_similar;
  uint32_t left_traversable_outbound;
  uint32_t left_similar_traversable_outbound;
};

class EnhancedTripLeg_Node {
public:
  EnhancedTripLeg_Node(TripLeg_Node* mutable_node);

  int intersecting_edge_size() const {
    return mutable_node_->intersecting_edge_size();
  }

  const ::valhalla::TransitPlatformInfo& transit_platform_info() const {
    return mutable_node_->transit_platform_info();
  }

  bool fork() const {
    return mutable_node_->fork();
  }

  const ::valhalla::TripLeg_IntersectingEdge& intersecting_edge(int index) const {
    return mutable_node_->intersecting_edge(index);
  }

  ::valhalla::TripLeg_IntersectingEdge* mutable_intersecting_edge(int index) {
    return mutable_node_->mutable_intersecting_edge(index);
  }

  const ::google::protobuf::RepeatedPtrField<::valhalla::TripLeg_IntersectingEdge>&
  intersecting_edge() const {
    return mutable_node_->intersecting_edge();
  }

  const ::valhalla::TripLeg_Edge& edge() const {
    return mutable_node_->edge();
  }

  ::valhalla::TripLeg_Node_Type type() const {
    return mutable_node_->type();
  }

  uint32_t elapsed_time() const {
    return mutable_node_->elapsed_time();
  }

  uint32_t admin_index() const {
    return mutable_node_->admin_index();
  }

  bool has_transit_platform_info() const {
    return mutable_node_->has_transit_platform_info();
  }

  const std::string& time_zone() const {
    return mutable_node_->time_zone();
  }

  bool HasIntersectingEdges() const;

  bool HasIntersectingEdgeNameConsistency() const;

  bool HasIntersectingEdgeCurrNameConsistency() const;

  std::unique_ptr<EnhancedTripLeg_IntersectingEdge> GetIntersectingEdge(size_t index);

  void CalculateRightLeftIntersectingEdgeCounts(uint32_t from_heading,
                                                const TripLeg_TravelMode travel_mode,
                                                IntersectingEdgeCounts& xedge_counts);

  bool HasFowardIntersectingEdge(uint32_t from_heading);

  bool HasForwardTraversableIntersectingEdge(uint32_t from_heading,
                                             const TripLeg_TravelMode travel_mode);

  bool HasForwardTraversableSignificantRoadClassXEdge(uint32_t from_heading,
                                                      const TripLeg_TravelMode travel_mode,
                                                      TripLeg_RoadClass path_road_class);

  bool HasWiderForwardTraversableIntersectingEdge(uint32_t from_heading,
                                                  const TripLeg_TravelMode travel_mode);

  bool HasWiderForwardTraversableHighwayXEdge(uint32_t from_heading,
                                              const TripLeg_TravelMode travel_mode);

  bool HasTraversableOutboundIntersectingEdge(const TripLeg_TravelMode travel_mode);

  bool HasSpecifiedTurnXEdge(const baldr::Turn::Type turn_type,
                             uint32_t from_heading,
                             const TripLeg_TravelMode travel_mode);

  bool HasSpecifiedRoadClassXEdge(const TripLeg_RoadClass road_class);

  uint32_t GetStraightestIntersectingEdgeTurnDegree(uint32_t from_heading);

  uint32_t GetStraightestTraversableIntersectingEdgeTurnDegree(uint32_t from_heading,
                                                               const TripLeg_TravelMode travel_mode,
                                                               TripLeg_Use* use = nullptr);

  bool IsStraightestTraversableIntersectingEdgeReversed(uint32_t from_heading,
                                                        const TripLeg_TravelMode travel_mode);

  uint32_t GetRightMostTurnDegree(uint32_t turn_degree,
                                  uint32_t from_heading,
                                  const TripLeg_TravelMode travel_mode);

  uint32_t GetLeftMostTurnDegree(uint32_t turn_degree,
                                 uint32_t from_heading,
                                 const TripLeg_TravelMode travel_mode);

  // Type
  bool IsStreetIntersection() const;
  bool IsGate() const;
  bool IsBollard() const;
  bool IsTollBooth() const;
  bool IsTransitEgress() const;
  bool IsTransitStation() const;
  bool IsTransitPlatform() const;
  bool IsBikeShare() const;
  bool IsParking() const;
  bool IsMotorwayJunction() const;
  bool IsBorderControl() const;

  std::string ToString() const;

protected:
  TripLeg_Node* mutable_node_;
};

class EnhancedTripLeg_Admin {
public:
  EnhancedTripLeg_Admin(TripLeg_Admin* mutable_admin);

  const std::string& country_code() const {
    return mutable_admin_->country_code();
  }

  const std::string& country_text() const {
    return mutable_admin_->country_text();
  }

  const std::string& state_code() const {
    return mutable_admin_->state_code();
  }

  const std::string& state_text() const {
    return mutable_admin_->state_text();
  }

  std::string ToString() const;

protected:
  TripLeg_Admin* mutable_admin_;
};

const std::unordered_map<uint8_t, std::string> TripLeg_TravelMode_Strings{
    {static_cast<uint8_t>(TripLeg_TravelMode_kDrive), "drive"},
    {static_cast<uint8_t>(TripLeg_TravelMode_kPedestrian), "pedestrian"},
    {static_cast<uint8_t>(TripLeg_TravelMode_kBicycle), "bicycle"},
    {static_cast<uint8_t>(TripLeg_TravelMode_kTransit), "transit"},
};
inline std::string to_string(TripLeg_TravelMode travel_mode) {
  auto i = TripLeg_TravelMode_Strings.find(static_cast<uint8_t>(travel_mode));
  if (i == TripLeg_TravelMode_Strings.cend()) {
    return "null";
  }
  return i->second;
}

const std::unordered_map<uint8_t, std::string> TripLeg_VehicleType_Strings{
    {static_cast<uint8_t>(TripLeg_VehicleType_kCar), "car"},
    {static_cast<uint8_t>(TripLeg_VehicleType_kMotorcycle), "motorcycle"},
    {static_cast<uint8_t>(TripLeg_VehicleType_kAutoBus), "bus"},
    {static_cast<uint8_t>(TripLeg_VehicleType_kTractorTrailer), "tractor_trailer"},
};
inline std::string to_string(TripLeg_VehicleType vehicle_type) {
  auto i = TripLeg_VehicleType_Strings.find(static_cast<uint8_t>(vehicle_type));
  if (i == TripLeg_VehicleType_Strings.cend()) {
    return "null";
  }
  return i->second;
}

const std::unordered_map<uint8_t, std::string> TripLeg_PedestrianType_Strings{
    {static_cast<uint8_t>(TripLeg_PedestrianType_kFoot), "foot"},
    {static_cast<uint8_t>(TripLeg_PedestrianType_kWheelchair), "wheelchair"},
    {static_cast<uint8_t>(TripLeg_PedestrianType_kSegway), "segway"},
};
inline std::string to_string(TripLeg_PedestrianType pedestrian_type) {
  auto i = TripLeg_PedestrianType_Strings.find(static_cast<uint8_t>(pedestrian_type));
  if (i == TripLeg_PedestrianType_Strings.cend()) {
    return "null";
  }
  return i->second;
}

const std::unordered_map<uint8_t, std::string> TripLeg_BicycleType_Strings{
    {static_cast<uint8_t>(TripLeg_BicycleType_kRoad), "road"},
    {static_cast<uint8_t>(TripLeg_BicycleType_kCross), "cross"},
    {static_cast<uint8_t>(TripLeg_BicycleType_kHybrid), "hybrid"},
    {static_cast<uint8_t>(TripLeg_BicycleType_kMountain), "mountain"},
};
inline std::string to_string(TripLeg_BicycleType bicycle_type) {
  auto i = TripLeg_BicycleType_Strings.find(static_cast<uint8_t>(bicycle_type));
  if (i == TripLeg_BicycleType_Strings.cend()) {
    return "null";
  }
  return i->second;
}

const std::unordered_map<uint8_t, std::string> TripLeg_Sidewalk_Strings = {
    {static_cast<uint8_t>(TripLeg_Sidewalk_kNoSidewalk), "none"},
    {static_cast<uint8_t>(TripLeg_Sidewalk_kLeft), "left"},
    {static_cast<uint8_t>(TripLeg_Sidewalk_kRight), "right"},
    {static_cast<uint8_t>(TripLeg_Sidewalk_kBothSides), "both"},
};
inline std::string to_string(TripLeg_Sidewalk s) {
  auto i = TripLeg_Sidewalk_Strings.find(static_cast<uint8_t>(s));
  if (i == TripLeg_Sidewalk_Strings.cend()) {
    return "null";
  }
  return i->second;
}

const std::unordered_map<uint8_t, std::string> TripLeg_Traversability_Strings = {
    {static_cast<uint8_t>(TripLeg_Traversability_kNone), "none"},
    {static_cast<uint8_t>(TripLeg_Traversability_kForward), "forward"},
    {static_cast<uint8_t>(TripLeg_Traversability_kBackward), "backward"},
    {static_cast<uint8_t>(TripLeg_Traversability_kBoth), "both"},
};
inline std::string to_string(TripLeg_Traversability t) {
  auto i = TripLeg_Traversability_Strings.find(static_cast<uint8_t>(t));
  if (i == TripLeg_Traversability_Strings.cend()) {
    return "null";
  }
  return i->second;
}

} // namespace odin
} // namespace valhalla

#endif // VALHALLA_ODIN_ENHANCEDTRIPPATH_H_
