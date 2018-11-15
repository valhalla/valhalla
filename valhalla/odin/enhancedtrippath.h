#ifndef VALHALLA_ODIN_ENHANCEDTRIPPATH_H_
#define VALHALLA_ODIN_ENHANCEDTRIPPATH_H_

#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <valhalla/proto/directions_options.pb.h>
#include <valhalla/proto/trippath.pb.h>

namespace valhalla {
namespace odin {

class EnhancedTripPath;
class EnhancedTripPath_Edge;
class EnhancedTripPath_Node;
class EnhancedTripPath_Admin;

class EnhancedTripPath : public TripPath {
public:
  EnhancedTripPath() = delete;

  EnhancedTripPath_Node* GetEnhancedNode(const int node_index);

  EnhancedTripPath_Edge* GetPrevEdge(const int node_index, int delta = 1);

  EnhancedTripPath_Edge* GetCurrEdge(const int node_index);

  EnhancedTripPath_Edge* GetNextEdge(const int node_index, int delta = 1);

  bool IsValidNodeIndex(int node_index) const;

  bool IsFirstNodeIndex(int node_index) const;

  bool IsLastNodeIndex(int node_index) const;

  int GetLastNodeIndex() const;

  EnhancedTripPath_Admin* GetAdmin(size_t index);

  std::string GetCountryCode(int node_index);

  std::string GetStateCode(int node_index);

  const ::valhalla::odin::Location& GetOrigin() const;

  const ::valhalla::odin::Location& GetDestination() const;

  float GetLength(const DirectionsOptions::Units& units);
};

class EnhancedTripPath_Edge : public TripPath_Edge {
public:
  EnhancedTripPath_Edge() = delete;

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

  float GetLength(const DirectionsOptions::Units& units);

  std::string ToString() const;

#ifdef LOGGING_LEVEL_TRACE
  std::string ToParameterString() const;
#endif

protected:
#ifdef LOGGING_LEVEL_TRACE
  std::string StreetNamesToString(
      const ::google::protobuf::RepeatedPtrField<::valhalla::odin::StreetName>& street_names) const;

  std::string StreetNamesToParameterString(
      const ::google::protobuf::RepeatedPtrField<::valhalla::odin::StreetName>& street_names) const;

  std::string SignElementsToString(
      const ::google::protobuf::RepeatedPtrField<::valhalla::odin::TripPath_SignElement>&
          sign_elements) const;

  std::string SignElementsToParameterString(
      const ::google::protobuf::RepeatedPtrField<::valhalla::odin::TripPath_SignElement>&
          sign_elements) const;
#endif
};

class EnhancedTripPath_IntersectingEdge : public TripPath_IntersectingEdge {
public:
  EnhancedTripPath_IntersectingEdge() = delete;

  bool IsTraversable(const TripPath_TravelMode travel_mode) const;

  bool IsTraversableOutbound(const TripPath_TravelMode travel_mode) const;

  std::string ToString() const;
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

class EnhancedTripPath_Node : public TripPath_Node {
public:
  EnhancedTripPath_Node() = delete;

  bool HasIntersectingEdges() const;

  bool HasIntersectingEdgeNameConsistency() const;

  EnhancedTripPath_IntersectingEdge* GetIntersectingEdge(size_t index);

  void CalculateRightLeftIntersectingEdgeCounts(uint32_t from_heading,
                                                const TripPath_TravelMode travel_mode,
                                                IntersectingEdgeCounts& xedge_counts);

  bool HasFowardIntersectingEdge(uint32_t from_heading);

  bool HasForwardTraversableIntersectingEdge(uint32_t from_heading,
                                             const TripPath_TravelMode travel_mode);

  bool HasTraversableOutboundIntersectingEdge(const TripPath_TravelMode travel_mode);

  uint32_t GetStraightestIntersectingEdgeTurnDegree(uint32_t from_heading);

  uint32_t GetStraightestTraversableIntersectingEdgeTurnDegree(uint32_t from_heading,
                                                               const TripPath_TravelMode travel_mode);

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
};

class EnhancedTripPath_Admin : public TripPath_Admin {
public:
  EnhancedTripPath_Admin() = delete;

  std::string ToString() const;
};

const std::unordered_map<uint8_t, std::string> TripPath_TravelMode_Strings{
    {static_cast<uint8_t>(TripPath_TravelMode_kDrive), "drive"},
    {static_cast<uint8_t>(TripPath_TravelMode_kPedestrian), "pedestrian"},
    {static_cast<uint8_t>(TripPath_TravelMode_kBicycle), "bicycle"},
    {static_cast<uint8_t>(TripPath_TravelMode_kTransit), "transit"},
};
inline std::string to_string(TripPath_TravelMode travel_mode) {
  auto i = TripPath_TravelMode_Strings.find(static_cast<uint8_t>(travel_mode));
  if (i == TripPath_TravelMode_Strings.cend()) {
    return "null";
  }
  return i->second;
}

const std::unordered_map<uint8_t, std::string> TripPath_VehicleType_Strings{
    {static_cast<uint8_t>(TripPath_VehicleType_kCar), "car"},
    {static_cast<uint8_t>(TripPath_VehicleType_kMotorcycle), "motorcycle"},
    {static_cast<uint8_t>(TripPath_VehicleType_kAutoBus), "bus"},
    {static_cast<uint8_t>(TripPath_VehicleType_kTractorTrailer), "tractor_trailer"},
};
inline std::string to_string(TripPath_VehicleType vehicle_type) {
  auto i = TripPath_VehicleType_Strings.find(static_cast<uint8_t>(vehicle_type));
  if (i == TripPath_VehicleType_Strings.cend()) {
    return "null";
  }
  return i->second;
}

const std::unordered_map<uint8_t, std::string> TripPath_PedestrianType_Strings{
    {static_cast<uint8_t>(TripPath_PedestrianType_kFoot), "foot"},
    {static_cast<uint8_t>(TripPath_PedestrianType_kWheelchair), "wheelchair"},
    {static_cast<uint8_t>(TripPath_PedestrianType_kSegway), "segway"},
};
inline std::string to_string(TripPath_PedestrianType pedestrian_type) {
  auto i = TripPath_PedestrianType_Strings.find(static_cast<uint8_t>(pedestrian_type));
  if (i == TripPath_PedestrianType_Strings.cend()) {
    return "null";
  }
  return i->second;
}

const std::unordered_map<uint8_t, std::string> TripPath_BicycleType_Strings{
    {static_cast<uint8_t>(TripPath_BicycleType_kRoad), "road"},
    {static_cast<uint8_t>(TripPath_BicycleType_kCross), "cross"},
    {static_cast<uint8_t>(TripPath_BicycleType_kHybrid), "hybrid"},
    {static_cast<uint8_t>(TripPath_BicycleType_kMountain), "mountain"},
};
inline std::string to_string(TripPath_BicycleType bicycle_type) {
  auto i = TripPath_BicycleType_Strings.find(static_cast<uint8_t>(bicycle_type));
  if (i == TripPath_BicycleType_Strings.cend()) {
    return "null";
  }
  return i->second;
}

const std::unordered_map<uint8_t, std::string> TripPath_Sidewalk_Strings = {
    {static_cast<uint8_t>(TripPath_Sidewalk_kNoSidewalk), "none"},
    {static_cast<uint8_t>(TripPath_Sidewalk_kLeft), "left"},
    {static_cast<uint8_t>(TripPath_Sidewalk_kRight), "right"},
    {static_cast<uint8_t>(TripPath_Sidewalk_kBothSides), "both"},
};
inline std::string to_string(TripPath_Sidewalk s) {
  auto i = TripPath_Sidewalk_Strings.find(static_cast<uint8_t>(s));
  if (i == TripPath_Sidewalk_Strings.cend()) {
    return "null";
  }
  return i->second;
}

const std::unordered_map<uint8_t, std::string> TripPath_Traversability_Strings = {
    {static_cast<uint8_t>(TripPath_Traversability_kNone), "none"},
    {static_cast<uint8_t>(TripPath_Traversability_kForward), "forward"},
    {static_cast<uint8_t>(TripPath_Traversability_kBackward), "backward"},
    {static_cast<uint8_t>(TripPath_Traversability_kBoth), "both"},
};
inline std::string to_string(TripPath_Traversability t) {
  auto i = TripPath_Traversability_Strings.find(static_cast<uint8_t>(t));
  if (i == TripPath_Traversability_Strings.cend()) {
    return "null";
  }
  return i->second;
}

} // namespace odin
} // namespace valhalla

#endif // VALHALLA_ODIN_ENHANCEDTRIPPATH_H_
