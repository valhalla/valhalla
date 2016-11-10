#ifndef VALHALLA_ODIN_ENHANCEDTRIPPATH_H_
#define VALHALLA_ODIN_ENHANCEDTRIPPATH_H_

#include <valhalla/proto/trippath.pb.h>
#include <valhalla/proto/directions_options.pb.h>

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

  const ::valhalla::odin::TripPath_Location& GetOrigin() const;

  const ::valhalla::odin::TripPath_Location& GetDestination() const;

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
  bool IsRailConnectionUse() const;
  bool IsBusConnectionUse() const;
  bool IsTransitConnectionUse() const;

  bool IsUnnamedWalkway() const;

  bool IsUnnamedCycleway() const;

  bool IsUnnamedMountainBikeTrail() const;

  bool IsHighway() const;

  bool IsOneway() const;

  bool IsForward(uint32_t prev2curr_turn_degree) const;

  bool IsWiderForward(uint32_t prev2curr_turn_degree) const;

  bool IsStraightest(uint32_t prev2curr_turn_degree,
                     uint32_t straightest_xedge_turn_degree) const;

  std::vector<std::string> GetNameList() const;

  float GetLength(const DirectionsOptions::Units& units);

  std::string ToString() const;

  std::string ToParameterString() const;

 protected:

  std::string ListToString(
      const ::google::protobuf::RepeatedPtrField<::std::string>& string_list) const;

  std::string ListToParameterString(
      const ::google::protobuf::RepeatedPtrField<::std::string>& string_list) const;

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

  IntersectingEdgeCounts(uint32_t r, uint32_t rs, uint32_t rdo, uint32_t rsdo,
                         uint32_t l, uint32_t ls, uint32_t ldo, uint32_t lsdo)
      : right(r),
        right_similar(rs),
        right_traversable_outbound(rdo),
        right_similar_traversable_outbound(rsdo),
        left(l),
        left_similar(ls),
        left_traversable_outbound(ldo),
        left_similar_traversable_outbound(lsdo) {
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

  void CalculateRightLeftIntersectingEdgeCounts(
      uint32_t from_heading, const TripPath_TravelMode travel_mode,
      IntersectingEdgeCounts& xedge_counts);

  bool HasFowardIntersectingEdge(uint32_t from_heading);

  bool HasForwardTraversableIntersectingEdge(
      uint32_t from_heading, const TripPath_TravelMode travel_mode);

  bool HasTraversableOutboundIntersectingEdge(
      const TripPath_TravelMode travel_mode);

  uint32_t GetStraightestIntersectingEdgeTurnDegree(uint32_t from_heading);

  uint32_t GetStraightestTraversableIntersectingEdgeTurnDegree(
      uint32_t from_heading, const TripPath_TravelMode travel_mode);

  // Type
  bool IsStreetIntersection() const;
  bool IsGate() const;
  bool IsBollard() const;
  bool IsTollBooth() const;
  bool IsMultiUseTransitStop() const;
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

}
}

#endif  // VALHALLA_ODIN_ENHANCEDTRIPPATH_H_

