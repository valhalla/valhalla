#include "odin/enhancedtrippath.h"
#include "baldr/turnlanes.h"
#include "midgard/util.h"

#include <cstdint>

#include "test.h"

using namespace std;
using namespace valhalla;
using namespace valhalla::odin;
using namespace valhalla::baldr;

namespace {

void TryCalculateRightLeftIntersectingEdgeCounts(uint32_t from_heading,
                                                 std::unique_ptr<EnhancedTripLeg_Node> node,
                                                 const IntersectingEdgeCounts& expected_xedge_counts,
                                                 const TravelMode travel_mode = TravelMode::kDrive) {

  IntersectingEdgeCounts xedge_counts;
  xedge_counts.clear();

  node->CalculateRightLeftIntersectingEdgeCounts(from_heading, travel_mode, xedge_counts);
  EXPECT_EQ(xedge_counts.right, expected_xedge_counts.right);
  EXPECT_EQ(xedge_counts.right_similar, expected_xedge_counts.right_similar);
  EXPECT_EQ(xedge_counts.right_traversable_outbound,
            expected_xedge_counts.right_traversable_outbound);
  EXPECT_EQ(xedge_counts.right_similar_traversable_outbound,
            expected_xedge_counts.right_similar_traversable_outbound);
  EXPECT_EQ(xedge_counts.left, expected_xedge_counts.left);
  EXPECT_EQ(xedge_counts.left_similar, expected_xedge_counts.left_similar);
  EXPECT_EQ(xedge_counts.left_traversable_outbound, expected_xedge_counts.left_traversable_outbound);
  EXPECT_EQ(xedge_counts.left_similar_traversable_outbound,
            expected_xedge_counts.left_similar_traversable_outbound);
}

TEST(EnhancedTripPathCalculateRightLeftIntersectingEdgeCounts, StraightStraight) {
  // Path straight, intersecting straight
  TripLeg_Node node1;
  node1.mutable_edge()->set_begin_heading(5);
  TripLeg_IntersectingEdge* n1_ie1 = node1.add_intersecting_edge();
  n1_ie1->set_begin_heading(355);
  n1_ie1->set_driveability(TripLeg_Traversability_kBoth);
  TryCalculateRightLeftIntersectingEdgeCounts(0, std::make_unique<EnhancedTripLeg_Node>(&node1),
                                              IntersectingEdgeCounts(0, 0, 0, 0, 1, 1, 1, 1));

  // Path straight, intersecting straight
  TripLeg_Node node2;
  node2.mutable_edge()->set_begin_heading(355);
  TripLeg_IntersectingEdge* n2_ie1 = node2.add_intersecting_edge();
  n2_ie1->set_begin_heading(5);
  n2_ie1->set_driveability(TripLeg_Traversability_kForward);
  TryCalculateRightLeftIntersectingEdgeCounts(0, std::make_unique<EnhancedTripLeg_Node>(&node2),
                                              IntersectingEdgeCounts(1, 1, 1, 1, 0, 0, 0, 0));
}

TEST(EnhancedTripPathCalculateRightLeftIntersectingEdgeCounts, SlightRightStraight) {
  // Path slight right, intersecting straight
  TripLeg_Node node1;
  node1.mutable_edge()->set_begin_heading(11);
  TripLeg_IntersectingEdge* n1_ie1 = node1.add_intersecting_edge();
  n1_ie1->set_begin_heading(0);
  n1_ie1->set_driveability(TripLeg_Traversability_kBackward);
  TryCalculateRightLeftIntersectingEdgeCounts(0, std::make_unique<EnhancedTripLeg_Node>(&node1),
                                              IntersectingEdgeCounts(0, 0, 0, 0, 1, 1, 0, 0));

  // Path slight right, intersecting straight
  TripLeg_Node node2;
  node2.mutable_edge()->set_begin_heading(105);
  TripLeg_IntersectingEdge* n2_ie1 = node2.add_intersecting_edge();
  n2_ie1->set_begin_heading(85);
  n2_ie1->set_driveability(TripLeg_Traversability_kNone);
  TryCalculateRightLeftIntersectingEdgeCounts(90, std::make_unique<EnhancedTripLeg_Node>(&node2),
                                              IntersectingEdgeCounts(0, 0, 0, 0, 1, 1, 0, 0));
}

TEST(EnhancedTripPathCalculateRightLeftIntersectingEdgeCounts, SlightLeftStraight) {
  // Path slight left, intersecting straight
  TripLeg_Node node1;
  node1.mutable_edge()->set_begin_heading(345);
  node1.add_intersecting_edge()->set_begin_heading(355);
  TryCalculateRightLeftIntersectingEdgeCounts(0, std::make_unique<EnhancedTripLeg_Node>(&node1),
                                              IntersectingEdgeCounts(1, 1, 0, 0, 0, 0, 0, 0));

  // Path slight left, intersecting straight
  TripLeg_Node node2;
  node2.mutable_edge()->set_begin_heading(255);
  node2.add_intersecting_edge()->set_begin_heading(275);
  TryCalculateRightLeftIntersectingEdgeCounts(270, std::make_unique<EnhancedTripLeg_Node>(&node2),
                                              IntersectingEdgeCounts(1, 1, 0, 0, 0, 0, 0, 0));
}

TEST(EnhancedTripPathCalculateRightLeftIntersectingEdgeCounts, SlightLeftRightLeft) {
  // Path slight left, intersecting right and left
  TripLeg_Node node1;
  node1.mutable_edge()->set_begin_heading(340);
  node1.add_intersecting_edge()->set_begin_heading(45);
  node1.add_intersecting_edge()->set_begin_heading(90);
  node1.add_intersecting_edge()->set_begin_heading(135);
  node1.add_intersecting_edge()->set_begin_heading(315);
  node1.add_intersecting_edge()->set_begin_heading(270);
  node1.add_intersecting_edge()->set_begin_heading(225);
  TryCalculateRightLeftIntersectingEdgeCounts(0, std::make_unique<EnhancedTripLeg_Node>(&node1),
                                              IntersectingEdgeCounts(3, 0, 0, 0, 3, 1, 0, 0));

  // Path slight left, intersecting right and left
  TripLeg_Node node2;
  node2.mutable_edge()->set_begin_heading(60);
  TripLeg_IntersectingEdge* n2_ie1 = node2.add_intersecting_edge();
  n2_ie1->set_begin_heading(157);
  n2_ie1->set_driveability(TripLeg_Traversability_kBoth);
  TripLeg_IntersectingEdge* n2_ie2 = node2.add_intersecting_edge();
  n2_ie2->set_begin_heading(337);
  n2_ie2->set_driveability(TripLeg_Traversability_kForward);
  TryCalculateRightLeftIntersectingEdgeCounts(80, std::make_unique<EnhancedTripLeg_Node>(&node2),
                                              IntersectingEdgeCounts(1, 0, 1, 0, 1, 0, 1, 0));
}

TEST(EnhancedTripPathCalculateRightLeftIntersectingEdgeCounts, SharpRightRightLeft) {
  // Path sharp right, intersecting right and left
  TripLeg_Node node1;
  node1.mutable_edge()->set_begin_heading(352);
  node1.add_intersecting_edge()->set_begin_heading(355);
  node1.add_intersecting_edge()->set_begin_heading(270);
  node1.add_intersecting_edge()->set_begin_heading(180);
  node1.add_intersecting_edge()->set_begin_heading(90);
  node1.add_intersecting_edge()->set_begin_heading(10);
  TryCalculateRightLeftIntersectingEdgeCounts(180, std::make_unique<EnhancedTripLeg_Node>(&node1),
                                              IntersectingEdgeCounts(1, 1, 0, 0, 4, 0, 0, 0));
}

TEST(EnhancedTripPathCalculateRightLeftIntersectingEdgeCounts, SharpLeftRightLeft) {
  // Path sharp left, intersecting right and left
  TripLeg_Node node1;
  node1.mutable_edge()->set_begin_heading(10);
  node1.add_intersecting_edge()->set_begin_heading(90);
  node1.add_intersecting_edge()->set_begin_heading(180);
  node1.add_intersecting_edge()->set_begin_heading(270);
  node1.add_intersecting_edge()->set_begin_heading(352);
  node1.add_intersecting_edge()->set_begin_heading(355);
  node1.add_intersecting_edge()->set_begin_heading(5);
  TryCalculateRightLeftIntersectingEdgeCounts(180, std::make_unique<EnhancedTripLeg_Node>(&node1),
                                              IntersectingEdgeCounts(5, 0, 0, 0, 1, 1, 0, 0));
}

TEST(EnhancedTripPathDefaultTurnLaneState, True) {
  TripLeg_Edge edge;
  edge.add_turn_lanes()->set_directions_mask(kTurnLaneLeft);
  ASSERT_EQ(edge.mutable_turn_lanes(0)->state(), TurnLane::kInvalid);
}

void TryHasActiveTurnLane(std::unique_ptr<EnhancedTripLeg_Edge> edge, bool expected) {
  EXPECT_EQ(edge->HasActiveTurnLane(), expected);
}

TEST(EnhancedTripPathHasActiveTurnLane, False) {
  TripLeg_Edge edge;
  edge.add_turn_lanes()->set_directions_mask(kTurnLaneLeft);
  edge.add_turn_lanes()->set_directions_mask(kTurnLaneThrough);
  edge.add_turn_lanes()->set_directions_mask(kTurnLaneRight);
  TryHasActiveTurnLane(std::make_unique<EnhancedTripLeg_Edge>(&edge), false);
}

TEST(EnhancedTripPathHasActiveTurnLane, True) {
  TripLeg_Edge edge;
  edge.add_turn_lanes()->set_directions_mask(kTurnLaneLeft);
  edge.add_turn_lanes()->set_directions_mask(kTurnLaneThrough);
  edge.add_turn_lanes()->set_directions_mask(kTurnLaneRight);

  // Left active
  edge.mutable_turn_lanes(0)->set_state(TurnLane::kActive);
  TryHasActiveTurnLane(std::make_unique<EnhancedTripLeg_Edge>(&edge), true);

  // Straight active
  edge.mutable_turn_lanes(0)->set_state(TurnLane::kInvalid);
  edge.mutable_turn_lanes(1)->set_state(TurnLane::kActive);
  TryHasActiveTurnLane(std::make_unique<EnhancedTripLeg_Edge>(&edge), true);

  // Right active
  edge.mutable_turn_lanes(1)->set_state(TurnLane::kInvalid);
  edge.mutable_turn_lanes(2)->set_state(TurnLane::kActive);
  TryHasActiveTurnLane(std::make_unique<EnhancedTripLeg_Edge>(&edge), true);
}

void TryHasNonDirectionalTurnLane(std::unique_ptr<EnhancedTripLeg_Edge> edge, bool expected) {
  EXPECT_EQ(edge->HasNonDirectionalTurnLane(), expected);
}

TEST(EnhancedTripPathHasNonDirectionalTurnLane, False) {
  TripLeg_Edge edge;
  edge.add_turn_lanes()->set_directions_mask(kTurnLaneLeft);
  edge.add_turn_lanes()->set_directions_mask(kTurnLaneThrough);
  edge.add_turn_lanes()->set_directions_mask(kTurnLaneRight);
  TryHasNonDirectionalTurnLane(std::make_unique<EnhancedTripLeg_Edge>(&edge), false);
}

TEST(EnhancedTripPathHasNonDirectionalTurnLane, True) {
  TripLeg_Edge edge_1;
  edge_1.add_turn_lanes()->set_directions_mask(kTurnLaneLeft);
  edge_1.add_turn_lanes()->set_directions_mask(kTurnLaneNone);
  TryHasNonDirectionalTurnLane(std::make_unique<EnhancedTripLeg_Edge>(&edge_1), true);

  TripLeg_Edge edge_2;
  edge_2.add_turn_lanes()->set_directions_mask(kTurnLaneEmpty);
  edge_2.add_turn_lanes()->set_directions_mask(kTurnLaneRight);
  TryHasNonDirectionalTurnLane(std::make_unique<EnhancedTripLeg_Edge>(&edge_2), true);
}

void ClearActiveTurnLanes(::google::protobuf::RepeatedPtrField<::valhalla::TurnLane>* turn_lanes) {
  for (auto& turn_lane : *(turn_lanes)) {
    turn_lane.clear_state();
  }
}

void TryActivateTurnLanes(std::unique_ptr<EnhancedTripLeg_Edge> edge,
                          uint16_t turn_lane_direction,
                          float remaining_step_distance,
                          const DirectionsLeg_Maneuver_Type& curr_maneuver_type,
                          const DirectionsLeg_Maneuver_Type& next_maneuver_type,
                          uint16_t expected_activated_count) {
  uint16_t activated_count = edge->ActivateTurnLanes(turn_lane_direction, remaining_step_distance,
                                                     curr_maneuver_type, next_maneuver_type);
  EXPECT_EQ(activated_count, expected_activated_count)
      << "Incorrect activated count returned from ActivateTurnLanes(" +
             std::to_string(turn_lane_direction) + ", " + std::to_string(remaining_step_distance) +
             ", " + std::to_string(curr_maneuver_type) + ", " + std::to_string(next_maneuver_type) +
             ") - found: " + std::to_string(activated_count) +
             " | expected: " + std::to_string(expected_activated_count);
}

TEST(EnhancedTripPath, TestActivateTurnLanes) {
  //
  // Test various active angles
  //
  TripLeg_Edge edge_1;
  edge_1.add_turn_lanes()->set_directions_mask(kTurnLaneReverse);
  edge_1.add_turn_lanes()->set_directions_mask(kTurnLaneSharpLeft);
  edge_1.add_turn_lanes()->set_directions_mask(kTurnLaneLeft);
  edge_1.add_turn_lanes()->set_directions_mask(kTurnLaneLeft);
  edge_1.add_turn_lanes()->set_directions_mask(kTurnLaneLeft | kTurnLaneThrough);
  edge_1.add_turn_lanes()->set_directions_mask(kTurnLaneThrough);
  edge_1.add_turn_lanes()->set_directions_mask(kTurnLaneThrough);
  edge_1.add_turn_lanes()->set_directions_mask(kTurnLaneThrough | kTurnLaneRight);
  edge_1.add_turn_lanes()->set_directions_mask(kTurnLaneRight);
  edge_1.add_turn_lanes()->set_directions_mask(kTurnLaneSharpRight);

  float remaining_step_distance = 2.f; // kilometers
  DirectionsLeg_Maneuver_Type next_maneuver_type =
      DirectionsLeg_Maneuver_Type::DirectionsLeg_Maneuver_Type_kRight;

  // Reverse active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_1), kTurnLaneReverse,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kUturnLeft,
                       next_maneuver_type, 1);
  ClearActiveTurnLanes(edge_1.mutable_turn_lanes());

  // Sharp left active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_1), kTurnLaneSharpLeft,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kSharpLeft,
                       next_maneuver_type, 1);
  ClearActiveTurnLanes(edge_1.mutable_turn_lanes());

  // Left active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_1), kTurnLaneLeft,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kLeft, next_maneuver_type,
                       3);
  ClearActiveTurnLanes(edge_1.mutable_turn_lanes());

  // Slight left non-active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_1), kTurnLaneSlightLeft,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kSlightLeft,
                       next_maneuver_type, 0);
  ClearActiveTurnLanes(edge_1.mutable_turn_lanes());

  // Through active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_1), kTurnLaneThrough,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kContinue,
                       next_maneuver_type, 4);
  ClearActiveTurnLanes(edge_1.mutable_turn_lanes());

  // Slight right non-active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_1), kTurnLaneSlightRight,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kSlightRight,
                       next_maneuver_type, 0);
  ClearActiveTurnLanes(edge_1.mutable_turn_lanes());

  // Right active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_1), kTurnLaneRight,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kRight,
                       next_maneuver_type, 2);
  ClearActiveTurnLanes(edge_1.mutable_turn_lanes());

  // Sharp right active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_1), kTurnLaneSharpRight,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kSharpRight,
                       next_maneuver_type, 1);
  ClearActiveTurnLanes(edge_1.mutable_turn_lanes());

  //
  // Test slight left, through, and merge right
  //
  TripLeg_Edge edge_2;
  edge_2.add_turn_lanes()->set_directions_mask(kTurnLaneSlightLeft);
  edge_2.add_turn_lanes()->set_directions_mask(kTurnLaneSlightLeft);
  edge_2.add_turn_lanes()->set_directions_mask(kTurnLaneThrough);
  edge_2.add_turn_lanes()->set_directions_mask(kTurnLaneThrough);
  edge_2.add_turn_lanes()->set_directions_mask(kTurnLaneThrough);
  edge_2.add_turn_lanes()->set_directions_mask(kTurnLaneMergeToRight);

  // Slight left active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_2), kTurnLaneSlightLeft,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kSlightLeft,
                       next_maneuver_type, 2);
  ClearActiveTurnLanes(edge_2.mutable_turn_lanes());

  // Through active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_2), kTurnLaneThrough,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kContinue,
                       next_maneuver_type, 3);
  ClearActiveTurnLanes(edge_2.mutable_turn_lanes());

  // Merge-to-right active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_2), kTurnLaneMergeToRight,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kMergeRight,
                       next_maneuver_type, 1);
  ClearActiveTurnLanes(edge_2.mutable_turn_lanes());

  //
  // Test merge left, through, and slight right
  //
  TripLeg_Edge edge_3;
  edge_3.add_turn_lanes()->set_directions_mask(kTurnLaneMergeToLeft);
  edge_3.add_turn_lanes()->set_directions_mask(kTurnLaneThrough);
  edge_3.add_turn_lanes()->set_directions_mask(kTurnLaneThrough);
  edge_3.add_turn_lanes()->set_directions_mask(kTurnLaneThrough);
  edge_3.add_turn_lanes()->set_directions_mask(kTurnLaneSlightRight);
  edge_3.add_turn_lanes()->set_directions_mask(kTurnLaneSlightRight);

  // Merge-to-left active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_3), kTurnLaneMergeToLeft,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kMergeLeft,
                       next_maneuver_type, 1);
  ClearActiveTurnLanes(edge_3.mutable_turn_lanes());

  // Through active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_3), kTurnLaneThrough,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kContinue,
                       next_maneuver_type, 3);
  ClearActiveTurnLanes(edge_3.mutable_turn_lanes());

  // Slight right active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_3), kTurnLaneSlightRight,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kSlightRight,
                       next_maneuver_type, 2);
  ClearActiveTurnLanes(edge_3.mutable_turn_lanes());

  //
  // Test u-turn maneuver with left/right lane
  //
  TripLeg_Edge edge_4;
  edge_4.add_turn_lanes()->set_directions_mask(kTurnLaneLeft);
  edge_4.add_turn_lanes()->set_directions_mask(kTurnLaneLeft);
  edge_4.add_turn_lanes()->set_directions_mask(kTurnLaneThrough);
  edge_4.add_turn_lanes()->set_directions_mask(kTurnLaneThrough);
  edge_4.add_turn_lanes()->set_directions_mask(kTurnLaneThrough);
  edge_4.add_turn_lanes()->set_directions_mask(kTurnLaneRight);
  edge_4.add_turn_lanes()->set_directions_mask(kTurnLaneRight);

  // Both left turns active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_4), kTurnLaneLeft,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kLeft, next_maneuver_type,
                       2);
  ClearActiveTurnLanes(edge_4.mutable_turn_lanes());

  // Left most turn active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_4), kTurnLaneLeft,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kUturnLeft,
                       next_maneuver_type, 1);
  ClearActiveTurnLanes(edge_4.mutable_turn_lanes());

  // Both right turns active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_4), kTurnLaneRight,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kRight,
                       next_maneuver_type, 2);
  ClearActiveTurnLanes(edge_4.mutable_turn_lanes());

  // Right most turn active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_4), kTurnLaneRight,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kUturnRight,
                       next_maneuver_type, 1);
  ClearActiveTurnLanes(edge_4.mutable_turn_lanes());
}

TEST(EnhancedTripPath, TestActivateTurnLanesShortNextRight) {
  //
  // Test various active angles
  //
  TripLeg_Edge edge_1;
  edge_1.add_turn_lanes()->set_directions_mask(kTurnLaneReverse);
  edge_1.add_turn_lanes()->set_directions_mask(kTurnLaneSharpLeft);
  edge_1.add_turn_lanes()->set_directions_mask(kTurnLaneLeft);
  edge_1.add_turn_lanes()->set_directions_mask(kTurnLaneLeft);
  edge_1.add_turn_lanes()->set_directions_mask(kTurnLaneLeft | kTurnLaneThrough);
  edge_1.add_turn_lanes()->set_directions_mask(kTurnLaneThrough);
  edge_1.add_turn_lanes()->set_directions_mask(kTurnLaneThrough);
  edge_1.add_turn_lanes()->set_directions_mask(kTurnLaneThrough | kTurnLaneRight);
  edge_1.add_turn_lanes()->set_directions_mask(kTurnLaneRight);
  edge_1.add_turn_lanes()->set_directions_mask(kTurnLaneSharpRight);

  float remaining_step_distance = 0.1f; // kilometers
  DirectionsLeg_Maneuver_Type next_maneuver_type =
      DirectionsLeg_Maneuver_Type::DirectionsLeg_Maneuver_Type_kRight;

  // Reverse active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_1), kTurnLaneReverse,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kUturnLeft,
                       next_maneuver_type, 1);
  ClearActiveTurnLanes(edge_1.mutable_turn_lanes());

  // Sharp left active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_1), kTurnLaneSharpLeft,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kSharpLeft,
                       next_maneuver_type, 1);
  ClearActiveTurnLanes(edge_1.mutable_turn_lanes());

  // Left active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_1), kTurnLaneLeft,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kLeft, next_maneuver_type,
                       1);
  ClearActiveTurnLanes(edge_1.mutable_turn_lanes());

  // Slight left non-active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_1), kTurnLaneSlightLeft,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kSlightLeft,
                       next_maneuver_type, 0);
  ClearActiveTurnLanes(edge_1.mutable_turn_lanes());

  // Through active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_1), kTurnLaneThrough,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kContinue,
                       next_maneuver_type, 1);
  ClearActiveTurnLanes(edge_1.mutable_turn_lanes());

  // Slight right non-active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_1), kTurnLaneSlightRight,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kSlightRight,
                       next_maneuver_type, 0);
  ClearActiveTurnLanes(edge_1.mutable_turn_lanes());

  // Right active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_1), kTurnLaneRight,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kRight,
                       next_maneuver_type, 1);
  ClearActiveTurnLanes(edge_1.mutable_turn_lanes());

  // Sharp right active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_1), kTurnLaneSharpRight,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kSharpRight,
                       next_maneuver_type, 1);
  ClearActiveTurnLanes(edge_1.mutable_turn_lanes());

  //
  // Test slight left, through, and merge right
  //
  TripLeg_Edge edge_2;
  edge_2.add_turn_lanes()->set_directions_mask(kTurnLaneSlightLeft);
  edge_2.add_turn_lanes()->set_directions_mask(kTurnLaneSlightLeft);
  edge_2.add_turn_lanes()->set_directions_mask(kTurnLaneThrough);
  edge_2.add_turn_lanes()->set_directions_mask(kTurnLaneThrough);
  edge_2.add_turn_lanes()->set_directions_mask(kTurnLaneThrough);
  edge_2.add_turn_lanes()->set_directions_mask(kTurnLaneMergeToRight);

  // Slight left active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_2), kTurnLaneSlightLeft,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kSlightLeft,
                       next_maneuver_type, 1);
  ClearActiveTurnLanes(edge_2.mutable_turn_lanes());

  // Through active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_2), kTurnLaneThrough,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kContinue,
                       next_maneuver_type, 1);
  ClearActiveTurnLanes(edge_2.mutable_turn_lanes());

  // Merge-to-right active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_2), kTurnLaneMergeToRight,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kMergeRight,
                       next_maneuver_type, 1);
  ClearActiveTurnLanes(edge_2.mutable_turn_lanes());

  //
  // Test merge left, through, and slight right
  //
  TripLeg_Edge edge_3;
  edge_3.add_turn_lanes()->set_directions_mask(kTurnLaneMergeToLeft);
  edge_3.add_turn_lanes()->set_directions_mask(kTurnLaneThrough);
  edge_3.add_turn_lanes()->set_directions_mask(kTurnLaneThrough);
  edge_3.add_turn_lanes()->set_directions_mask(kTurnLaneThrough);
  edge_3.add_turn_lanes()->set_directions_mask(kTurnLaneSlightRight);
  edge_3.add_turn_lanes()->set_directions_mask(kTurnLaneSlightRight);

  // Merge-to-left active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_3), kTurnLaneMergeToLeft,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kMergeLeft,
                       next_maneuver_type, 1);
  ClearActiveTurnLanes(edge_3.mutable_turn_lanes());

  // Through active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_3), kTurnLaneThrough,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kContinue,
                       next_maneuver_type, 1);
  ClearActiveTurnLanes(edge_3.mutable_turn_lanes());

  // Slight right active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_3), kTurnLaneSlightRight,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kSlightRight,
                       next_maneuver_type, 1);
  ClearActiveTurnLanes(edge_3.mutable_turn_lanes());
}

TEST(EnhancedTripPath, TestActivateTurnLanesShortNextLeft) {
  //
  // Test various active angles
  //
  TripLeg_Edge edge_1;
  edge_1.add_turn_lanes()->set_directions_mask(kTurnLaneReverse);
  edge_1.add_turn_lanes()->set_directions_mask(kTurnLaneSharpLeft);
  edge_1.add_turn_lanes()->set_directions_mask(kTurnLaneLeft);
  edge_1.add_turn_lanes()->set_directions_mask(kTurnLaneLeft);
  edge_1.add_turn_lanes()->set_directions_mask(kTurnLaneLeft | kTurnLaneThrough);
  edge_1.add_turn_lanes()->set_directions_mask(kTurnLaneThrough);
  edge_1.add_turn_lanes()->set_directions_mask(kTurnLaneThrough);
  edge_1.add_turn_lanes()->set_directions_mask(kTurnLaneThrough | kTurnLaneRight);
  edge_1.add_turn_lanes()->set_directions_mask(kTurnLaneRight);
  edge_1.add_turn_lanes()->set_directions_mask(kTurnLaneSharpRight);

  float remaining_step_distance = 0.1f; // kilometers
  DirectionsLeg_Maneuver_Type next_maneuver_type =
      DirectionsLeg_Maneuver_Type::DirectionsLeg_Maneuver_Type_kLeft;

  // Reverse active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_1), kTurnLaneReverse,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kUturnLeft,
                       next_maneuver_type, 1);
  ClearActiveTurnLanes(edge_1.mutable_turn_lanes());

  // Sharp left active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_1), kTurnLaneSharpLeft,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kSharpLeft,
                       next_maneuver_type, 1);
  ClearActiveTurnLanes(edge_1.mutable_turn_lanes());

  // Left active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_1), kTurnLaneLeft,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kLeft, next_maneuver_type,
                       1);
  ClearActiveTurnLanes(edge_1.mutable_turn_lanes());

  // Slight left non-active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_1), kTurnLaneSlightLeft,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kSlightLeft,
                       next_maneuver_type, 0);
  ClearActiveTurnLanes(edge_1.mutable_turn_lanes());

  // Through active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_1), kTurnLaneThrough,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kContinue,
                       next_maneuver_type, 1);
  ClearActiveTurnLanes(edge_1.mutable_turn_lanes());

  // Slight right non-active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_1), kTurnLaneSlightRight,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kSlightRight,
                       next_maneuver_type, 0);
  ClearActiveTurnLanes(edge_1.mutable_turn_lanes());

  // Right active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_1), kTurnLaneRight,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kRight,
                       next_maneuver_type, 1);
  ClearActiveTurnLanes(edge_1.mutable_turn_lanes());

  // Sharp right active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_1), kTurnLaneSharpRight,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kSharpRight,
                       next_maneuver_type, 1);
  ClearActiveTurnLanes(edge_1.mutable_turn_lanes());

  //
  // Test slight left, through, and merge right
  //
  TripLeg_Edge edge_2;
  edge_2.add_turn_lanes()->set_directions_mask(kTurnLaneSlightLeft);
  edge_2.add_turn_lanes()->set_directions_mask(kTurnLaneSlightLeft);
  edge_2.add_turn_lanes()->set_directions_mask(kTurnLaneThrough);
  edge_2.add_turn_lanes()->set_directions_mask(kTurnLaneThrough);
  edge_2.add_turn_lanes()->set_directions_mask(kTurnLaneThrough);
  edge_2.add_turn_lanes()->set_directions_mask(kTurnLaneMergeToRight);

  // Slight left active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_2), kTurnLaneSlightLeft,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kSlightLeft,
                       next_maneuver_type, 1);
  ClearActiveTurnLanes(edge_2.mutable_turn_lanes());

  // Through active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_2), kTurnLaneThrough,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kContinue,
                       next_maneuver_type, 1);
  ClearActiveTurnLanes(edge_2.mutable_turn_lanes());

  // Merge-to-right active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_2), kTurnLaneMergeToRight,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kMergeRight,
                       next_maneuver_type, 1);
  ClearActiveTurnLanes(edge_2.mutable_turn_lanes());

  //
  // Test merge left, through, and slight right
  //
  TripLeg_Edge edge_3;
  edge_3.add_turn_lanes()->set_directions_mask(kTurnLaneMergeToLeft);
  edge_3.add_turn_lanes()->set_directions_mask(kTurnLaneThrough);
  edge_3.add_turn_lanes()->set_directions_mask(kTurnLaneThrough);
  edge_3.add_turn_lanes()->set_directions_mask(kTurnLaneThrough);
  edge_3.add_turn_lanes()->set_directions_mask(kTurnLaneSlightRight);
  edge_3.add_turn_lanes()->set_directions_mask(kTurnLaneSlightRight);

  // Merge-to-left active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_3), kTurnLaneMergeToLeft,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kMergeLeft,
                       next_maneuver_type, 1);
  ClearActiveTurnLanes(edge_3.mutable_turn_lanes());

  // Through active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_3), kTurnLaneThrough,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kContinue,
                       next_maneuver_type, 1);
  ClearActiveTurnLanes(edge_3.mutable_turn_lanes());

  // Slight right active
  TryActivateTurnLanes(std::make_unique<EnhancedTripLeg_Edge>(&edge_3), kTurnLaneSlightRight,
                       remaining_step_distance, DirectionsLeg_Maneuver_Type_kSlightRight,
                       next_maneuver_type, 1);
  ClearActiveTurnLanes(edge_3.mutable_turn_lanes());
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
