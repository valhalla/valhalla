#include "odin/enhancedtrippath.h"
#include "baldr/turnlanes.h"

#include <gtest/gtest.h>

#include <cstdint>

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

TEST(EnhancedTripPath, EdgeToString) {
  TripLeg_Edge edge;
  edge.add_name()->set_value("Main Street");
  auto* name = edge.add_name();
  name->set_value("US 1");
  name->mutable_pronunciation()->set_value("you es one");
  edge.set_length_km(1.5f);
  edge.set_use(TripLeg_Use_kRoadUse);
  edge.mutable_sign()->add_exit_numbers()->set_text("67B");
  edge.mutable_sign()->add_exit_toward_locations()->set_text("Harrisburg");
  edge.mutable_transit_route_info()->set_onestop_id("r-route");
  edge.mutable_transit_route_info()->set_operator_name("Metro");

  edge.add_turn_lanes()->set_directions_mask(kTurnLaneEmpty);
  edge.add_turn_lanes()->set_directions_mask(kTurnLaneNone);
  auto* lane = edge.add_turn_lanes();
  lane->set_directions_mask(kTurnLaneReverse | kTurnLaneSharpLeft | kTurnLaneLeft |
                            kTurnLaneSlightLeft | kTurnLaneMergeToLeft | kTurnLaneThrough |
                            kTurnLaneMergeToRight | kTurnLaneSlightRight | kTurnLaneRight |
                            kTurnLaneSharpRight);
  lane->set_state(TurnLane::kActive);
  lane->set_active_direction(kTurnLaneThrough);
  lane = edge.add_turn_lanes();
  lane->set_directions_mask(kTurnLaneRight);
  lane->set_state(TurnLane::kValid);

  EnhancedTripLeg_Edge enhanced(&edge);
  auto str = enhanced.ToString();
  EXPECT_NE(str.find("name=Main Street/US 1(you es one)"), std::string::npos) << str;
  EXPECT_NE(str.find("exit_numbers=67B"), std::string::npos) << str;
  EXPECT_NE(str.find("exit_toward_locations=Harrisburg"), std::string::npos) << str;
  EXPECT_NE(str.find("transit_route_info.onestop_id=r-route"), std::string::npos) << str;
  EXPECT_NE(str.find("transit_route_info.operator_name=Metro"), std::string::npos) << str;
  EXPECT_EQ(enhanced.TurnLanesToString(),
            "[ empty | none | reverse;sharp_left;left;slight_left;merge_to_left;*through*;"
            "merge_to_right;slight_right;right;sharp_right ACTIVE | right VALID ]");
  EXPECT_NE(str.find(enhanced.TurnLanesToString()), std::string::npos) << str;

  // left-hand traffic puts the u-turn lane on the right
  edge.set_drive_on_left(true);
  edge.clear_turn_lanes();
  edge.add_turn_lanes()->set_directions_mask(kTurnLaneSharpRight | kTurnLaneReverse);
  EXPECT_EQ(enhanced.TurnLanesToString(), "[ sharp_right;reverse ]");

  TripLeg_Edge unnamed;
  EXPECT_NE(EnhancedTripLeg_Edge(&unnamed).ToString().find("name=unnamed"), std::string::npos);
}

TEST(EnhancedTripPath, EdgeAccessors) {
  TripLeg_Edge edge;
  edge.add_name()->set_value("Main Street");
  auto* route = edge.add_name();
  route->set_value("US 1");
  route->set_is_route_number(true);
  edge.set_length_km(2.0f);

  EnhancedTripLeg_Edge enhanced(&edge);
  std::vector<std::pair<std::string, bool>> expected{{"Main Street", false}, {"US 1", true}};
  EXPECT_EQ(enhanced.GetNameList(), expected);
  EXPECT_FLOAT_EQ(enhanced.GetLength(Options::kilometers), 2.0f);
  EXPECT_NEAR(enhanced.GetLength(Options::miles), 1.24274f, 0.0001f);
}

TEST(EnhancedTripPath, EdgeUsePredicates) {
  TripLeg_Edge edge;
  EnhancedTripLeg_Edge enhanced(&edge);
  const std::vector<std::pair<TripLeg_Use, bool (EnhancedTripLeg_Edge::*)() const>> uses = {
      {TripLeg_Use_kTrackUse, &EnhancedTripLeg_Edge::IsTrackUse},
      {TripLeg_Use_kSidewalkUse, &EnhancedTripLeg_Edge::IsSidewalkUse},
      {TripLeg_Use_kPathUse, &EnhancedTripLeg_Edge::IsPathUse},
      {TripLeg_Use_kPedestrianUse, &EnhancedTripLeg_Edge::IsPedestrianUse},
      {TripLeg_Use_kBridlewayUse, &EnhancedTripLeg_Edge::IsBridlewayUse},
      {TripLeg_Use_kRestAreaUse, &EnhancedTripLeg_Edge::IsRestAreaUse},
      {TripLeg_Use_kServiceAreaUse, &EnhancedTripLeg_Edge::IsServiceAreaUse},
      {TripLeg_Use_kOtherUse, &EnhancedTripLeg_Edge::IsOtherUse},
      {TripLeg_Use_kConstructionUse, &EnhancedTripLeg_Edge::IsConstructionUse},
  };
  for (const auto& [use, is_use] : uses) {
    edge.set_use(use);
    EXPECT_TRUE((enhanced.*is_use)()) << TripLeg_Use_Name(use);
    edge.set_use(TripLeg_Use_kRoadUse);
    EXPECT_FALSE((enhanced.*is_use)()) << TripLeg_Use_Name(use);
  }
}

TEST(EnhancedTripPath, NodeTypePredicates) {
  TripLeg_Node node;
  EnhancedTripLeg_Node enhanced(&node);
  const std::vector<std::pair<TripLeg_Node_Type, bool (EnhancedTripLeg_Node::*)() const>> types = {
      {TripLeg_Node_Type_kStreetIntersection, &EnhancedTripLeg_Node::IsStreetIntersection},
      {TripLeg_Node_Type_kGate, &EnhancedTripLeg_Node::IsGate},
      {TripLeg_Node_Type_kBollard, &EnhancedTripLeg_Node::IsBollard},
      {TripLeg_Node_Type_kTollBooth, &EnhancedTripLeg_Node::IsTollBooth},
      {TripLeg_Node_Type_kTransitEgress, &EnhancedTripLeg_Node::IsTransitEgress},
      {TripLeg_Node_Type_kTransitStation, &EnhancedTripLeg_Node::IsTransitStation},
      {TripLeg_Node_Type_kTransitPlatform, &EnhancedTripLeg_Node::IsTransitPlatform},
      {TripLeg_Node_Type_kBikeShare, &EnhancedTripLeg_Node::IsBikeShare},
      {TripLeg_Node_Type_kParking, &EnhancedTripLeg_Node::IsParking},
      {TripLeg_Node_Type_kMotorwayJunction, &EnhancedTripLeg_Node::IsMotorwayJunction},
      {TripLeg_Node_Type_kBorderControl, &EnhancedTripLeg_Node::IsBorderControl},
      {TripLeg_Node_Type_kTollGantry, &EnhancedTripLeg_Node::IsTollGantry},
      {TripLeg_Node_Type_kSumpBuster, &EnhancedTripLeg_Node::IsSumpBuster},
      {TripLeg_Node_Type_kBuildingEntrance, &EnhancedTripLeg_Node::IsBuildingEntrance},
      {TripLeg_Node_Type_kElevator, &EnhancedTripLeg_Node::IsElevator},
  };
  for (const auto& [type, is_type] : types) {
    for (const auto& [other, unused] : types) {
      node.set_type(other);
      EXPECT_EQ((enhanced.*is_type)(), other == type)
          << TripLeg_Node_Type_Name(type) << " vs " << TripLeg_Node_Type_Name(other);
    }
  }
}

TEST(EnhancedTripPath, HasForwardIntersectingEdge) {
  TripLeg_Node node;
  node.add_intersecting_edge()->set_begin_heading(90);
  EnhancedTripLeg_Node enhanced(&node);
  EXPECT_FALSE(enhanced.HasForwardIntersectingEdge(0));
  node.add_intersecting_edge()->set_begin_heading(10);
  EXPECT_TRUE(enhanced.HasForwardIntersectingEdge(0));
}

TEST(EnhancedTripPath, NodeToString) {
  TripLeg_Node node;
  node.set_type(TripLeg_Node_Type_kTransitPlatform);
  node.set_time_zone("Europe/Berlin");
  auto* platform = node.mutable_transit_platform_info();
  platform->set_onestop_id("s-platform");
  platform->set_name("Hauptbahnhof");
  platform->set_station_name("Hbf");
  auto* xedge = node.add_intersecting_edge();
  xedge->set_begin_heading(123);
  xedge->set_lane_count(3);

  EnhancedTripLeg_Node enhanced(&node);
  auto str = enhanced.ToString();
  EXPECT_NE(str.find("transit_platform_info.onestop_id=s-platform"), std::string::npos) << str;
  EXPECT_NE(str.find("transit_platform_info.name=Hauptbahnhof"), std::string::npos) << str;
  EXPECT_NE(str.find("transit_platform_info.station_name=Hbf"), std::string::npos) << str;
  EXPECT_NE(str.find("time_zone=Europe/Berlin"), std::string::npos) << str;

  auto xedge_str = enhanced.GetIntersectingEdge(0)->ToString();
  EXPECT_NE(xedge_str.find("begin_heading=123"), std::string::npos) << xedge_str;
  EXPECT_NE(xedge_str.find("lane_count=3"), std::string::npos) << xedge_str;

  TripLeg_Node plain;
  EXPECT_EQ(EnhancedTripLeg_Node(&plain).ToString().find("transit_platform_info"), std::string::npos);
}

TEST(EnhancedTripPath, AdminToString) {
  TripLeg_Admin admin;
  admin.set_country_code("DE");
  admin.set_country_text("Germany");
  admin.set_state_code("BE");
  admin.set_state_text("Berlin");
  EXPECT_EQ(EnhancedTripLeg_Admin(&admin).ToString(),
            "country_code=DE | country_text=Germany | state_code=BE | state_text=Berlin");
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
