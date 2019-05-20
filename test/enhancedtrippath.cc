#include "odin/enhancedtrippath.h"
#include "midgard/util.h"

#include "test.h"
#include <cstdint>

using namespace std;
using namespace valhalla;
using namespace valhalla::odin;

namespace {

void TryCalculateRightLeftIntersectingEdgeCounts(
    uint32_t from_heading,
    std::unique_ptr<EnhancedTripLeg_Node> node,
    const IntersectingEdgeCounts& expected_xedge_counts,
    const TripLeg_TravelMode travel_mode = TripLeg_TravelMode_kDrive) {

  IntersectingEdgeCounts xedge_counts;
  xedge_counts.clear();

  node->CalculateRightLeftIntersectingEdgeCounts(from_heading, travel_mode, xedge_counts);
  if (xedge_counts.right != expected_xedge_counts.right) {
    throw std::runtime_error("Incorrect right");
  }

  if (xedge_counts.right_similar != expected_xedge_counts.right_similar) {
    throw std::runtime_error("Incorrect right_similar");
  }

  if (xedge_counts.right_traversable_outbound != expected_xedge_counts.right_traversable_outbound) {
    throw std::runtime_error("Incorrect right_driveable_outbound");
  }

  if (xedge_counts.right_similar_traversable_outbound !=
      expected_xedge_counts.right_similar_traversable_outbound) {
    throw std::runtime_error("Incorrect right_similar_driveable_outbound");
  }

  if (xedge_counts.left != expected_xedge_counts.left) {
    throw std::runtime_error("Incorrect left");
  }

  if (xedge_counts.left_similar != expected_xedge_counts.left_similar) {
    throw std::runtime_error("Incorrect left_similar");
  }

  if (xedge_counts.left_traversable_outbound != expected_xedge_counts.left_traversable_outbound) {
    throw std::runtime_error("Incorrect left_driveable_outbound");
  }

  if (xedge_counts.left_similar_traversable_outbound !=
      expected_xedge_counts.left_similar_traversable_outbound) {
    throw std::runtime_error("Incorrect left_similar_driveable_outbound");
  }
}

void TestCalculateRightLeftIntersectingEdgeCounts_Straight_Straight() {
  // Path straight, intersecting straight
  TripLeg_Node node1;
  node1.mutable_edge()->set_begin_heading(5);
  TripLeg_IntersectingEdge* n1_ie1 = node1.add_intersecting_edge();
  n1_ie1->set_begin_heading(355);
  n1_ie1->set_driveability(TripLeg_Traversability_kBoth);
  TryCalculateRightLeftIntersectingEdgeCounts(0, midgard::make_unique<EnhancedTripLeg_Node>(&node1),
                                              IntersectingEdgeCounts(0, 0, 0, 0, 1, 1, 1, 1));

  // Path straight, intersecting straight
  TripLeg_Node node2;
  node2.mutable_edge()->set_begin_heading(355);
  TripLeg_IntersectingEdge* n2_ie1 = node2.add_intersecting_edge();
  n2_ie1->set_begin_heading(5);
  n2_ie1->set_driveability(TripLeg_Traversability_kForward);
  TryCalculateRightLeftIntersectingEdgeCounts(0, midgard::make_unique<EnhancedTripLeg_Node>(&node2),
                                              IntersectingEdgeCounts(1, 1, 1, 1, 0, 0, 0, 0));
}

void TestCalculateRightLeftIntersectingEdgeCounts_SlightRight_Straight() {
  // Path slight right, intersecting straight
  TripLeg_Node node1;
  node1.mutable_edge()->set_begin_heading(11);
  TripLeg_IntersectingEdge* n1_ie1 = node1.add_intersecting_edge();
  n1_ie1->set_begin_heading(0);
  n1_ie1->set_driveability(TripLeg_Traversability_kBackward);
  TryCalculateRightLeftIntersectingEdgeCounts(0, midgard::make_unique<EnhancedTripLeg_Node>(&node1),
                                              IntersectingEdgeCounts(0, 0, 0, 0, 1, 1, 0, 0));

  // Path slight right, intersecting straight
  TripLeg_Node node2;
  node2.mutable_edge()->set_begin_heading(105);
  TripLeg_IntersectingEdge* n2_ie1 = node2.add_intersecting_edge();
  n2_ie1->set_begin_heading(85);
  n2_ie1->set_driveability(TripLeg_Traversability_kNone);
  TryCalculateRightLeftIntersectingEdgeCounts(90, midgard::make_unique<EnhancedTripLeg_Node>(&node2),
                                              IntersectingEdgeCounts(0, 0, 0, 0, 1, 1, 0, 0));
}

void TestCalculateRightLeftIntersectingEdgeCounts_SlightLeft_Straight() {
  // Path slight left, intersecting straight
  TripLeg_Node node1;
  node1.mutable_edge()->set_begin_heading(345);
  node1.add_intersecting_edge()->set_begin_heading(355);
  TryCalculateRightLeftIntersectingEdgeCounts(0, midgard::make_unique<EnhancedTripLeg_Node>(&node1),
                                              IntersectingEdgeCounts(1, 1, 0, 0, 0, 0, 0, 0));

  // Path slight left, intersecting straight
  TripLeg_Node node2;
  node2.mutable_edge()->set_begin_heading(255);
  node2.add_intersecting_edge()->set_begin_heading(275);
  TryCalculateRightLeftIntersectingEdgeCounts(270, midgard::make_unique<EnhancedTripLeg_Node>(&node2),
                                              IntersectingEdgeCounts(1, 1, 0, 0, 0, 0, 0, 0));
}

void TestCalculateRightLeftIntersectingEdgeCounts_SlightLeft_Right_Left() {
  // Path slight left, intersecting right and left
  TripLeg_Node node1;
  node1.mutable_edge()->set_begin_heading(340);
  node1.add_intersecting_edge()->set_begin_heading(45);
  node1.add_intersecting_edge()->set_begin_heading(90);
  node1.add_intersecting_edge()->set_begin_heading(135);
  node1.add_intersecting_edge()->set_begin_heading(315);
  node1.add_intersecting_edge()->set_begin_heading(270);
  node1.add_intersecting_edge()->set_begin_heading(225);
  TryCalculateRightLeftIntersectingEdgeCounts(0, midgard::make_unique<EnhancedTripLeg_Node>(&node1),
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
  TryCalculateRightLeftIntersectingEdgeCounts(80, midgard::make_unique<EnhancedTripLeg_Node>(&node2),
                                              IntersectingEdgeCounts(1, 0, 1, 0, 1, 0, 1, 0));
}

void TestCalculateRightLeftIntersectingEdgeCounts_SharpRight_Right_Left() {
  // Path sharp right, intersecting right and left
  TripLeg_Node node1;
  node1.mutable_edge()->set_begin_heading(352);
  node1.add_intersecting_edge()->set_begin_heading(355);
  node1.add_intersecting_edge()->set_begin_heading(270);
  node1.add_intersecting_edge()->set_begin_heading(180);
  node1.add_intersecting_edge()->set_begin_heading(90);
  node1.add_intersecting_edge()->set_begin_heading(10);
  TryCalculateRightLeftIntersectingEdgeCounts(180, midgard::make_unique<EnhancedTripLeg_Node>(&node1),
                                              IntersectingEdgeCounts(1, 1, 0, 0, 4, 0, 0, 0));
}

void TestCalculateRightLeftIntersectingEdgeCounts_SharpLeft_Right_Left() {
  // Path sharp left, intersecting right and left
  TripLeg_Node node1;
  node1.mutable_edge()->set_begin_heading(10);
  node1.add_intersecting_edge()->set_begin_heading(90);
  node1.add_intersecting_edge()->set_begin_heading(180);
  node1.add_intersecting_edge()->set_begin_heading(270);
  node1.add_intersecting_edge()->set_begin_heading(352);
  node1.add_intersecting_edge()->set_begin_heading(355);
  node1.add_intersecting_edge()->set_begin_heading(5);
  TryCalculateRightLeftIntersectingEdgeCounts(180, midgard::make_unique<EnhancedTripLeg_Node>(&node1),
                                              IntersectingEdgeCounts(5, 0, 0, 0, 1, 1, 0, 0));
}

} // namespace

int main() {
  test::suite suite("enhancedtrippath");

  // CalculateRightLeftIntersectingEdgeCounts_Straight_Straight
  suite.test(TEST_CASE(TestCalculateRightLeftIntersectingEdgeCounts_Straight_Straight));

  // CalculateRightLeftIntersectingEdgeCounts_SlightRight_Straight
  suite.test(TEST_CASE(TestCalculateRightLeftIntersectingEdgeCounts_SlightRight_Straight));

  // CalculateRightLeftIntersectingEdgeCounts_SlightLeft_Straight
  suite.test(TEST_CASE(TestCalculateRightLeftIntersectingEdgeCounts_SlightLeft_Straight));

  // CalculateRightLeftIntersectingEdgeCounts_SlightLeft_Right_Left
  suite.test(TEST_CASE(TestCalculateRightLeftIntersectingEdgeCounts_SlightLeft_Right_Left));

  // CalculateRightLeftIntersectingEdgeCounts_SharpRight_Right_Left
  suite.test(TEST_CASE(TestCalculateRightLeftIntersectingEdgeCounts_SharpRight_Right_Left));

  // CalculateRightLeftIntersectingEdgeCounts_SharpLeft_Right_Left
  suite.test(TEST_CASE(TestCalculateRightLeftIntersectingEdgeCounts_SharpLeft_Right_Left));

  return suite.tear_down();
}
