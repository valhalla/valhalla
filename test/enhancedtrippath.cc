#include "test.h"
#include "valhalla/odin/enhancedtrippath.h"

using namespace std;
using namespace valhalla::odin;

namespace {

void TryCalculateRightLeftIntersectingEdgeCounts(
    uint32_t from_heading, EnhancedTripPath_Node* node,
    uint32_t expected_right_count, uint32_t expected_right_similar_count,
    uint32_t expected_left_count, uint32_t expected_left_similar_count) {

  uint32_t right_count = 0;
  uint32_t right_similar_count = 0;
  uint32_t left_count = 0;
  uint32_t left_similar_count = 0;

  node->set_last_node(false);
  node->CalculateRightLeftIntersectingEdgeCounts(from_heading, right_count,
                                                 right_similar_count,
                                                 left_count,
                                                 left_similar_count);
  if (right_count != expected_right_count)
    throw std::runtime_error("Incorrect right_count");
  if (right_similar_count != expected_right_similar_count)
    throw std::runtime_error("Incorrect right_similar_count");
  if (left_count != expected_left_count)
    throw std::runtime_error("Incorrect left_count");
  if (left_similar_count != expected_left_similar_count)
    throw std::runtime_error("Incorrect left_similar_count");
}

void TestCalculateRightLeftIntersectingEdgeCounts_Straight_Straight() {
  // Path straight, intersecting straight
  TripPath_Node node1;
  node1.add_edge()->set_begin_heading(5);
  node1.add_edge()->set_begin_heading(355);
  TryCalculateRightLeftIntersectingEdgeCounts(
      0, static_cast<EnhancedTripPath_Node*>(&node1), 0, 0, 1, 1);

  // Path straight, intersecting straight
  TripPath_Node node2;
  node2.add_edge()->set_begin_heading(355);
  node2.add_edge()->set_begin_heading(5);
  TryCalculateRightLeftIntersectingEdgeCounts(
      0, static_cast<EnhancedTripPath_Node*>(&node2), 1, 1, 0, 0);

}

void TestCalculateRightLeftIntersectingEdgeCounts_SlightRight_Straight() {
  // Path slight right, intersecting straight
  TripPath_Node node1;
  node1.add_edge()->set_begin_heading(10);
  node1.add_edge()->set_begin_heading(0);
  TryCalculateRightLeftIntersectingEdgeCounts(
      0, static_cast<EnhancedTripPath_Node*>(&node1), 0, 0, 1, 1);

  // Path slight right, intersecting straight
  TripPath_Node node2;
  node2.add_edge()->set_begin_heading(105);
  node2.add_edge()->set_begin_heading(85);
  TryCalculateRightLeftIntersectingEdgeCounts(
      90, static_cast<EnhancedTripPath_Node*>(&node2), 0, 0, 1, 1);

}

void TestCalculateRightLeftIntersectingEdgeCounts_SlightLeft_Straight() {
  // Path slight left, intersecting straight
  TripPath_Node node1;
  node1.add_edge()->set_begin_heading(345);
  node1.add_edge()->set_begin_heading(355);
  TryCalculateRightLeftIntersectingEdgeCounts(
      0, static_cast<EnhancedTripPath_Node*>(&node1), 1, 1, 0, 0);

  // Path slight left, intersecting straight
  TripPath_Node node2;
  node2.add_edge()->set_begin_heading(255);
  node2.add_edge()->set_begin_heading(275);
  TryCalculateRightLeftIntersectingEdgeCounts(
      270, static_cast<EnhancedTripPath_Node*>(&node2), 1, 1, 0, 0);

}

void TestCalculateRightLeftIntersectingEdgeCounts_SlightLeft_Right_Left() {
  // Path slight left, intersecting right and left
  TripPath_Node node1;
  node1.add_edge()->set_begin_heading(340);
  node1.add_edge()->set_begin_heading(45);
  node1.add_edge()->set_begin_heading(90);
  node1.add_edge()->set_begin_heading(135);
  node1.add_edge()->set_begin_heading(315);
  node1.add_edge()->set_begin_heading(270);
  node1.add_edge()->set_begin_heading(225);
  TryCalculateRightLeftIntersectingEdgeCounts(
      0, static_cast<EnhancedTripPath_Node*>(&node1), 3, 0, 3, 1);

  // Path slight left, intersecting right and left
  TripPath_Node node2;
  node2.add_edge()->set_begin_heading(60);
  node2.add_edge()->set_begin_heading(157);
  node2.add_edge()->set_begin_heading(337);
  TryCalculateRightLeftIntersectingEdgeCounts(
      80, static_cast<EnhancedTripPath_Node*>(&node2), 1, 0, 1, 0);

}

void TestCalculateRightLeftIntersectingEdgeCounts_SharpRight_Right_Left() {
  // Path sharp right, intersecting right and left
  TripPath_Node node1;
  node1.add_edge()->set_begin_heading(352);
  node1.add_edge()->set_begin_heading(355);
  node1.add_edge()->set_begin_heading(270);
  node1.add_edge()->set_begin_heading(180);
  node1.add_edge()->set_begin_heading(90);
  node1.add_edge()->set_begin_heading(10);
  TryCalculateRightLeftIntersectingEdgeCounts(
      180, static_cast<EnhancedTripPath_Node*>(&node1), 1, 1, 4, 0);

}

void TestCalculateRightLeftIntersectingEdgeCounts_SharpLeft_Right_Left() {
  // Path sharp left, intersecting right and left
  TripPath_Node node1;
  node1.add_edge()->set_begin_heading(10);
  node1.add_edge()->set_begin_heading(90);
  node1.add_edge()->set_begin_heading(180);
  node1.add_edge()->set_begin_heading(270);
  node1.add_edge()->set_begin_heading(352);
  node1.add_edge()->set_begin_heading(355);
  node1.add_edge()->set_begin_heading(5);
  TryCalculateRightLeftIntersectingEdgeCounts(
      180, static_cast<EnhancedTripPath_Node*>(&node1), 5, 0, 1, 1);

}

}

int main() {
  test::suite suite("enhancedtrippath");

  // CalculateRightLeftIntersectingEdgeCounts_Straight_Straight
  suite.test(
      TEST_CASE(TestCalculateRightLeftIntersectingEdgeCounts_Straight_Straight)
      );

  // CalculateRightLeftIntersectingEdgeCounts_SlightRight_Straight
  suite.test(
      TEST_CASE(
          TestCalculateRightLeftIntersectingEdgeCounts_SlightRight_Straight)
      );

  // CalculateRightLeftIntersectingEdgeCounts_SlightLeft_Straight
  suite.test(
      TEST_CASE(
          TestCalculateRightLeftIntersectingEdgeCounts_SlightLeft_Straight)
      );

  // CalculateRightLeftIntersectingEdgeCounts_SlightLeft_Right_Left
  suite.test(
      TEST_CASE(
          TestCalculateRightLeftIntersectingEdgeCounts_SlightLeft_Right_Left)
      );

  // CalculateRightLeftIntersectingEdgeCounts_SharpRight_Right_Left
  suite.test(
      TEST_CASE(
          TestCalculateRightLeftIntersectingEdgeCounts_SharpRight_Right_Left)
      );

  // CalculateRightLeftIntersectingEdgeCounts_SharpLeft_Right_Left
  suite.test(
      TEST_CASE(
          TestCalculateRightLeftIntersectingEdgeCounts_SharpLeft_Right_Left)
      );

  return suite.tear_down();
}
