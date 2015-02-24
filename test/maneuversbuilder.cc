#include "test.h"
#include "valhalla/odin/maneuver.h"
#include "valhalla/odin/maneuversbuilder.h"
#include <valhalla/midgard/util.h>

using namespace std;
using namespace valhalla::odin;

namespace {

// Sub class to test protected methods
class ManeuversBuilderTest : public ManeuversBuilder {
 public:
  ManeuversBuilderTest()
      : ManeuversBuilder(nullptr) {
  }

  ManeuversBuilderTest(EnhancedTripPath* etp)
      : ManeuversBuilder(etp) {
  }

  void SetSimpleDirectionalManeuverType(Maneuver& maneuver) {
    ManeuversBuilder::SetSimpleDirectionalManeuverType(maneuver);
  }

  TripDirections_Maneuver_CardinalDirection DetermineCardinalDirection(
      uint32_t heading) {
    return ManeuversBuilder::DetermineCardinalDirection(heading);
  }

  void DetermineRelativeDirection(Maneuver& maneuver) {
    return ManeuversBuilder::DetermineRelativeDirection(maneuver);
  }

  static Maneuver::RelativeDirection DetermineRelativeDirection(
      uint32_t turn_degree) {
    return ManeuversBuilder::DetermineRelativeDirection(turn_degree);
  }

};

void TrySetSimpleDirectionalManeuverType(
    uint32_t turn_degree, TripDirections_Maneuver_Type expected) {
  ManeuversBuilderTest mbTest;
  Maneuver maneuver;
  maneuver.set_turn_degree(turn_degree);
  mbTest.SetSimpleDirectionalManeuverType(maneuver);
  if (maneuver.type() != expected)
    throw std::runtime_error("Incorrect maneuver type");
}

void TestSetSimpleDirectionalManeuverType() {
  // Continue lower bound
  TrySetSimpleDirectionalManeuverType(350,
                                      TripDirections_Maneuver_Type_kContinue);
  // Continue middle
  TrySetSimpleDirectionalManeuverType(0,
                                      TripDirections_Maneuver_Type_kContinue);
  // Continue upper bound
  TrySetSimpleDirectionalManeuverType(10,
                                      TripDirections_Maneuver_Type_kContinue);

  // Slight right lower bound
  TrySetSimpleDirectionalManeuverType(
      11, TripDirections_Maneuver_Type_kSlightRight);
  // Slight right middle
  TrySetSimpleDirectionalManeuverType(
      28, TripDirections_Maneuver_Type_kSlightRight);
  // Slight right upper bound
  TrySetSimpleDirectionalManeuverType(
      44, TripDirections_Maneuver_Type_kSlightRight);

  // Right lower bound
  TrySetSimpleDirectionalManeuverType(45, TripDirections_Maneuver_Type_kRight);
  // Right middle
  TrySetSimpleDirectionalManeuverType(90, TripDirections_Maneuver_Type_kRight);
  // Right upper bound
  TrySetSimpleDirectionalManeuverType(135, TripDirections_Maneuver_Type_kRight);

  // Sharp right lower bound
  TrySetSimpleDirectionalManeuverType(136,
                                      TripDirections_Maneuver_Type_kSharpRight);
  // Sharp right middle
  TrySetSimpleDirectionalManeuverType(158,
                                      TripDirections_Maneuver_Type_kSharpRight);
  // Sharp right upper bound
  TrySetSimpleDirectionalManeuverType(180,
                                      TripDirections_Maneuver_Type_kSharpRight);

  // Sharp left lower bound
  TrySetSimpleDirectionalManeuverType(181,
                                      TripDirections_Maneuver_Type_kSharpLeft);
  // Sharp left middle
  TrySetSimpleDirectionalManeuverType(203,
                                      TripDirections_Maneuver_Type_kSharpLeft);
  // Sharp left upper bound
  TrySetSimpleDirectionalManeuverType(224,
                                      TripDirections_Maneuver_Type_kSharpLeft);

  // Left lower bound
  TrySetSimpleDirectionalManeuverType(225, TripDirections_Maneuver_Type_kLeft);
  // Left middle
  TrySetSimpleDirectionalManeuverType(270, TripDirections_Maneuver_Type_kLeft);
  // Left upper bound
  TrySetSimpleDirectionalManeuverType(315, TripDirections_Maneuver_Type_kLeft);

  // Slight left lower bound
  TrySetSimpleDirectionalManeuverType(316,
                                      TripDirections_Maneuver_Type_kSlightLeft);
  // Slight left middle
  TrySetSimpleDirectionalManeuverType(333,
                                      TripDirections_Maneuver_Type_kSlightLeft);
  // Slight left upper bound
  TrySetSimpleDirectionalManeuverType(349,
                                      TripDirections_Maneuver_Type_kSlightLeft);

}

void TryDetermineCardinalDirection(
    uint32_t heading, TripDirections_Maneuver_CardinalDirection expected) {
  ManeuversBuilderTest mbTest;
  if (mbTest.DetermineCardinalDirection(heading) != expected)
    throw std::runtime_error("Incorrect cardinal direction");
}

void TestDetermineCardinalDirection() {
  // North lower bound
  TryDetermineCardinalDirection(
      337, TripDirections_Maneuver_CardinalDirection_kNorth);
  // North middle
  TryDetermineCardinalDirection(
      0, TripDirections_Maneuver_CardinalDirection_kNorth);
  // North upper bound
  TryDetermineCardinalDirection(
      23, TripDirections_Maneuver_CardinalDirection_kNorth);

  // Northeast lower bound
  TryDetermineCardinalDirection(
      24, TripDirections_Maneuver_CardinalDirection_kNorthEast);
  // Northeast middle
  TryDetermineCardinalDirection(
      45, TripDirections_Maneuver_CardinalDirection_kNorthEast);
  // Northeast upper bound
  TryDetermineCardinalDirection(
      66, TripDirections_Maneuver_CardinalDirection_kNorthEast);

  // East lower bound
  TryDetermineCardinalDirection(
      67, TripDirections_Maneuver_CardinalDirection_kEast);
  // East middle
  TryDetermineCardinalDirection(
      90, TripDirections_Maneuver_CardinalDirection_kEast);
  // East upper bound
  TryDetermineCardinalDirection(
      113, TripDirections_Maneuver_CardinalDirection_kEast);

  // Southeast lower bound
  TryDetermineCardinalDirection(
      114, TripDirections_Maneuver_CardinalDirection_kSouthEast);
  // Southeast middle
  TryDetermineCardinalDirection(
      135, TripDirections_Maneuver_CardinalDirection_kSouthEast);
  // Southeast upper bound
  TryDetermineCardinalDirection(
      156, TripDirections_Maneuver_CardinalDirection_kSouthEast);

  // South lower bound
  TryDetermineCardinalDirection(
      157, TripDirections_Maneuver_CardinalDirection_kSouth);
  // South middle
  TryDetermineCardinalDirection(
      180, TripDirections_Maneuver_CardinalDirection_kSouth);
  // South upper bound
  TryDetermineCardinalDirection(
      203, TripDirections_Maneuver_CardinalDirection_kSouth);

  // Southwest lower bound
  TryDetermineCardinalDirection(
      204, TripDirections_Maneuver_CardinalDirection_kSouthWest);
  // Southwest middle
  TryDetermineCardinalDirection(
      225, TripDirections_Maneuver_CardinalDirection_kSouthWest);
  // Southwest upper bound
  TryDetermineCardinalDirection(
      246, TripDirections_Maneuver_CardinalDirection_kSouthWest);

  // West lower bound
  TryDetermineCardinalDirection(
      247, TripDirections_Maneuver_CardinalDirection_kWest);
  // West middle
  TryDetermineCardinalDirection(
      270, TripDirections_Maneuver_CardinalDirection_kWest);
  // West upper bound
  TryDetermineCardinalDirection(
      293, TripDirections_Maneuver_CardinalDirection_kWest);

  // Northwest lower bound
  TryDetermineCardinalDirection(
      294, TripDirections_Maneuver_CardinalDirection_kNorthWest);
  // Northwest middle
  TryDetermineCardinalDirection(
      315, TripDirections_Maneuver_CardinalDirection_kNorthWest);
  // Northwest upper bound
  TryDetermineCardinalDirection(
      336, TripDirections_Maneuver_CardinalDirection_kNorthWest);

}

void TryDetermineRelativeDirection_Maneuver(
    uint32_t prev_heading, uint32_t curr_heading,
    const vector<uint32_t>& intersecting_headings,
    Maneuver::RelativeDirection expected) {
  TripPath path;
  TripPath_Node* node;

  // node:0
  node = path.add_node();
  node->add_edge()->set_end_heading(prev_heading);

  // node:1
  node = path.add_node();
  node->add_edge()->set_begin_heading(curr_heading);
  for (auto intersecting_heading : intersecting_headings)
    node->add_edge()->set_begin_heading(intersecting_heading);

  // node:2 dummy last node
  node = path.add_node();

  ManeuversBuilderTest mbTest(static_cast<EnhancedTripPath*>(&path));
  Maneuver maneuver;
  maneuver.set_begin_node_index(1);
  maneuver.set_turn_degree(
      valhalla::midgard::GetTurnDegree(prev_heading, curr_heading));
  mbTest.DetermineRelativeDirection(maneuver);
  if (maneuver.begin_relative_direction() != expected)
    throw std::runtime_error("Incorrect relative direction");
}

void TestDetermineRelativeDirection_Maneuver() {
  // Path straight, intersecting straight on the left - thus keep right
  TryDetermineRelativeDirection_Maneuver(
      0, 5, { 355 }, Maneuver::RelativeDirection::kKeepRight);

  // Path straight, intersecting straight on the right - thus keep left
  TryDetermineRelativeDirection_Maneuver(
      0, 355, { 5 }, Maneuver::RelativeDirection::kKeepLeft);

  // Path slight right, intersecting straight on the left - thus keep right
  TryDetermineRelativeDirection_Maneuver(
      0, 11, { 0 }, Maneuver::RelativeDirection::kKeepRight);

  // Path slight right, intersecting straight on the left - thus keep right
  TryDetermineRelativeDirection_Maneuver(
      90, 105, { 85 }, Maneuver::RelativeDirection::kKeepRight);

  // Path slight left, intersecting straight on the right - thus keep left
  TryDetermineRelativeDirection_Maneuver(
      0, 345, { 355 }, Maneuver::RelativeDirection::kKeepLeft);

  // Path slight left, intersecting straight on the right - thus keep left
  TryDetermineRelativeDirection_Maneuver(
      270, 255, { 275 }, Maneuver::RelativeDirection::kKeepLeft);

  // Path slight left, intersecting right and left - thus keep straight
  TryDetermineRelativeDirection_Maneuver(
      80, 60, { 157, 337 }, Maneuver::RelativeDirection::kKeepStraight);

  // Path sharp right, intersecting right and left - thus right
  TryDetermineRelativeDirection_Maneuver(180, 339, { 355, 270, 180, 90, 10 },
                                         Maneuver::RelativeDirection::kRight);

  // Path sharp left, intersecting right and left - thus left
  TryDetermineRelativeDirection_Maneuver(180, 21, { 90, 180, 270, 352, 355, 5 },
                                         Maneuver::RelativeDirection::kLeft);

  // Path reverse right, intersecting right and left - thus reverse
  TryDetermineRelativeDirection_Maneuver(180, 352, { 355, 270, 180, 90, 10 },
                                         Maneuver::RelativeDirection::KReverse);

  // Path reverse left, intersecting right and left - thus reverse
  TryDetermineRelativeDirection_Maneuver(180, 15, { 355, 270, 180, 90, 10 },
                                         Maneuver::RelativeDirection::KReverse);

}

void TryDetermineRelativeDirection(uint32_t turn_degree,
                                   Maneuver::RelativeDirection expected) {
  if (ManeuversBuilderTest::DetermineRelativeDirection(turn_degree) != expected)
    throw std::runtime_error("Incorrect relative direction");
}

void TestDetermineRelativeDirection() {
  // kKeepStraight lower bound
  TryDetermineRelativeDirection(330,
                                Maneuver::RelativeDirection::kKeepStraight);
  // kKeepStraight middle
  TryDetermineRelativeDirection(0, Maneuver::RelativeDirection::kKeepStraight);
  // kKeepStraight upper bound
  TryDetermineRelativeDirection(30, Maneuver::RelativeDirection::kKeepStraight);

  // kRight lower bound
  TryDetermineRelativeDirection(31, Maneuver::RelativeDirection::kRight);
  // kRight middle
  TryDetermineRelativeDirection(90, Maneuver::RelativeDirection::kRight);
  // kRight upper bound
  TryDetermineRelativeDirection(159, Maneuver::RelativeDirection::kRight);

  // KReverse lower bound
  TryDetermineRelativeDirection(160, Maneuver::RelativeDirection::KReverse);
  // KReverse middle
  TryDetermineRelativeDirection(180, Maneuver::RelativeDirection::KReverse);
  // KReverse upper bound
  TryDetermineRelativeDirection(200, Maneuver::RelativeDirection::KReverse);

  // kLeft lower bound
  TryDetermineRelativeDirection(201, Maneuver::RelativeDirection::kLeft);
  // kLeft middle
  TryDetermineRelativeDirection(270, Maneuver::RelativeDirection::kLeft);
  // kLeft upper bound
  TryDetermineRelativeDirection(329, Maneuver::RelativeDirection::kLeft);

}

}

int main() {
  test::suite suite("maneuversbuilder");

  // SetSimpleDirectionalManeuverType
  suite.test(TEST_CASE(TestSetSimpleDirectionalManeuverType));

  // DetermineCardinalDirection
  suite.test(TEST_CASE(TestDetermineCardinalDirection));

  // DetermineRelativeDirection_Maneuver
  suite.test(TEST_CASE(TestDetermineRelativeDirection_Maneuver));

  // DetermineRelativeDirection
  suite.test(TEST_CASE(TestDetermineRelativeDirection));

  return suite.tear_down();
}
