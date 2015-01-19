#include "test.h"
#include "valhalla/odin/maneuver.h"
#include "valhalla/odin/maneuversbuilder.h"

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

}

int main() {
  test::suite suite("maneuversbuilder");

  // SetSimpleDirectionalManeuverType
  suite.test(TEST_CASE(TestSetSimpleDirectionalManeuverType));

  return suite.tear_down();
}
