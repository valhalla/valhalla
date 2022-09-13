#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

class InstructionsFerryToward : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 1000;

    const std::string ascii_map = R"(
          A--B----C--D
    )";

    const gurka::ways ways = {{"AB", {{"highway", "trunk"}, {"name", ""}, {"ref", "A1"}}},
                              {"BC",
                               {{"motor_vehicle", "yes"},
                                {"motorcar", "yes"},
                                {"bicycle", "yes"},
                                {"foot", "yes"},
                                {"horse", "no"},
                                {"duration", "01:25"},
                                {"route", "ferry"},
                                {"operator", "Cape May-Lewes Ferry"},
                                {"name", "Cape May-Lewes Ferry"},
                                {"ref", "A1"},
                                {"destination:forward", "Cape May"},
                                {"destination:backward", "Lewes"}}},
                              {"CD", {{"highway", "trunk"}, {"name", ""}, {"ref", "A1"}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_instructions_ferry_toward",
                            {{"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map InstructionsFerryToward::map = {};

///////////////////////////////////////////////////////////////////////////////
// Take the ferry toward (forward)
// "3": "Take the ferry toward <TOWARD_SIGN>."
TEST_F(InstructionsFerryToward, TakeFerryTowardForward) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kFerryEnter,
                                                DirectionsLeg_Maneuver_Type_kFerryExit,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the ferry toward instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                            "Take the ferry toward Cape May.", "",
                                                            "Take the ferry toward Cape May.",
                                                            "Take the ferry toward Cape May.",
                                                            "Continue for 3 kilometers.");
}

///////////////////////////////////////////////////////////////////////////////
// Take the ferry toward (backward)
// "3": "Take the ferry toward <TOWARD_SIGN>."
TEST_F(InstructionsFerryToward, TakeFerryTowardBackward) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"D", "A"}, "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kFerryEnter,
                                                DirectionsLeg_Maneuver_Type_kFerryExit,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the ferry toward instructions
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                            "Take the ferry toward Lewes.", "",
                                                            "Take the ferry toward Lewes.",
                                                            "Take the ferry toward Lewes.",
                                                            "Continue for 3 kilometers.");
}
