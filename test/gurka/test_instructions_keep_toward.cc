#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

class InstructionsKeepToward : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 1000;

    const std::string ascii_map = R"(
                C--D
          A--B<
                E--F
    )";

    const gurka::ways ways = {{"AB", {{"highway", "motorway"}, {"name", ""}, {"ref", "A6"}}},
                              {"BC",
                               {{"highway", "motorway"},
                                {"name", ""},
                                {"ref", "A6"},
                                {"destination", "Harrisburg;Lancaster"},
                                {"destination:ref", "A6"}}},
                              {"CD", {{"highway", "motorway"}, {"name", ""}, {"ref", "A6"}}},
                              {"BE",
                               {{"highway", "motorway"},
                                {"name", ""},
                                {"ref", "A66"},
                                {"destination", "Baltimore;Washington"},
                                {"destination:ref", "A66"}}},
                              {"EF", {{"highway", "motorway"}, {"name", ""}, {"ref", "A66"}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_instructions_keep_toward",
                            {{"mjolnir.admin",
                              {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}});
  }
};

gurka::map InstructionsKeepToward::map = {};

///////////////////////////////////////////////////////////////////////////////
// Keep right toward
// "4": "Keep <RELATIVE_DIRECTION> toward <TOWARD_SIGN>."
TEST_F(InstructionsKeepToward, TurnRightToward) {
  auto result = gurka::route(map, "A", "F", "auto");

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStayRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  int maneuver_index = 1;

  // Verify the turn right toward instructions
  gurka::assert::raw::
      expect_instructions_at_maneuver_index(result, maneuver_index,
                                            "Keep right toward A66/Baltimore/Washington.",
                                            "Keep right toward A66.",
                                            "Keep right toward A66, Baltimore.",
                                            "Continue for 4 kilometers.");
}
